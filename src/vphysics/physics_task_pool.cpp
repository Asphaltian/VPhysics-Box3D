#include "physics_task_pool.h"

#include "box3d/constants.h"

#include <algorithm>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#else
#include <sched.h>
#include <fstream>
#include <set>
#include <string>
#endif

int CTaskPool::GetPhysicalCoreCount()
{
	int nCores = 0;
#ifdef _WIN32
	DWORD nBytes = 0;
	GetLogicalProcessorInformationEx( RelationProcessorCore, nullptr, &nBytes );
	std::vector<char> buffer( nBytes );
	if ( GetLogicalProcessorInformationEx( RelationProcessorCore, reinterpret_cast<SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX *>( buffer.data() ), &nBytes ) )
	{
		for ( DWORD nOffset = 0; nOffset < nBytes; nOffset += reinterpret_cast<const SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX *>( buffer.data() + nOffset )->Size )
			++nCores;
	}
#else
	cpu_set_t affinity;
	if ( sched_getaffinity( 0, sizeof( affinity ), &affinity ) == 0 )
	{
		std::set<std::string> cores;
		for ( int i = 0; i < CPU_SETSIZE; ++i )
		{
			if ( !CPU_ISSET( i, &affinity ) )
				continue;

			std::ifstream file( "/sys/devices/system/cpu/cpu" + std::to_string( i ) + "/topology/thread_siblings_list" );
			std::string siblings;
			cores.insert( std::getline( file, siblings ) ? siblings : std::to_string( i ) );
		}
		nCores = static_cast<int>( cores.size() );
	}
#endif
	return nCores > 0 ? nCores : static_cast<int>( std::thread::hardware_concurrency() );
}

CTaskPool::CTaskPool( int nThreadCount )
: m_Tasks( std::make_unique<Task_t[]>( B3_MAX_TASKS ) )
{
	for ( int i = 1; i < std::min( nThreadCount, B3_MAX_WORKERS ); ++i )
		m_Workers.emplace_back( &CTaskPool::WorkerMain, this );
}

CTaskPool::~CTaskPool()
{
	m_bQuit.store( true );
	{
		std::lock_guard lock( m_SignalMutex );
		m_nSignals += static_cast<int>( m_Workers.size() );
	}
	m_SignalCondition.notify_all();
	for ( std::thread &worker : m_Workers )
		worker.join();
}

void *CTaskPool::EnqueueTask( b3TaskCallback *pCallback, void *pTaskContext, void *pUserContext, const char * )
{
	return static_cast<CTaskPool *>( pUserContext )->Enqueue( pCallback, pTaskContext );
}

void CTaskPool::FinishTask( void *pUserTask, void *pUserContext )
{
	static_cast<CTaskPool *>( pUserContext )->Finish( static_cast<Task_t *>( pUserTask ) );
}

CTaskPool::Task_t *CTaskPool::Enqueue( b3TaskCallback *pCallback, void *pContext )
{
	if ( m_bSerial || m_Workers.empty() )
	{
		pCallback( pContext );
		return nullptr;
	}

	const int nSlot = m_nNextSlot.fetch_add( 1 );
	if ( nSlot >= B3_MAX_TASKS )
	{
		pCallback( pContext );
		return nullptr;
	}

	Task_t &task = m_Tasks[nSlot];
	task.m_pCallback = pCallback;
	task.m_pContext = pContext;
	task.m_nStatus.store( TASK_PENDING );
	Signal( 1 );
	return &task;
}

void CTaskPool::Finish( Task_t *pTask )
{
	if ( !pTask )
		return;

	while ( pTask->m_nStatus.load( std::memory_order_acquire ) != TASK_COMPLETE )
	{
		if ( !ExecuteOne() )
			std::this_thread::yield();
	}
}

bool CTaskPool::ExecuteOne()
{
	const int nCount = std::min( m_nNextSlot.load( std::memory_order_acquire ), B3_MAX_TASKS );
	for ( int i = 0; i < nCount; ++i )
	{
		Task_t &task = m_Tasks[i];
		int nExpected = TASK_PENDING;
		if ( task.m_nStatus.load( std::memory_order_acquire ) != TASK_PENDING || !task.m_nStatus.compare_exchange_strong( nExpected, TASK_CLAIMED, std::memory_order_acq_rel ) )
			continue;

		task.m_pCallback( task.m_pContext );
		task.m_nStatus.store( TASK_COMPLETE, std::memory_order_release );
		return true;
	}
	return false;
}

bool CTaskPool::HasPendingTask() const
{
	const int nCount = std::min( m_nNextSlot.load(), B3_MAX_TASKS );
	for ( int i = 0; i < nCount; ++i )
	{
		if ( m_Tasks[i].m_nStatus.load() == TASK_PENDING )
			return true;
	}
	return false;
}

void CTaskPool::Signal( int nCount )
{
	if ( m_nSleeping.load() == 0 )
		return;

	{
		std::lock_guard lock( m_SignalMutex );
		m_nSignals += nCount;
	}
	if ( nCount == 1 )
		m_SignalCondition.notify_one();
	else
		m_SignalCondition.notify_all();
}

void CTaskPool::WorkerMain()
{
	while ( !m_bQuit.load() )
	{
		if ( ExecuteOne() )
			continue;

		if ( m_bActive.load( std::memory_order_relaxed ) )
		{
			std::this_thread::yield();
			continue;
		}

		std::unique_lock lock( m_SignalMutex );
		m_nSleeping.fetch_add( 1 );
		if ( !HasPendingTask() )
		{
			m_SignalCondition.wait( lock, [this]() { return m_nSignals > 0; } );
			--m_nSignals;
		}
		m_nSleeping.fetch_sub( 1 );
	}
}

void CTaskPool::RunParallelFor( void *pContext )
{
	ParallelForTask_t &task = *static_cast<ParallelForTask_t *>( pContext );
	for ( ;; )
	{
		const int nIndex = task.m_pNextIndex->fetch_add( 1, std::memory_order_relaxed );
		if ( nIndex >= task.m_nCount )
			return;
		( *task.m_pFunction )( nIndex, task.m_nTask );
	}
}

void CTaskPool::ParallelFor( int nCount, const std::function<void( int nIndex, int nTask )> &function )
{
	if ( nCount <= 0 )
		return;

	if ( m_bSerial || m_Workers.empty() || nCount == 1 )
	{
		for ( int i = 0; i < nCount; ++i )
			function( i, 0 );
		return;
	}

	Reset();
	std::atomic<int> nNextIndex = 0;
	const int nTaskCount = std::min( nCount, GetThreadCount() );
	ParallelForTask_t tasks[B3_MAX_WORKERS];
	Task_t *handles[B3_MAX_WORKERS];
	for ( int i = 0; i < nTaskCount; ++i )
	{
		tasks[i] = { &function, &nNextIndex, nCount, i };
		handles[i] = i == 0 ? nullptr : Enqueue( &CTaskPool::RunParallelFor, &tasks[i] );
	}

	RunParallelFor( &tasks[0] );
	for ( int i = 1; i < nTaskCount; ++i )
		Finish( handles[i] );
}
