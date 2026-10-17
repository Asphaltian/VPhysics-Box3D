#include "physics_task_pool.h"

#include "box3d/constants.h"

#include <algorithm>

CTaskPool::CTaskPool( int nThreadCount )
: m_Tasks( std::make_unique<Task_t[]>( B3_MAX_TASKS ) )
{
	for ( int i = 1; i < std::min( nThreadCount, B3_MAX_WORKERS ); ++i )
		m_Workers.emplace_back( &CTaskPool::WorkerMain, this );
}

CTaskPool::~CTaskPool()
{
	m_bQuit.store( true );
	m_Semaphore.release( static_cast<std::ptrdiff_t>( m_Workers.size() ) );
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

	const int nSlot = m_nNextSlot.fetch_add( 1, std::memory_order_relaxed );
	if ( nSlot >= B3_MAX_TASKS )
	{
		pCallback( pContext );
		return nullptr;
	}

	Task_t &task = m_Tasks[nSlot];
	task.m_pCallback = pCallback;
	task.m_pContext = pContext;
	task.m_nStatus.store( TASK_PENDING, std::memory_order_release );
	m_Semaphore.release();
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

void CTaskPool::WorkerMain()
{
	for ( ;; )
	{
		m_Semaphore.acquire();
		if ( m_bQuit.load() )
			return;

		while ( ExecuteOne() )
		{
		}
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
