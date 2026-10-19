#pragma once

#include "box3d/types.h"

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

class CTaskPool
{
public:
	explicit CTaskPool( int nThreadCount );
	~CTaskPool();

	CTaskPool( const CTaskPool & ) = delete;
	CTaskPool &operator=( const CTaskPool & ) = delete;

	int GetThreadCount() const { return static_cast<int>( m_Workers.size() ) + 1; }

	void SetSerial( bool bSerial ) { m_bSerial = bSerial; }

	bool IsSerial() const { return m_bSerial; }

	void Reset() { m_nNextSlot.store( 0, std::memory_order_relaxed ); }

	void ParallelFor( int nCount, const std::function<void( int nIndex, int nTask )> &function );

	static void *EnqueueTask( b3TaskCallback *pCallback, void *pTaskContext, void *pUserContext, const char *pName );
	static void FinishTask( void *pUserTask, void *pUserContext );

private:
	enum Status_t
	{
		TASK_FREE,
		TASK_PENDING,
		TASK_CLAIMED,
		TASK_COMPLETE,
	};

	struct Task_t
	{
		b3TaskCallback *m_pCallback = nullptr;
		void *m_pContext = nullptr;
		std::atomic<int> m_nStatus = TASK_FREE;
	};

	struct ParallelForTask_t
	{
		const std::function<void( int, int )> *m_pFunction;
		std::atomic<int> *m_pNextIndex;
		int m_nCount;
		int m_nTask;
	};

	static void RunParallelFor( void *pContext );

	Task_t *Enqueue( b3TaskCallback *pCallback, void *pContext );
	void Finish( Task_t *pTask );
	bool ExecuteOne();
	void WorkerMain();
	void Signal( int nCount );

	std::vector<std::thread> m_Workers;
	std::unique_ptr<Task_t[]> m_Tasks;
	std::atomic<int> m_nNextSlot = 0;
	std::mutex m_SignalMutex;
	std::condition_variable m_SignalCondition;
	int m_nSignals = 0;
	std::atomic<bool> m_bQuit = false;
	bool m_bSerial = false;
};
