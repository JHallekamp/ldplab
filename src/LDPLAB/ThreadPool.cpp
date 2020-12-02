#include "ThreadPool.hpp"

#include "Log.hpp"
#include "Utils/Assert.hpp"

#include <string>
#include <sstream>

std::string getThreadIDString(std::thread::id tid)
{
    std::stringstream ss;
    ss << tid;
    return ss.str();
}

ldplab::ThreadPool::BatchHandle::BatchHandle(
    std::unique_ptr<IJob> job, size_t size)
    :
    m_job{ std::move(job) },
    m_batch_size{ size },
    m_next_job_id{ 0 },
    m_running_jobs{ 0 },
    m_completed_jobs{ 0 }
{
    LDPLAB_ASSERT(size > 0);
    LDPLAB_LOG_INFO("Thread pool job batch %i: Created with %i jobs", 
        m_uid, size);
}

void ldplab::ThreadPool::BatchHandle::runJob(bool& remove_batch_from_queue)
{
    size_t job_id;
    prepareJob(job_id, remove_batch_from_queue);

    if (remove_batch_from_queue)
        return;

    LDPLAB_LOG_DEBUG("Thread pool job batch %i: Executes job %i (%i of %i)", 
        m_uid, job_id, job_id + 1, m_batch_size);
    m_job->execute(job_id);
    finishJob();
    LDPLAB_LOG_DEBUG("Thread pool job batch %i: Finished job %i (%i of %i)",
        m_uid, job_id, job_id + 1, m_batch_size);
}

void ldplab::ThreadPool::BatchHandle::prepareJob(
    size_t& id, bool& remove_batch_from_queue)
{
    std::unique_lock<std::mutex> lck{ m_mutex };
    remove_batch_from_queue = (m_next_job_id >= m_batch_size);
    id = m_next_job_id++;
    ++m_running_jobs;
}

void ldplab::ThreadPool::BatchHandle::finishJob()
{
    std::unique_lock<std::mutex> lck{ m_mutex };
    ++m_completed_jobs;
    --m_running_jobs;
    if (m_running_jobs == 0 && m_next_job_id >= m_batch_size)
        m_batch_join.notify_all();
}

void ldplab::ThreadPool::BatchHandle::join()
{
    LDPLAB_LOG_DEBUG("Thread pool job batch %i: Join on thread %s", 
        m_uid, getThreadIDString(std::this_thread::get_id()));
    std::unique_lock<std::mutex> lck{ m_mutex };
    if (m_next_job_id < m_batch_size || m_running_jobs > 0) 
        m_batch_join.wait(lck);
}

void ldplab::ThreadPool::BatchHandle::cancel()
{
    LDPLAB_LOG_DEBUG("Thread pool job batch %i: Batch canceled", m_uid);
    std::unique_lock<std::mutex> lck{ m_mutex };
    m_next_job_id = m_batch_size;
    if (m_running_jobs == 0)
        m_batch_join.notify_all();
}

ldplab::ThreadPool::BatchHandle::State ldplab::ThreadPool::BatchHandle::state()
{
    std::unique_lock<std::mutex> lck{ m_mutex };
    if (m_running_jobs == 0 && m_completed_jobs == 0)
        return State::queued;
    else if (m_next_job_id >= m_batch_size && m_completed_jobs == m_batch_size)
        return State::completed;
    else if (m_next_job_id >= m_batch_size)
        return State::cancelled;
    else
        return State::in_process;
}

ldplab::ThreadPool::ThreadPool(size_t size)
    :
    m_worker_threads{ size },
    m_threads_state{ State::threads_inactive }
{
    LDPLAB_ASSERT(size > 0);
    LDPLAB_LOG_INFO("Thread pool %i: Created with %i threads", m_uid, size);
}

ldplab::ThreadPool::~ThreadPool()
{
    terminate();
    LDPLAB_LOG_INFO("Thread pool %i: Instance destroyed", m_uid);
}

std::shared_ptr<ldplab::ThreadPool::BatchHandle>
    ldplab::ThreadPool::submitJobBatch(
        std::unique_ptr<IJob> job, size_t batch_size)
{
    if (batch_size < 1 || size() < 1)
        return nullptr;

    std::shared_ptr<BatchHandle> batch{ 
        new BatchHandle{std::move(job), batch_size} };

    std::unique_lock<std::mutex> lck{ m_thread_control_mutex };
    if (m_threads_state == State::threads_terminating)
        return nullptr;
    else if (m_threads_state == State::threads_inactive)
        startThreads();
    enqueueBatch(batch);
    lck.unlock();

    LDPLAB_LOG_DEBUG("Thread pool %i: Batch %i submitted",
        m_uid, batch->m_uid);

    return batch;
}

void ldplab::ThreadPool::executeJobBatch(
    std::unique_ptr<IJob> job, size_t batch_size)
{
    if (batch_size < 1 || size() < 1)
        return;

    std::shared_ptr<BatchHandle> batch{ 
        new BatchHandle{std::move(job), batch_size} };

    std::unique_lock<std::mutex> lck{ m_thread_control_mutex };
    if (m_threads_state == State::threads_terminating)
        return;
    else if (m_threads_state == State::threads_inactive)
        startThreads();
    enqueueBatch(batch);
    lck.unlock();

    LDPLAB_LOG_DEBUG("Thread pool %i: Batch %i submitted for synchronized "\
        "execution", m_uid, batch->m_uid);

    batch->join();
}

unsigned int ldplab::ThreadPool::size() const
{
    return m_worker_threads.size();
}

unsigned int ldplab::ThreadPool::numBatches()
{
    std::unique_lock<std::mutex> queue_lock{ m_queue_mutex };
    return m_batch_queue.size();
}

void ldplab::ThreadPool::terminate()
{
    std::unique_lock<std::mutex> thread_lck{ m_thread_control_mutex };
    if (m_threads_state != State::threads_active)
        return;
    m_threads_state = State::threads_terminating;
    thread_lck.unlock();

    LDPLAB_LOG_DEBUG("Thread pool %i: Terminate", m_uid);

    m_idle_worker.notify_all();
    for (size_t i = 0; i < m_worker_threads.size(); ++i)
    {
        LDPLAB_LOG_DEBUG("Thread pool %i: Termination joins thread %s",
            m_uid, m_worker_threads[i].get_id());
        m_worker_threads[i].join();
    }

    LDPLAB_LOG_DEBUG("Thread pool %i: Termination joined all threads", m_uid);
    LDPLAB_LOG_DEBUG("Thread pool %i: Termination cancels unfinished batches", 
        m_uid);

    std::unique_lock<std::mutex> queue_lock{ m_queue_mutex };
    typedef std::list <std::shared_ptr<BatchHandle>>::iterator iterator;
    for (iterator it = m_batch_queue.begin(); it != m_batch_queue.end(); ++it)
        (*it)->cancel();

    LDPLAB_LOG_DEBUG("Thread pool %i: Termination completed", m_uid);
}

void ldplab::ThreadPool::workerThread()
{
    LDPLAB_LOG_DEBUG("Thread pool %i: Thread %s started", 
        m_uid, getThreadIDString(std::this_thread::get_id()));
    while (workerThreadRunning())
    {
        std::shared_ptr<BatchHandle> batch_handle =
            std::move(workerThreadGetCurrentBatch());
        if (batch_handle != nullptr)
        {
            LDPLAB_LOG_DEBUG("Thread pool %i: Thread %s executes job "\
                "in batch %i",
                m_uid, 
                getThreadIDString(std::this_thread::get_id()), 
                batch_handle->m_uid);

            bool remove_from_queue;
            batch_handle->runJob(remove_from_queue);
            if (remove_from_queue)
                workerThreadDequeueBatch(batch_handle);
        }
    }
}

void ldplab::ThreadPool::enqueueBatch(std::shared_ptr<BatchHandle> batch)
{
    std::unique_lock<std::mutex> lck{ m_queue_mutex };
    m_batch_queue.push_back(batch);
    m_idle_worker.notify_all();
}

void ldplab::ThreadPool::startThreads()
{
    for (size_t i = 0; i < m_worker_threads.size(); ++i)
        m_worker_threads[i] = std::thread(&ThreadPool::workerThread, this);
}

void ldplab::ThreadPool::workerThreadDequeueBatch(
    std::shared_ptr<BatchHandle> batch)
{
    std::unique_lock<std::mutex> lck{ m_queue_mutex };
    if (m_batch_queue.size() > 0 && m_batch_queue.front() == batch)
        m_batch_queue.pop_front();

    LDPLAB_LOG_DEBUG("Thread pool %i: Thread %s dequeued batch %i",
        m_uid, getThreadIDString(std::this_thread::get_id()), batch->m_uid);
}

std::shared_ptr<ldplab::ThreadPool::BatchHandle> 
    ldplab::ThreadPool::workerThreadGetCurrentBatch()
{
    std::unique_lock<std::mutex> queue_lck{ m_queue_mutex };
    if (m_batch_queue.size() > 0)
        return m_batch_queue.front();
    queue_lck.unlock();

    std::unique_lock<std::mutex> thread_lck{ m_thread_control_mutex };
    if (m_threads_state == State::threads_active)
        m_idle_worker.wait(thread_lck);
    return nullptr;
}

bool ldplab::ThreadPool::workerThreadRunning()
{
    std::unique_lock<std::mutex> lck{ m_thread_control_mutex };
    if (m_threads_state != State::threads_active)
        return false;
    return true;
}
