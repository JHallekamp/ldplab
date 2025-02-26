#include "ThreadPool.hpp"

#include "Log.hpp"
#include "Assert.hpp"

#include <string>
#include <sstream>

std::string getThreadIDString(std::thread::id tid)
{
    std::stringstream ss;
    ss << tid;
    return ss.str();
}

ldplab::utils::ThreadPool::BatchHandle::BatchHandle(
    std::shared_ptr<IJob> job, size_t size)
    :
    m_job{ std::move(job) },
    m_batch_size{ size },
    m_next_job_id{ 0 },
    m_running_jobs{ 0 },
    m_completed_jobs{ 0 }
{
    LDPLAB_ASSERT(size > 0);
    LDPLAB_LOG_DEBUG("LDPLAB thread pool batch %i: Created with %i jobs", 
        m_uid, size);
}

void ldplab::utils::ThreadPool::BatchHandle::runJob(
    bool& remove_batch_from_queue, 
    size_t thread_id)
{
    size_t job_id;
    prepareJob(job_id, remove_batch_from_queue);

    if (remove_batch_from_queue)
        return;

    LDPLAB_LOG_TRACE("LDPLAB thread pool batch %i: Thread %s executes job %i "\
        "(%i of %i)", 
        m_uid,
        getThreadIDString(std::this_thread::get_id()).c_str(),
        job_id, 
        job_id + 1, 
        m_batch_size);
    
    m_job->execute(job_id, m_batch_size, thread_id);
    
    LDPLAB_LOG_TRACE("LDPLAB thread pool batch %i: Thread %s finished job execution "\
        "%i (%i of %i)",
        m_uid, 
        getThreadIDString(std::this_thread::get_id()).c_str(),
        job_id, 
        job_id + 1, 
        m_batch_size);

    finishJob();
}

void ldplab::utils::ThreadPool::BatchHandle::prepareJob(
    size_t& id, bool& remove_batch_from_queue)
{
    std::unique_lock<std::mutex> lck{ m_mutex };
    remove_batch_from_queue = (m_next_job_id >= m_batch_size);
    id = m_next_job_id++;
    if (!remove_batch_from_queue)
        ++m_running_jobs;
}

void ldplab::utils::ThreadPool::BatchHandle::finishJob()
{
    std::unique_lock<std::mutex> lck{ m_mutex };
    ++m_completed_jobs;
    --m_running_jobs;
    if (m_running_jobs == 0 && m_next_job_id >= m_batch_size)
    {
        LDPLAB_LOG_DEBUG("LDPLAB thread pool batch %i: Batch execution finished",
            m_uid);
        m_batch_join.notify_all();
    }
}

ldplab::utils::ThreadPool::BatchHandle::~BatchHandle()
{
    LDPLAB_LOG_DEBUG("LDPLAB thread pool batch %i: Destroyed", m_uid);
}

void ldplab::utils::ThreadPool::BatchHandle::join()
{
    std::unique_lock<std::mutex> lck{ m_mutex };
    if (m_next_job_id < m_batch_size || m_running_jobs > 0)
    {
        LDPLAB_LOG_TRACE("LDPLAB thread pool batch %i: Join on thread %s",
            m_uid, getThreadIDString(std::this_thread::get_id()).c_str());
        m_batch_join.wait(lck);
    }
}

void ldplab::utils::ThreadPool::BatchHandle::cancel()
{
    LDPLAB_LOG_TRACE("LDPLAB thread pool batch %i: Batch canceled", m_uid);
    std::unique_lock<std::mutex> lck{ m_mutex };
    m_next_job_id = m_batch_size;
    if (m_running_jobs == 0)
        m_batch_join.notify_all();
}

ldplab::utils::ThreadPool::BatchHandle::State 
    ldplab::utils::ThreadPool::BatchHandle::state()
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

ldplab::utils::ThreadPool::ThreadPool(size_t size)
    :
    m_worker_threads{ size },
    m_threads_state{ State::threads_inactive }
{
    LDPLAB_ASSERT(size > 0);
    if (size > 0)
        LDPLAB_LOG_INFO("LDPLAB thread pool %i: Created with %i threads", m_uid, size);
    else
        LDPLAB_LOG_ERROR("LDPLAB thread pool %i: Not functional, number of threads "\
            "is 0", m_uid);
}

ldplab::utils::ThreadPool::~ThreadPool()
{
    if (m_threads_state != State::threads_inactive)
        terminate();    
    LDPLAB_LOG_INFO("LDPLAB thread pool %i: Instance destroyed", m_uid);
}

std::shared_ptr<ldplab::utils::ThreadPool::BatchHandle>
    ldplab::utils::ThreadPool::submitJobBatch(
        std::shared_ptr<IJob> job, size_t batch_size)
{
    if (batch_size < 1 || size() < 1)
    {
        if (batch_size < 1)
            LDPLAB_LOG_WARNING("LDPLAB thread pool %i: Reject submission of an empty "\
                "batch", m_uid);
        if (size() < 1)
            LDPLAB_LOG_ERROR("LDPLAB thread pool %i: Reject batch submission, thread "\
                "pool owns no threads");
        return nullptr;
    }

    std::shared_ptr<BatchHandle> batch{ 
        new BatchHandle{std::move(job), batch_size} };

    std::unique_lock<std::mutex> lck{ m_thread_control_mutex };
    if (m_threads_state == State::threads_terminating)
    {
        LDPLAB_LOG_WARNING("LDPLAB thread pool %i: Submission of batch %i failed "\
            "because thread pool is terminating.",
            m_uid, batch->m_uid);
        return nullptr;
    }
    else if (m_threads_state == State::threads_joining)
    {
        LDPLAB_LOG_WARNING("LDPLAB thread pool %i: Submission of batch %i failed "\
            "because thread pool is joining.",
            m_uid, batch->m_uid);
        return nullptr;
    }
    else if (m_threads_state == State::threads_inactive)
        startThreads();
    lck.unlock();

    enqueueBatch(batch);
    LDPLAB_LOG_DEBUG("LDPLAB thread pool %i: Batch %i submitted by thread %s",
        m_uid, 
        batch->m_uid, 
        getThreadIDString(std::this_thread::get_id()).c_str());

    return batch;
}

void ldplab::utils::ThreadPool::executeJobBatch(
    std::shared_ptr<IJob> job, size_t batch_size)
{
    if (batch_size < 1 || size() < 1)
    {
        if (batch_size < 1)
            LDPLAB_LOG_WARNING("LDPLAB thread pool %i: Reject execution of an empty "\
                "batch", m_uid);
        if (size() < 1)
            LDPLAB_LOG_ERROR("LDPLAB thread pool %i: Reject batch execution, thread "\
                "pool owns no threads");
        return;
    }

    std::shared_ptr<BatchHandle> batch{ 
        new BatchHandle{std::move(job), batch_size} };

    std::unique_lock<std::mutex> lck{ m_thread_control_mutex };
    if (m_threads_state == State::threads_terminating)
    {
        LDPLAB_LOG_DEBUG("LDPLAB thread pool %i: Submission of batch %i failed "\
            "because thread pool is terminating.",
            m_uid, batch->m_uid);
        return;
    }
    else if (m_threads_state == State::threads_joining)
    {
        LDPLAB_LOG_DEBUG("LDPLAB thread pool %i: Submission of batch %i failed "\
            "because thread pool is joining.",
            m_uid, batch->m_uid);
        return;
    }
    else if (m_threads_state == State::threads_inactive)
        startThreads();
    lck.unlock();
    
    enqueueBatch(batch);
    LDPLAB_LOG_DEBUG("LDPLAB thread pool %i: Batch %i submitted for synchronized "\
        "execution by thread %s", 
        m_uid, 
        batch->m_uid, 
        getThreadIDString(std::this_thread::get_id()).c_str());

    batch->join();
}

size_t ldplab::utils::ThreadPool::size() const
{
    return m_worker_threads.size();
}

size_t ldplab::utils::ThreadPool::numBatches()
{
    std::unique_lock<std::mutex> queue_lock{ m_queue_mutex };
    return m_batch_queue.size();
}

void ldplab::utils::ThreadPool::join()
{
    std::unique_lock<std::mutex> thread_lck{ m_thread_control_mutex };
    std::unique_lock<std::mutex> queue_lck{ m_queue_mutex };
    if (m_threads_state == State::threads_inactive ||
        m_threads_state == State::threads_terminating)
        return;
    m_threads_state = State::threads_joining;
    m_idle_worker.notify_all();
    queue_lck.unlock();
    thread_lck.unlock();

    LDPLAB_LOG_DEBUG("LDPLAB thread pool %i: Performs join on thread %s", 
        m_uid, getThreadIDString(std::this_thread::get_id()).c_str());
    LDPLAB_LOG_TRACE("LDPLAB thread pool %i: Joins batches on thread %s",
        m_uid, getThreadIDString(std::this_thread::get_id()).c_str());
    while (true)
    {
        std::shared_ptr<BatchHandle> batch = getUnjoinedBatch();
        if (batch == nullptr)
            break;
        else
            batch->join();
    }

    for (size_t i = 0; i < m_worker_threads.size(); ++i)
    {
        LDPLAB_LOG_TRACE("LDPLAB thread pool %i: Joins thread %s on thread %s",
            m_uid, 
            getThreadIDString(m_worker_threads[i].get_id()).c_str(),
            getThreadIDString(std::this_thread::get_id()).c_str());
        m_worker_threads[i].join();
    }

    LDPLAB_LOG_DEBUG("LDPLAB thread pool %i: Join complete on thread %s", 
        m_uid, getThreadIDString(std::this_thread::get_id()).c_str());
}

void ldplab::utils::ThreadPool::terminate()
{
    std::unique_lock<std::mutex> thread_lck{ m_thread_control_mutex };
    std::unique_lock<std::mutex> queue_lck{ m_queue_mutex };
    if (m_threads_state != State::threads_active)
        return;
    m_threads_state = State::threads_terminating;
    m_idle_worker.notify_all();
    queue_lck.unlock();
    thread_lck.unlock();

    LDPLAB_LOG_DEBUG("LDPLAB thread pool %i: Terminate", m_uid);
    for (size_t i = 0; i < m_worker_threads.size(); ++i)
    {
        LDPLAB_LOG_TRACE("LDPLAB thread pool %i: Termination joins thread %s",
            m_uid, getThreadIDString(m_worker_threads[i].get_id()).c_str());
        m_worker_threads[i].join();
    }

    LDPLAB_LOG_TRACE("LDPLAB thread pool %i: Termination joined on all threads", m_uid);
    LDPLAB_LOG_TRACE("LDPLAB thread pool %i: Termination cancels unfinished batches", 
        m_uid);

    queue_lck.lock();
    typedef std::list <std::shared_ptr<BatchHandle>>::iterator iterator;
    for (iterator it = m_batch_queue.begin(); it != m_batch_queue.end(); ++it)
        (*it)->cancel();
    m_batch_queue.clear();

    LDPLAB_LOG_DEBUG("LDPLAB thread pool %i: Terminated", m_uid);
}

void ldplab::utils::ThreadPool::workerThread(size_t thread_id)
{
    LDPLAB_LOG_DEBUG("LDPLAB thread pool %i: Thread %s started", 
        m_uid, getThreadIDString(std::this_thread::get_id()).c_str());
    while (workerThreadRunning())
    {
        std::shared_ptr<BatchHandle> batch_handle =
            std::move(workerThreadGetCurrentBatch());
        if (batch_handle != nullptr)
        {
            LDPLAB_LOG_TRACE("LDPLAB thread pool %i: Thread %s executes job "\
                "in batch %i",
                m_uid,
                getThreadIDString(std::this_thread::get_id()).c_str(),
                batch_handle->m_uid);

            bool remove_from_queue;
            batch_handle->runJob(remove_from_queue, thread_id);
            if (remove_from_queue)
            {
                LDPLAB_LOG_TRACE("LDPLAB thread pool %i: Thread %s had no job to "\
                    "execute in batch %i, batch will be removed from queue",
                    m_uid,
                    getThreadIDString(std::this_thread::get_id()).c_str(),
                    batch_handle->m_uid);
                workerThreadDequeueBatch(batch_handle);
            }
        }
        else if (!workerThreadCanIdle())
            break;
    }
    LDPLAB_LOG_DEBUG("LDPLAB thread pool %i: Thread %s stopped",
        m_uid, getThreadIDString(std::this_thread::get_id()).c_str());
}

void ldplab::utils::ThreadPool::enqueueBatch(std::shared_ptr<BatchHandle> batch)
{
    std::unique_lock<std::mutex> lck{ m_queue_mutex };
    m_batch_queue.push_back(batch);
    LDPLAB_LOG_TRACE("LDPLAB thread pool %i: Enqueued batch %i",
        m_uid, batch->m_uid);
    m_idle_worker.notify_all();
}

std::shared_ptr<ldplab::utils::ThreadPool::BatchHandle>
    ldplab::utils::ThreadPool::getUnjoinedBatch()
{
    std::unique_lock<std::mutex> queue_lck{ m_queue_mutex };
    if (m_batch_queue.size() > 0)
        return m_batch_queue.front();
    return nullptr;
}

void ldplab::utils::ThreadPool::startThreads()
{
    for (size_t i = 0; i < m_worker_threads.size(); ++i)
        m_worker_threads[i] = std::thread(&ThreadPool::workerThread, this, i);
    m_threads_state = State::threads_active;
}

void ldplab::utils::ThreadPool::workerThreadDequeueBatch(
    std::shared_ptr<BatchHandle> batch)
{
    std::unique_lock<std::mutex> lck{ m_queue_mutex };
    if (m_batch_queue.size() > 0 &&
        m_batch_queue.front()->m_uid == batch->m_uid)
    {
        m_batch_queue.pop_front();
        LDPLAB_LOG_DEBUG("LDPLAB thread pool %i: Thread %s dequeued batch %i",
            m_uid, 
            getThreadIDString(std::this_thread::get_id()).c_str(), 
            batch->m_uid);
    }
}

std::shared_ptr<ldplab::utils::ThreadPool::BatchHandle>
    ldplab::utils::ThreadPool::workerThreadGetCurrentBatch()
{
    std::unique_lock<std::mutex> queue_lck{ m_queue_mutex };
    if (m_batch_queue.size() > 0)
        return m_batch_queue.front();

    if (workerThreadCanIdle())
    {
        LDPLAB_LOG_TRACE("LDPLAB thread pool %i: Thread %s waits, no job available",
            m_uid, getThreadIDString(std::this_thread::get_id()).c_str());
        m_idle_worker.wait(queue_lck);
        LDPLAB_LOG_TRACE("LDPLAB thread pool %i: Thread %s woke up",
            m_uid, getThreadIDString(std::this_thread::get_id()).c_str());
    }
    return nullptr;
}

bool ldplab::utils::ThreadPool::workerThreadRunning()
{
    std::unique_lock<std::mutex> lck{ m_thread_control_mutex };
    return (m_threads_state != State::threads_terminating);
}

bool ldplab::utils::ThreadPool::workerThreadCanIdle()
{
    std::unique_lock<std::mutex> lck{ m_thread_control_mutex };
    return (m_threads_state != State::threads_joining &&
        m_threads_state != State::threads_terminating);
}
