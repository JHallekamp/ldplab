#include "ThreadPool.hpp"

ldplab::ThreadPool::BatchHandle::BatchHandle(
    std::shared_ptr<IJob> job, unsigned int size)
    :
    m_job{ std::move(job) },
    m_batch_size{ size },
    m_next_job_id{ 0 },
    m_running_jobs{ 0 },
    m_completed_jobs{ 0 }
{ }

void ldplab::ThreadPool::BatchHandle::runJob(bool& remove_batch_from_queue)
{
    unsigned int job_id;
    prepareJob(job_id, remove_batch_from_queue);
    if (remove_batch_from_queue)
        return;
    m_job->execute(job_id);
    finishJob();
}

void ldplab::ThreadPool::BatchHandle::prepareJob(
    unsigned int &id, bool& remove_batch_from_queue)
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
    std::unique_lock<std::mutex> lck{ m_mutex };
    if (m_next_job_id < m_batch_size || m_running_jobs > 0) 
        m_batch_join.wait(lck);
}

void ldplab::ThreadPool::BatchHandle::cancel()
{
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

ldplab::ThreadPool::ThreadPool(unsigned int size)
    :
    m_worker_threads{ size },
    m_threads_state{ State::threads_inactive }
{ }

ldplab::ThreadPool::~ThreadPool()
{
    terminate();
}

std::shared_ptr<ldplab::ThreadPool::BatchHandle>
    ldplab::ThreadPool::submitJobBatch(
        std::shared_ptr<IJob> job, unsigned int batch_size)
{
    if (batch_size < 1 || size() < 1)
        return nullptr;

    std::shared_ptr<BatchHandle> batch =
        std::make_shared<BatchHandle>(job, batch_size);

    std::unique_lock<std::mutex> lck{ m_thread_control_mutex };
    if (m_threads_state == State::threads_terminating)
        return nullptr;
    else if (m_threads_state == State::threads_inactive)
        startThreads();
    enqueueBatch(batch);
    lck.unlock();

    return batch;
}

void ldplab::ThreadPool::executeJobBatch(
    std::shared_ptr<IJob> job, unsigned int batch_size)
{
    if (batch_size < 1 || size() < 1)
        return;

    std::shared_ptr<BatchHandle> batch =
        std::make_shared<BatchHandle>(job, batch_size);

    std::unique_lock<std::mutex> lck{ m_thread_control_mutex };
    if (m_threads_state == State::threads_terminating)
        return;
    else if (m_threads_state == State::threads_inactive)
        startThreads();
    enqueueBatch(batch);
    lck.unlock();

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

    m_idle_worker.notify_all();
    for (size_t i = 0; i < m_worker_threads.size(); ++i)
        m_worker_threads[i].join();

    std::unique_lock<std::mutex> queue_lock{ m_queue_mutex };
    typedef std::list <std::shared_ptr<BatchHandle>>::iterator iterator;
    for (iterator it = m_batch_queue.begin(); it != m_batch_queue.end(); ++it)
        (*it)->cancel();
}

void ldplab::ThreadPool::workerThread()
{
    while (workerThreadRunning())
    {
        std::shared_ptr<BatchHandle> batch_handle =
            std::move(workerThreadGetCurrentBatch());
        if (batch_handle != nullptr)
        {
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

void ldplab::ThreadPool::workerThreadDequeueBatch(std::shared_ptr<BatchHandle> batch)
{
    std::unique_lock<std::mutex> lck{ m_queue_mutex };
    if (m_batch_queue.size() > 0 && m_batch_queue.front() == batch)
        m_batch_queue.pop_front();
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
