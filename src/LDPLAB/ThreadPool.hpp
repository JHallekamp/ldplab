#ifndef WWU_LDPLAB_THREAD_POOL_HPP
#define WWU_LDPLAB_THREAD_POOL_HPP

#include <condition_variable>
#include <list>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace ldplab
{
    /**
     * @brief Manages a pool of threads.
     * @details A thread pool can be assigned jobs, which will be distributed
     *          to its threads.
     *          Idle threads sleep until they are assigned a job.
     */
    class ThreadPool
    {
    public:
        /** @brief Interface class for a job. */
        class IJob
        {
        public:
            /**
             * @brief Called by the thread pool to execute the job.
             * @param[in] job_id The ID of the job inside its batch (ranging
             *                   from 0 to batch size - 1).
             */
            virtual void execute(unsigned int job_id) = 0;
        };
        /** 
         * @brief Used to manage a batch of jobs that were submitted to the
         *        thread pool.
         */
        class BatchHandle
        {
            friend ThreadPool;
        public:
            /** @brief Represents the current state of the batch. */
            enum class State
            {
                /** @brief No job has been executed yet. */
                queued,
                /** @brief Currently there are jobs in processing. */
                in_process,
                /** @brief The batch has been completed. */
                completed,
                /** @brief The batch has been cancelled. */
                cancelled
            };
        public:
            /** 
             * @brief Stops execution of the calling thread until the batch
             *        is executed completely.
             */
            void join();
            /**
             * @brief Forfeits jobs that have not yet been executed.
             * @note Jobs that are currently executed during the call will
             *       NOT be terminated.
             */
            void cancel();
            /**
             * @brief Checks the current batch state.
             * @returns The current batch state.
             */
            State state();
        private:
            BatchHandle(std::shared_ptr<IJob> job, unsigned int size);
            void runJob(bool& remove_batch_from_queue);
            void prepareJob(
                unsigned int& job_id, bool& remove_batch_from_queue);
            void finishJob();
        private:
            std::mutex m_mutex;
            const std::shared_ptr<IJob> m_job;
            const unsigned int m_batch_size;
            unsigned int m_next_job_id;
            unsigned int m_running_jobs;
            unsigned int m_completed_jobs;
            std::condition_variable m_batch_join;
        };
    public:
        /**
         * @brief Constructs a thread pool of the given size.
         * @param[in] size The number of threads inside the pool. 
         * @warning Parameter size cannot be 0.
         */
        ThreadPool(unsigned int size);
        ~ThreadPool();
        /**
         * @brief Enqueues a batch of jobs which will be executed asynchronous
         *        to the calling thread.
         * @returns A shared pointer to the batch handle object for the 
         *          submitted batch of jobs.
         * @note To synchronize later, use the join method of the returned 
         *       batch handle.
         */
        std::shared_ptr<BatchHandle> submitJobBatch(
            std::shared_ptr<IJob> job, unsigned int batch_size);
        /**
         * @brief Enqueues a batch of jobs and waits until the batch is 
         *        finished.
         */
        void executeJobBatch(
            std::shared_ptr<IJob> job, unsigned int batch_size);
        /** @brief Returns the number of threads in the thread pool. */
        unsigned int size() const;
        /** @brief Returns the number of enqueued batches. */
        unsigned int numBatches();
        /** @brief Terminates the threads. */
        void terminate();
    private:
        void enqueueBatch(std::shared_ptr<BatchHandle> batch);
        void startThreads();
        void workerThread();
        void workerThreadDequeueBatch(std::shared_ptr<BatchHandle> batch);
        std::shared_ptr<BatchHandle> workerThreadGetCurrentBatch();
        bool workerThreadRunning();
    private:
        enum class State
        {
            threads_inactive,
            threads_active,
            threads_terminating
        };
    private:
        std::list<std::shared_ptr<BatchHandle>> m_batch_queue;
        std::mutex m_queue_mutex;
        std::mutex m_thread_control_mutex;
        std::vector<std::thread> m_worker_threads;
        State m_threads_state;
        std::condition_variable m_idle_worker;
    };
}

#endif