#ifndef WWU_LDPLAB_UTILS_THREAD_POOL_HPP
#define WWU_LDPLAB_UTILS_THREAD_POOL_HPP

#include <LDPLAB/UID.hpp>

#include <condition_variable>
#include <stddef.h>
#include <list>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace ldplab
{
    namespace utils
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
                 * @param[in] batch_size The number of jobs inside the batch to
                 *                       which this job belongs.
                 */
                virtual void execute(size_t job_id, size_t batch_size) = 0;
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
                ~BatchHandle();
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
                BatchHandle(std::shared_ptr<IJob> job, size_t size);
                void runJob(bool& remove_batch_from_queue);
                void prepareJob(
                    size_t& job_id, bool& remove_batch_from_queue);
                void finishJob();
            private:
                std::mutex m_mutex;
                const std::shared_ptr<IJob> m_job;
                const size_t m_batch_size;
                size_t m_next_job_id;
                size_t m_running_jobs;
                size_t m_completed_jobs;
                std::condition_variable m_batch_join;
                UID<BatchHandle> m_uid;
            };
        public:
            /**
             * @brief Constructs a thread pool of the given size.
             * @param[in] size The number of threads inside the pool.
             * @warning Parameter size cannot be 0.
             */
            ThreadPool(size_t size);
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
                std::shared_ptr<IJob> job, size_t batch_size);
            /**
             * @brief Enqueues a batch of jobs and waits until the batch is
             *        finished.
             */
            void executeJobBatch(
                std::shared_ptr<IJob> job, size_t batch_size);
            /** @brief Returns the number of threads in the thread pool. */
            size_t size() const;
            /** @brief Returns the number of enqueued batches. */
            size_t numBatches();
            /** @brief Joins all remaining batches before termination. */
            void join();
            /** @brief Terminates the threads. */
            void terminate();
        private:
            void enqueueBatch(std::shared_ptr<BatchHandle> batch);
            std::shared_ptr<BatchHandle> getUnjoinedBatch();
            void startThreads();
            void workerThread();
            void workerThreadDequeueBatch(std::shared_ptr<BatchHandle> batch);
            std::shared_ptr<BatchHandle> workerThreadGetCurrentBatch();
            bool workerThreadRunning();
            bool workerThreadCanIdle();
        private:
            enum class State
            {
                threads_inactive,
                threads_active,
                threads_terminating,
                threads_joining
            };
        private:
            std::list<std::shared_ptr<BatchHandle>> m_batch_queue;
            std::mutex m_queue_mutex;
            std::mutex m_thread_control_mutex;
            std::vector<std::thread> m_worker_threads;
            State m_threads_state;
            std::condition_variable m_idle_worker;
            UID<ThreadPool> m_uid;
        };
    }
}

#endif