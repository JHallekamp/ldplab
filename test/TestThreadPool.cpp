#include <ldplab.hpp>
#include <LDPLAB/Utils/ThreadPool.hpp>

class Job : public ldplab::utils::ThreadPool::IJob
{
public:
    Job(unsigned int offset)
        :
        m_offset{ offset }
    { }
    
    void execute(size_t job_id) override
    {
        int a = 0xDEAD;
        int b = 0xBEEF;
        int c = a * b;
    }
private:
    unsigned int m_offset;
};

int main()
{
    ldplab::LogCallbackFileStream flog{ "test_thread_pool.log" };
    ldplab::LogCallbackStdout clog{};
    flog.setLogLevel(ldplab::LOG_LEVEL_TRACE);
    clog.setLogLevel(ldplab::LOG_LEVEL_DEBUG);
    flog.subscribe();
    clog.subscribe();

    constexpr unsigned int num_threads = 4;
    constexpr unsigned int batch_size = 16;
    constexpr unsigned int num_batches = 4;

    // Create a thread pool with num_threads threads.
    ldplab::utils::ThreadPool tp0{ num_threads };
    ldplab::utils::ThreadPool tp1{ num_threads };

    // Executes a batch synchronously.
    unsigned int jobctr = 0;
    tp0.executeJobBatch(std::make_unique<Job>(jobctr++ * batch_size), batch_size);

    // Submit multiple batches in parrallel
    for (unsigned int i = 0; i < num_batches; ++i)
    {
        tp0.submitJobBatch(std::make_unique<Job>(jobctr++ * batch_size), batch_size);
        tp1.submitJobBatch(std::make_unique<Job>(jobctr++ * batch_size), batch_size);
    }
    tp0.join();
    tp1.terminate();

    return 0;
}