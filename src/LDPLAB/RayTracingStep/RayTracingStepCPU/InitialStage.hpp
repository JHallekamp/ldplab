#ifndef WWU_LDPLAB_RTSCPU_INITIAL_STAGE_HPP
#define WWU_LDPLAB_RTSCPU_INITIAL_STAGE_HPP

#include "RayTracingStepCPU.hpp"
#include "Context.hpp"

namespace ldplab
{
    namespace rtscpu
    {
        class InitialStage
        {
        public:
            InitialStage(
                std::shared_ptr<Context> context);
            void executeSetup();
            void execute(size_t particle);
        private:
            std::shared_ptr<Context> m_context;
        };
    }
}

#endif