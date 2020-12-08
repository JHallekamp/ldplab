#include "RayTracingStepCPUInitialStage.hpp"

ldplab::RayTracingStepCPUInitialStage::RayTracingStepCPUInitialStage(
    std::shared_ptr<RayTracingStepCPUContext> context)
    :
    m_context{ context }
{ }

void ldplab::RayTracingStepCPUInitialStage::executeSetup()
{
}

void ldplab::RayTracingStepCPUInitialStage::execute(size_t particle)
{
}