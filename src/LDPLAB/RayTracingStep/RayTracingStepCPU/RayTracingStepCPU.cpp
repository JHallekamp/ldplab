#include "RayTracingStepCPU.hpp"

#include "Context.hpp"

ldplab::rtscpu::RayTracingStepCPU::RayTracingStepCPU(
    std::shared_ptr<Context> context) 
    :
    m_context{context}
{
}