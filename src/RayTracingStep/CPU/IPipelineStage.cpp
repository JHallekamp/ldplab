#include <LDPLAB/RayTracingStep/CPU/IPipelineStage.hpp>
#include "Factory.hpp"

ldplab::UID <ldplab::IRayTracingStep>
    ldplab::rtscpu::IPipelineStage::getParentRayTracingStepUID() const
{
    return m_parent_rts_uid;
}
