#include "InitialStage.hpp"

ldplab::rtscpu::InitialStageBoundingSpheresHomogenousLight::
    InitialStageBoundingSpheresHomogenousLight(
        std::shared_ptr<Context> context)
    :
    m_context{ context }
{
}

void ldplab::rtscpu::InitialStageBoundingSpheresHomogenousLight::setup()
{
    // Projections per light source
    std::vector<std::vector<Projection>> projection_per_light_source;

    for (size_t i = 0; i < m_context->light_sources.size(); ++i)
    {
        
    }
}

bool ldplab::rtscpu::InitialStageBoundingSpheresHomogenousLight::createBatch(
    RayBuffer& initial_batch_buffer, size_t& particle_index)
{
    return false;
}
