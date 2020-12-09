#include "InitialStage.hpp"

ldplab::rtscpu::InitialStageBoundingSpheres::InitialStageBoundingSpheres(
    std::shared_ptr<Context> context)
    :
    m_context{ context }
{
}

void ldplab::rtscpu::InitialStageBoundingSpheres::setup()
{
}

bool ldplab::rtscpu::InitialStageBoundingSpheres::createBatch(
    RayBuffer& initial_batch_buffer, size_t& particle_index)
{
    return false;
}
