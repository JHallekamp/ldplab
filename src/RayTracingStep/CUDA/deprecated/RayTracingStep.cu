#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "RayTracingStep.hpp"

#include "Context.hpp"
#include "../../../Utils/Log.hpp"
#include "../../../Utils/Profiler.hpp"

#include <chrono>
#include <glm/ext.hpp>

void ldplab::rtscuda::RayTracingStep::execute(
    const SimulationState& input, 
    RayTracingStepOutput& output)
{
    LDPLAB_PROFILING_START(rtscuda_step);
    // Start execution
    LDPLAB_LOG_INFO("RTSCUDA context %i: "\
        "Ray tracing step starts execution",
        m_context->uid);
    std::chrono::steady_clock::time_point start =
        std::chrono::steady_clock::now();

    // Update context
    if (!updateContext(input))
        return;

    // Execute pipeline
    LDPLAB_PROFILING_START(ray_tracing_step_pipeline_setup);
    m_context->pipeline->setup();
    LDPLAB_PROFILING_STOP(ray_tracing_step_pipeline_setup);

    LDPLAB_PROFILING_START(ray_tracing_step_pipeline_execution);
    m_context->pipeline->execute();
    LDPLAB_PROFILING_STOP(ray_tracing_step_pipeline_execution);

    LDPLAB_PROFILING_START(ray_tracing_step_receive_output);
    m_context->pipeline->getOutput(output);
    LDPLAB_PROFILING_STOP(ray_tracing_step_receive_output);

    // Finish execution
    std::chrono::steady_clock::time_point end =
        std::chrono::steady_clock::now();
    const double elapsed_time = std::chrono::duration<double>(
        end - start).count();
    LDPLAB_LOG_INFO("RTSCUDA context %i: Ray tracing step executed after %fs",
        m_context->uid, elapsed_time);
    LDPLAB_PROFILING_STOP(rtscuda_step);
}

bool ldplab::rtscuda::RayTracingStep::updateContext(
    const SimulationState& input)
{
    for (size_t i = 0; i < m_context->parameters.num_particles; ++i)
    {
        UID<Particle> puid{ 
            m_context->interface_mapping.particle_index_to_uid[i] };
        std::map<UID<Particle>, ParticleInstance>::const_iterator particle_it =
            input.particle_instances.find(puid);
        if (particle_it == input.particle_instances.end())
        {
            LDPLAB_LOG_ERROR("RTSCPU context %i: Could not update particle "\
                "transformations, particle %i is not present in the given "\
                "simulation state, abort RTSCPU execution",
                m_context->uid,
                puid);
            return false;
        }

        // Set particle current transformation
        const ParticleInstance& particle = particle_it->second;
        m_context->resources.transformations.host_w2p_translation[i] =
            -particle.position;
        m_context->resources.transformations.host_p2w_translation[i] =
            particle.position;
        m_context->resources.transformations.host_w2p_transformation[i] =
            getRotationMatrix(
                -particle.orientation.x,
                -particle.orientation.y,
                -particle.orientation.z,
                invertRotationOrder(particle.rotation_order));
        m_context->resources.transformations.host_p2w_transformation[i] =
            getRotationMatrix(
                particle.orientation.x,
                particle.orientation.y,
                particle.orientation.z,
                particle.rotation_order);
    }

    // Upload data
    if (!m_context->resources.transformations.p2w_transformation.upload(
        m_context->resources.transformations.host_p2w_transformation.data()))
        return false;
    if (!m_context->resources.transformations.p2w_translation.upload(
        m_context->resources.transformations.host_p2w_translation.data()))
        return false;
    if (!m_context->resources.transformations.w2p_transformation.upload(
        m_context->resources.transformations.host_w2p_transformation.data()))
        return false;
    if (!m_context->resources.transformations.w2p_translation.upload(
        m_context->resources.transformations.host_w2p_translation.data()))
        return false;
    return true;
}

ldplab::Mat3 ldplab::rtscuda::RayTracingStep::getRotationMatrix(
    double rx,
    double ry,
    double rz,
    RotationOrder order)
{
    ldplab::Mat3 rotx(0), roty(0), rotz(0);

    rotx[0][0] = 1;
    rotx[1][1] = cos(rx);
    rotx[1][2] = -sin(rx);
    rotx[2][1] = sin(rx);
    rotx[2][2] = cos(rx);

    roty[0][0] = cos(ry);
    roty[0][2] = sin(ry);
    roty[1][1] = 1;
    roty[2][0] = -sin(ry);
    roty[2][2] = cos(ry);

    rotz[0][0] = cos(rz);
    rotz[0][1] = -sin(rz);
    rotz[1][0] = sin(rz);
    rotz[1][1] = cos(rz);
    rotz[2][2] = 1;

    switch (order)
    {
    case (RotationOrder::xyz): return rotz * roty * rotx;
    case (RotationOrder::xzy): return roty * rotz * rotx;
    case (RotationOrder::yxz): return rotz * rotx * roty;
    case (RotationOrder::yzx): return rotx * rotz * roty;
    case (RotationOrder::zxy): return roty * rotx * rotz;
    case (RotationOrder::zyx): return rotx * roty * rotz;
    }

    // To avoid compiler warnings
    LDPLAB_LOG_WARNING("RTSCPU context %i: Encountered unknown rotation "\
        "order, assumes xyz instead.");
    return rotz * roty * rotx;
}

#endif