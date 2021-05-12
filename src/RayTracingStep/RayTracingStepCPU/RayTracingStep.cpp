#include "RayTracingStep.hpp"
#include "Context.hpp"

#include "../../Utils/Log.hpp"
#include "../../Utils/Assert.hpp"
#include "../../Utils/Profiler.hpp"

#include <chrono>
#include <glm/ext.hpp>

ldplab::rtscpu::RayTracingStep::RayTracingStep(
    std::unique_ptr<Context> context)
    :
    m_context{ std::move(context) }
{
    LDPLAB_LOG_INFO("RTSCPU context %i: "\
        "RayTracingStep instance created",
        m_context->uid);
}

ldplab::Mat3 ldplab::rtscpu::RayTracingStep::getRotationMatrix(
    real_t rx, 
    real_t ry, 
    real_t rz,
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

void ldplab::rtscpu::RayTracingStep::updateContext(const SimulationState& input)
{
    for (size_t i = 0; i < m_context->particles.size(); ++i)
    {
        UID<Particle> puid{ m_context->particle_index_to_uid_map[i] };
        std::map<UID<Particle>, ParticleInstance>::const_iterator particle_it
            = input.particle_instances.find(puid);
        if (particle_it == input.particle_instances.end())
        {
            LDPLAB_LOG_ERROR("RTSCPU context %i: Could not update particle "\
                "transformations, particle %i is not present in the given "\
                "simulation state, abort RTSCPU execution",
                m_context->uid,
                puid);
            return;
        }
        const ParticleInstance& particle = particle_it->second;

        // Set particle current transformation
        m_context->particle_transformations[i].w2p_translation =
            -particle.position;
        m_context->particle_transformations[i].p2w_translation =
            particle.position;
        m_context->particle_transformations[i].w2p_rotation_scale =
            getRotationMatrix(
                -particle.orientation.x,
                -particle.orientation.y,
                -particle.orientation.z,
                invertRotationOrder(particle.rotation_order));
        m_context->particle_transformations[i].p2w_scale_rotation =
            getRotationMatrix(
                particle.orientation.x,
                particle.orientation.y,
                particle.orientation.z,
                particle.rotation_order);

        // Transform bounding volumes
        if (m_context->bounding_volume_data->type() ==
            IBoundingVolumeData::Type::spheres)
        {
            // Set sphere radius
            ((BoundingSphereData*)
                m_context->bounding_volume_data.get())->sphere_data[i].radius =
                ((BoundingVolumeSphere*)
                    m_context->particles[i].bounding_volume.get())->radius;
            // Set sphere center
            ((BoundingSphereData*)
                m_context->bounding_volume_data.get())->sphere_data[i].center =
                m_context->particle_transformations[i].p2w_scale_rotation *
                ((BoundingVolumeSphere*)
                    m_context->particles[i].bounding_volume.get())->center +
                m_context->particle_transformations[i].p2w_translation;
        }
    }
}

void ldplab::rtscpu::RayTracingStep::execute(
    const SimulationState& input, RayTracingStepOutput& output)
{
    LDPLAB_ASSERT(input.particle_instances.size() == 
        m_context->particles.size());
    LDPLAB_LOG_INFO("RTSCPU context %i: "\
        "Ray tracing step starts execution",
        m_context->uid);
    std::chrono::steady_clock::time_point start = 
        std::chrono::steady_clock::now();

    LDPLAB_PROFILING_START(rtscpu_update_context);
    updateContext(input);
    LDPLAB_PROFILING_STOP(rtscpu_update_context);

    // Execute pipeline
    LDPLAB_LOG_DEBUG("RTSCPU context %i: Setup ray tracing pipeline",
        m_context->uid);
    LDPLAB_PROFILING_START(rtscpu_setup_pipeline);
    m_context->pipeline->setup();
    LDPLAB_PROFILING_STOP(rtscpu_setup_pipeline);

    LDPLAB_LOG_DEBUG("RTSCPU context %i: Execute ray tracing pipeline",
        m_context->uid);
    LDPLAB_PROFILING_START(rtscpu_execute_pipeline);
    m_context->thread_pool->executeJobBatch(
        m_context->pipeline, m_context->parameters.number_parallel_pipelines);
    m_context->pipeline->finalizeOutput(output);
    LDPLAB_PROFILING_STOP(rtscpu_execute_pipeline);

    std::chrono::steady_clock::time_point end =
        std::chrono::steady_clock::now();
    const real_t elapsed_time = std::chrono::duration<real_t>(
            end - start).count();
    LDPLAB_LOG_INFO("RTSCPU context %i: Ray tracing step executed after %fs",
        m_context->uid, elapsed_time);
}