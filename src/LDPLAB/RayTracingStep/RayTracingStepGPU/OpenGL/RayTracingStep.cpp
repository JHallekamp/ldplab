#include "RayTracingStep.hpp"
#include "Context.hpp"

#include "../../../Log.hpp"
#include "../../../Utils/Assert.hpp"

#include <chrono>
#include <glm/ext.hpp>

ldplab::rtsgpu_ogl::RayTracingStep::RayTracingStep(
    std::shared_ptr<Context> context)
    :
    m_context{ context }
{
    LDPLAB_LOG_INFO("RTSGPU (OpenGL) context %i: "\
        "RayTracingStep instance created",
        m_context->uid);
}

void ldplab::rtsgpu_ogl::RayTracingStep::updateContext(
    const SimulationState& input)
{
    // Update context
    for (size_t i = 0; i < m_context->particles.size(); ++i)
    {
        UID<Particle> puid{ m_context->particle_index_to_uid_map[i] };
        std::map<UID<Particle>, ParticleInstance>::const_iterator particle_it
            = input.particle_instances.find(puid);
        if (particle_it == input.particle_instances.end())
        {
            LDPLAB_LOG_ERROR("RTSGPU (OpenGL) context %i: Could not update particle "\
                "transformations, particle %i is not present in the given "\
                "simulation state, abort RTSGPU (OpenGL) execution",
                m_context->uid,
                puid);
            return;
        }
        const ParticleInstance& particle = particle_it->second;

        // Set particle current transformation
        m_context->particle_transformation_data.w2p_translation_data[i] =
            Vec4(-particle.position, 0);
        m_context->particle_transformation_data.p2w_translation_data[i] =
            Vec4(particle.position, 0);
        m_context->particle_transformation_data.w2p_rotation_scale_data[i] =
            getRotationMatrix(
                particle.orientation.x,
                particle.orientation.y,
                particle.orientation.z,
                particle.rotation_order);
        m_context->particle_transformation_data.p2w_scale_rotation_data[i] =
            getRotationMatrix(
                -particle.orientation.x,
                -particle.orientation.y,
                -particle.orientation.z,
                invertRotationOrder(particle.rotation_order));

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
                m_context->particle_transformation_data.p2w_scale_rotation_data[i] *
                Vec4(((BoundingVolumeSphere*)
                    m_context->particles[i].bounding_volume.get())->center, 0) +
                m_context->particle_transformation_data.p2w_translation_data[i];
        }
    }

    // Upload ssbos
    std::unique_lock<std::mutex> lck{ m_context->ogl->getGPUMutex() };
    m_context->ogl->bindGlContext();
    m_context->particle_transformation_data.uploadSSBO();
    m_context->bounding_volume_data->uploadSSBO();
    m_context->ogl->unbindGlContext();
}

void ldplab::rtsgpu_ogl::RayTracingStep::initGPU()
{
    std::lock_guard<std::mutex> gpu_lock(m_context->ogl->getGPUMutex());
    m_context->ogl->bindGlContext();

    // Create SSBOs for particle geometry
    if (m_context->particle_data->type() == IParticleData::Type::rod_particles)
    {
        RodParticleData* const particle_data = ((RodParticleData*)
            m_context->particle_data.get());
        particle_data->ssbo.cylinder_length =
            m_context->ogl->createShaderStorageBuffer(
                particle_data->particle_data.size() * sizeof(double));
        particle_data->ssbo.cylinder_radius =
            m_context->ogl->createShaderStorageBuffer(
                particle_data->particle_data.size() * sizeof(double));
        particle_data->ssbo.origin_cap =
            m_context->ogl->createShaderStorageBuffer(
                particle_data->particle_data.size() * sizeof(Vec4));
        particle_data->ssbo.origin_indentation =
            m_context->ogl->createShaderStorageBuffer(
                particle_data->particle_data.size() * sizeof(Vec4));
        particle_data->ssbo.sphere_radius =
            m_context->ogl->createShaderStorageBuffer(
                particle_data->particle_data.size() * sizeof(double));
        particle_data->uploadSSBO();
    }
    else
    {
        LDPLAB_LOG_ERROR("RTSGPU (OpenGL) context %i: Could not create "\
            "particle SSBOs, particle type not supported", m_context->uid);
    }

    // Create SSBOs for particle materials
    if (m_context->particle_material_data->type() ==
        IParticleMaterialData::Type::linear_one_directional)
    {
        ParticleMaterialLinearOneDirectionalData* const particle_material =
            ((ParticleMaterialLinearOneDirectionalData*)
                m_context->particle_material_data.get());
        particle_material->ssbo.direction_times_gradient =
            m_context->ogl->createShaderStorageBuffer(
                particle_material->material_data.size() * sizeof(Vec4));
        particle_material->ssbo.index_of_refraction_sum_term =
            m_context->ogl->createShaderStorageBuffer(
                particle_material->material_data.size() * sizeof(double));
        particle_material->uploadSSBO();
    }
    else
    {
        LDPLAB_LOG_ERROR("RTSGPU (OpenGL) context %i: Could not create "\
            "particle material SSBOs, particle material type not supported",
            m_context->uid);
    }

    // Create SSBOs for space transformations
    const size_t num_particles = m_context->particles.size();
    m_context->particle_transformation_data.ssbo.p2w_scale_rotation =
        m_context->ogl->createShaderStorageBuffer(num_particles * sizeof(Mat4));
    m_context->particle_transformation_data.ssbo.p2w_translation =
        m_context->ogl->createShaderStorageBuffer(num_particles * sizeof(Vec4));
    m_context->particle_transformation_data.ssbo.w2p_rotation_scale =
        m_context->ogl->createShaderStorageBuffer(num_particles * sizeof(Mat4));
    m_context->particle_transformation_data.ssbo.w2p_translation =
        m_context->ogl->createShaderStorageBuffer(num_particles * sizeof(Vec4));

    m_context->ogl->unbindGlContext();
}

ldplab::Mat4 ldplab::rtsgpu_ogl::RayTracingStep::getRotationMatrix(
    double rx,
    double ry,
    double rz,
    RotationOrder order)
{
    ldplab::Mat4 rotx(0), roty(0), rotz(0);

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
    LDPLAB_LOG_WARNING("RTSGPU (OpenGL) context %i: Encountered unknown "\
        "rotation order, assumes xyz instead.");
    return rotz * roty * rotx;
}

void ldplab::rtsgpu_ogl::RayTracingStep::execute(
    const SimulationState& input, RayTracingStepOutput& output)
{
    LDPLAB_ASSERT(input.particle_instances.size() == 
        m_context->particles.size());
    LDPLAB_LOG_INFO("RTSGPU (OpenGL) context %i: "\
        "Ray tracing step starts execution",
        m_context->uid);
    std::chrono::steady_clock::time_point start = 
        std::chrono::steady_clock::now();
    
    updateContext(input);

    // Execute pipeline
    LDPLAB_LOG_DEBUG("RTSGPU (OpenGL) context %i: Setup ray tracing pipeline",
        m_context->uid);
    m_context->pipeline->setup();
    LDPLAB_LOG_DEBUG("RTSGPU (OpenGL) context %i: Execute ray tracing pipeline",
        m_context->uid);
    m_context->thread_pool->executeJobBatch(
        m_context->pipeline, m_context->parameters.number_parallel_pipelines);
    m_context->pipeline->finalizeOutput(output);

    std::chrono::steady_clock::time_point end =
        std::chrono::steady_clock::now();
    const double elapsed_time = std::chrono::duration<double>(
            end - start).count();
    LDPLAB_LOG_INFO("RTSGPU (OpenGL) context %i: Ray tracing step executed after %fs",
        m_context->uid, elapsed_time);
}