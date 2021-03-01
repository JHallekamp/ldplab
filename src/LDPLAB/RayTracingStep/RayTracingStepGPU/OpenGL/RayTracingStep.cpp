#include "RayTracingStep.hpp"
#include "Context.hpp"
#include "Data.hpp"

#include "../../../Log.hpp"
#include "../../../Utils/Assert.hpp"

#include <chrono>
#include <glm/ext.hpp>

ldplab::rtsgpu_ogl::ParticleTransformationData::Transformation
    buildTranformationData(ldplab::Mat3 rotation_scale, ldplab::Vec3 translation)
{
    ldplab::rtsgpu_ogl::ParticleTransformationData::Transformation trsf;
    trsf.rotation_scale = ldplab::Mat4(0.0);
    trsf.rotation_scale[0][0] = rotation_scale[0][0];
    trsf.rotation_scale[0][1] = rotation_scale[0][1];
    trsf.rotation_scale[0][2] = rotation_scale[0][2];
    trsf.rotation_scale[1][0] = rotation_scale[1][0];
    trsf.rotation_scale[1][1] = rotation_scale[1][1];
    trsf.rotation_scale[1][2] = rotation_scale[1][2];
    trsf.rotation_scale[2][0] = rotation_scale[2][0];
    trsf.rotation_scale[2][1] = rotation_scale[2][1];
    trsf.rotation_scale[2][2] = rotation_scale[2][2];
    trsf.rotation_scale[3][3] = 1.0;
    trsf.translation = ldplab::Mat4(0.0);
    trsf.translation[0][0] = 1.0;
    trsf.translation[0][3] = translation.x;
    trsf.translation[1][1] = 1.0;
    trsf.translation[1][3] = translation.y;
    trsf.translation[2][2] = 1.0;
    trsf.translation[2][3] = translation.z;
    trsf.translation[3][3] = 1.0;
    return trsf;
}

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
        m_context->particle_transformation_data.w2p_translation[i] =
            -particle.position;
        m_context->particle_transformation_data.p2w_translation[i] =
            particle.position;
        m_context->particle_transformation_data.w2p_rotation_scale[i] =
            getRotationMatrix(
                -particle.orientation.x,
                -particle.orientation.y,
                -particle.orientation.z,
                invertRotationOrder(particle.rotation_order));
        m_context->particle_transformation_data.p2w_scale_rotation[i] =
            getRotationMatrix(
                particle.orientation.x,
                particle.orientation.y,
                particle.orientation.z,
                particle.rotation_order);
        m_context->particle_transformation_data.w2p_data[i] =
            buildTranformationData(
                m_context->particle_transformation_data.w2p_rotation_scale[i],
                m_context->particle_transformation_data.w2p_translation[i]);
        m_context->particle_transformation_data.p2w_data[i] =
            buildTranformationData(
                m_context->particle_transformation_data.p2w_scale_rotation[i],
                m_context->particle_transformation_data.p2w_translation[i]);

        // Transform bounding volumes
        if (m_context->bounding_volume_data->type() ==
            IBoundingVolumeData::Type::spheres)
        {
            BoundingSphereData* transformed_bounding_sphere =
                (BoundingSphereData*)m_context->bounding_volume_data.get();
            const BoundingVolumeSphere* original_sphere =
                (BoundingVolumeSphere*)m_context->particles[i].bounding_volume.get();

            // Set sphere radius
            transformed_bounding_sphere->sphere_properties_data[i].radius =
                original_sphere->radius;

            // Set sphere center
            transformed_bounding_sphere->sphere_properties_data[i].center =
                m_context->particle_transformation_data.p2w_scale_rotation[i] *
                original_sphere->center +
                m_context->particle_transformation_data.p2w_translation[i];
        }
    }

    // Upload ssbos
    std::unique_lock<std::mutex> lck{ m_context->ogl->getGPUMutex() };
    m_context->ogl->bindGlContext();
    m_context->particle_transformation_data.ssbo.p2w->upload(
        m_context->particle_transformation_data.p2w_data.data());
    m_context->particle_transformation_data.ssbo.w2p->upload(
        m_context->particle_transformation_data.w2p_data.data());
    m_context->bounding_volume_data->uploadSSBO();
    m_context->ogl->unbindGlContext();
}

bool ldplab::rtsgpu_ogl::RayTracingStep::initGPU(
    const RayTracingStepGPUOpenGLInfo& info)
{
    std::lock_guard<std::mutex> gpu_lock(m_context->ogl->getGPUMutex());
    m_context->ogl->bindGlContext();

    // Create SSBOs for particle geometry
    if (m_context->particle_data->type() == IParticleData::Type::rod_particles)
    {
        RodParticleData* const particle_data = ((RodParticleData*)
            m_context->particle_data.get());
        particle_data->ssbo.rod_particles =
            m_context->ogl->createShaderStorageBuffer(
                particle_data->rod_particles_data.size() * 
                sizeof(RodParticleData::RodParticleProperties));
        if (particle_data->ssbo.rod_particles == nullptr)
        {
            LDPLAB_LOG_ERROR("RTSGPU (OpenGL) context %i: Failed to create "\
                "rod particle data SSBO",
                m_context->uid);
            return false;
        }
        particle_data->ssbo.rod_particles->upload(
            particle_data->rod_particles_data.data());
    }
    else
    {
        LDPLAB_LOG_ERROR("RTSGPU (OpenGL) context %i: Could not create "\
            "particle SSBOs, particle type not supported", m_context->uid);
        return false;
    }

    // Create SSBOs for particle materials
    if (m_context->particle_material_data->type() ==
        IParticleMaterialData::Type::linear_one_directional)
    {
        ParticleMaterialLinearOneDirectionalData* const particle_material =
            ((ParticleMaterialLinearOneDirectionalData*)
                m_context->particle_material_data.get());
        particle_material->ssbo.material =
            m_context->ogl->createShaderStorageBuffer(
                particle_material->material_data.size() * 
                sizeof(ParticleMaterialLinearOneDirectionalData::
                    LinearOneDirectionalMaterialProperties));
        if (particle_material->ssbo.material == nullptr)
        {
            LDPLAB_LOG_ERROR("RTSGPU (OpenGL) context %i: Failed to create "\
                "particle material SSBO",
                m_context->uid);
            return false;
        }
        particle_material->ssbo.material->upload(
            particle_material->material_data.data());
    }
    else
    {
        LDPLAB_LOG_ERROR("RTSGPU (OpenGL) context %i: Could not create "\
            "particle material SSBOs, particle material type not supported",
            m_context->uid);
        return false;
    }

    // Create SSBOs for boundary volume
    if (m_context->bounding_volume_data->type() ==
        IBoundingVolumeData::Type::spheres)
    {
        BoundingSphereData* const spheres =
            (BoundingSphereData*)m_context->bounding_volume_data.get();
        spheres->ssbo.sphere_properties =
            m_context->ogl->createShaderStorageBuffer(
                spheres->sphere_properties_data.size() *
                sizeof(BoundingSphereData::BoundingSphereProperties));
        if (spheres->ssbo.sphere_properties == nullptr)
        {
            LDPLAB_LOG_ERROR("RTSGPU (OpenGL) context %i: Failed to create "\
                "bounding volume SSBO",
                m_context->uid);
            return false;
        }
    }
    else
    {
        LDPLAB_LOG_ERROR("RTSGPU (OpenGL) context %i: Could not create "\
            "bounding volume SSBOs, bounding volume type not supported",
            m_context->uid);
        return false;
    }

    // Create SSBOs for space transformations
    const size_t num_particles = m_context->particles.size();
    m_context->particle_transformation_data.ssbo.w2p =
        m_context->ogl->createShaderStorageBuffer(num_particles * sizeof(Mat4));
    if (m_context->particle_transformation_data.ssbo.w2p == nullptr)
    {
        LDPLAB_LOG_ERROR("RTSGPU (OpenGL) context %i: Failed to create "\
            "particle transformation w2p SSBO",
            m_context->uid);
        return false;
    }
    m_context->particle_transformation_data.ssbo.p2w =
        m_context->ogl->createShaderStorageBuffer(num_particles * sizeof(Mat4));
    if (m_context->particle_transformation_data.ssbo.p2w == nullptr)
    {
        LDPLAB_LOG_ERROR("RTSGPU (OpenGL) context %i: Failed to create "\
            "particle transformation p2w SSBO",
            m_context->uid);
        return false;
    }

    // Initialize pipeline
    m_context->parameters.shader_base_directory = 
        info.shader_base_directory_path;
    if (!m_context->pipeline->initShaders())
    {
        LDPLAB_LOG_ERROR("RTSGPU (OpenGL) context %i: Failed to initialize "\
            "pipeline shaders",
            m_context->uid);
        return false;
    }

    // Done
    m_context->ogl->unbindGlContext();
    return true;
}

ldplab::Mat3 ldplab::rtsgpu_ogl::RayTracingStep::getRotationMatrix(
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