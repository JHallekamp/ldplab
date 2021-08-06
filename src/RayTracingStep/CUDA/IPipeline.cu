#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "IPipeline.hpp"

#include "../../Utils/Log.hpp"
#include "../../Utils/Profiler.hpp"

bool ldplab::rtscuda::IPipeline::stepSetup(
	const ldplab::SimulationState& sim_state)
{
    for (size_t i = 0; i < m_context->simulation_parameter.num_particles; ++i)
    {
        UID<Particle> puid{
            m_context->interface_mapping.particle_index_to_uid[i] };
        std::map<UID<Particle>, ParticleInstance>::const_iterator particle_it =
            sim_state.particle_instances.find(puid);
        if (particle_it == sim_state.particle_instances.end())
        {
            LDPLAB_LOG_ERROR("RTSCPU context %i: Could not update particle "\
                "transformations, particle %i is not present in the given "\
                "simulation state, abort RTSCPU execution",
                m_context->instance_uid,
                puid);
            return false;
        }

        // Set particle current transformation
        const ParticleInstance& particle = particle_it->second;
        m_context->particle_data_buffers.w2p_translation_buffer.getHostBuffer()[i] =
            -particle.position;
        m_context->particle_data_buffers.p2w_translation_buffer.getHostBuffer()[i] =
            particle.position;
        m_context->particle_data_buffers.w2p_transformation_buffer.getHostBuffer()[i] =
            getRotationMatrix(
                -particle.orientation.x,
                -particle.orientation.y,
                -particle.orientation.z,
                invertRotationOrder(particle.rotation_order));
        m_context->particle_data_buffers.p2w_transformation_buffer.getHostBuffer()[i] =
            getRotationMatrix(
                particle.orientation.x,
                particle.orientation.y,
                particle.orientation.z,
                particle.rotation_order);
    }

    // Upload data
    if (!m_context->particle_data_buffers.p2w_transformation_buffer.upload() ||
        !m_context->particle_data_buffers.p2w_translation_buffer.upload() ||
        !m_context->particle_data_buffers.w2p_transformation_buffer.upload() ||
        !m_context->particle_data_buffers.w2p_translation_buffer.upload())
        return false;
    return true;
}

bool ldplab::rtscuda::IPipeline::finalizeOutput(RayTracingStepOutput& output)
{
    // Download output data
    const size_t num_particles = m_context->simulation_parameter.num_particles;
    for (size_t p = 0; p < num_particles; ++p)
    {
        UID<Particle> puid =
            m_context->interface_mapping.particle_index_to_uid[p];
        output.force_per_particle[puid] = Vec3{ 0 };
        output.torque_per_particle[puid] = Vec3{ 0 };
    }
    for (size_t i = 0; i < m_context->batch_data.size(); ++i)
    {
        if (!m_context->batch_data[i].output_data_buffers.
                force_per_particle_buffer.download())
        {
            LDPLAB_LOG_ERROR("RTSCUDA context %i: Failed to download force "\
                "output from device",
                m_context->instance_uid);
            return false;
        }
        if (!m_context->batch_data[i].output_data_buffers.
                torque_per_particle_buffer.download())
        {
            LDPLAB_LOG_ERROR("RTSCUDA context %i: Failed to download torque "\
                "output from device",
                m_context->instance_uid);
            return false;
        }
        for (size_t p = 0; p < num_particles; ++p)
        {
            UID<Particle> puid =
                m_context->interface_mapping.particle_index_to_uid[p];
            output.force_per_particle[puid] += 
                m_context->batch_data[i].output_data_buffers.
                    force_per_particle_buffer.getHostBuffer()[p];
            output.torque_per_particle[puid] += 
                m_context->batch_data[i].output_data_buffers.
                    torque_per_particle_buffer.getHostBuffer()[p];
        }
    }    
    return true;
}

ldplab::Mat3 ldplab::rtscuda::IPipeline::getRotationMatrix(
	double rx, 
	double ry, 
	double rz, 
	RotationOrder order)
{
    Mat3 rotx(0), roty(0), rotz(0);

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
        "order, assumes xyz instead.", m_context->instance_uid);
    return rotz * roty * rotx;
}

#endif

