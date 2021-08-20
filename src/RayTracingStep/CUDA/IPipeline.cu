#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "IPipeline.hpp"

#include "../../Utils/Log.hpp"
#include "../../Utils/Profiler.hpp"

bool ldplab::rtscuda::IPipeline::stepSetup(
    const ldplab::SimulationState& sim_state)
{
    for (size_t i = 0; i < m_context->execution_model.device_contexts.size(); ++i)
    {
        auto& dvctx = m_context->execution_model.device_contexts[i];
        if (!dvctx.activateDevice())
            return false;

        ParticleTransformationBuffers& ptb =
            m_context->per_device_data[i].particle_transformation_buffers;
        if (i == 0)
        {
            for (size_t p = 0; 
                p < m_context->simulation_parameter.num_particles; 
                ++p)
            {
                UID<Particle> puid{
                m_context->interface_mapping.particle_index_to_uid[p] };
                std::map<UID<Particle>, ParticleInstance>::const_iterator particle_it =
                    sim_state.particle_instances.find(puid);
                if (particle_it == sim_state.particle_instances.end())
                {
                    LDPLAB_LOG_ERROR("RTSCPU context %p: Could not update particle "\
                        "transformations, particle %p is not present in the given "\
                        "simulation state, abort RTSCPU execution",
                        m_context->instance_uid,
                        puid);
                    return false;
                }

                // Set particle current transformation
                const ParticleInstance& particle = particle_it->second;
                ptb.w2p_translation_buffer.getHostBuffer()[p] = -particle.position;
                ptb.p2w_translation_buffer.getHostBuffer()[p] = particle.position;
                ptb.w2p_transformation_buffer.getHostBuffer()[p] =
                    getRotationMatrix(
                        -particle.orientation.x,
                        -particle.orientation.y,
                        -particle.orientation.z,
                        invertRotationOrder(particle.rotation_order));
                ptb.p2w_transformation_buffer.getHostBuffer()[p] =
                    getRotationMatrix(
                        particle.orientation.x,
                        particle.orientation.y,
                        particle.orientation.z,
                        particle.rotation_order);
            }
        }

        auto& ptb0 = m_context->per_device_data[0].particle_transformation_buffers;
        if (!ptb.p2w_transformation_buffer.uploadExt(
                ptb0.p2w_transformation_buffer.getHostBuffer()) ||
            !ptb.p2w_translation_buffer.uploadExt(
                ptb0.p2w_translation_buffer.getHostBuffer()) ||
            !ptb.w2p_transformation_buffer.uploadExt(
                ptb0.w2p_transformation_buffer.getHostBuffer()) ||
            !ptb.w2p_translation_buffer.uploadExt(
                ptb0.w2p_translation_buffer.getHostBuffer()))
            return false;

        // Call stepSetup on each pipeline stage.
        m_stage_bvi->stepSetup(sim_state, *m_context, dvctx);
        m_stage_is->stepSetup(sim_state, *m_context, dvctx);
        m_stage_ipp->stepSetup(sim_state, *m_context, dvctx);
        m_stage_pi->stepSetup(sim_state, *m_context, dvctx);
        m_stage_si->stepSetup(sim_state, *m_context, dvctx);

        dvctx.synchronizeOnDevice();
    }
    return true;
}

bool ldplab::rtscuda::IPipeline::finalizeOutput(RayTracingStepOutput& output)
{
    const size_t num_particles = m_context->simulation_parameter.num_particles;
    for (size_t p = 0; p < num_particles; ++p)
    {
        UID<Particle> puid =
            m_context->interface_mapping.particle_index_to_uid[p];
        output.force_per_particle[puid] = Vec3{ 0 };
        output.torque_per_particle[puid] = Vec3{ 0 };
    }

    // Download output data
    for (size_t i = 0; i < m_context->execution_model.stream_contexts.size(); ++i)
    {
        StreamContext& smctx = m_context->execution_model.stream_contexts[i];
        if (!smctx.deviceContext().activateDevice())
            return false;
        if (!smctx.outputDataBuffers().force_per_particle_buffer.download())
        {
            LDPLAB_LOG_ERROR("RTSCUDA context %i: Failed to download force "\
                "output from device",
                m_context->instance_uid);
            return false;
        }
        if (!smctx.outputDataBuffers().torque_per_particle_buffer.download())
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
                smctx.outputDataBuffers().force_per_particle_buffer.getHostBuffer()[p];
            output.torque_per_particle[puid] +=
                smctx.outputDataBuffers().torque_per_particle_buffer.getHostBuffer()[p];
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

