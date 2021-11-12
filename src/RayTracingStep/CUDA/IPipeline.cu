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
                        -particle_instance.orientation.z,
                        -particle_instance.orientation.y,
                        -particle_instance.orientation.x,
                        invertRotationOrder(particle_instance.rotation_order));
                ptb.p2w_transformation_buffer.getHostBuffer()[p] =
                    getRotationMatrix(
                        particle_instance.orientation.x,
                        particle_instance.orientation.y,
                        particle_instance.orientation.z,
                        particle_instance.rotation_order);
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

namespace
{
    ldplab::Mat3 rotationX(const double angle)
    {
        ldplab::Mat3 rot(0);

        rot[0][0] = 1;
        rot[1][1] = cos(angle);
        rot[1][2] = sin(angle);
        rot[2][1] = -sin(angle);
        rot[2][2] = cos(angle);

        return rot;
    }
    ldplab::Mat3 rotationY(const double angle)
    {
        ldplab::Mat3 rot(0);

        rot[0][0] = cos(angle);
        rot[0][2] = -sin(angle);
        rot[1][1] = 1;
        rot[2][0] = sin(angle);
        rot[2][2] = cos(angle);

        return rot;
    }
    ldplab::Mat3 rotationZ(const double angle)
    {
        ldplab::Mat3 rot(0);

        rot[0][0] = cos(angle);
        rot[0][1] = sin(angle);
        rot[1][0] = -sin(angle);
        rot[1][1] = cos(angle);
        rot[2][2] = 1;

        return rot;
    }
}

ldplab::Mat3 ldplab::rtscuda::IPipeline::getRotationMatrix(
	double a, 
	double b, 
	double c, 
	RotationOrder order)
{
    switch (order)
    {
    case (RotationOrder::xyz): return rotationZ(c) * rotationY(b) * rotationX(a);
    case (RotationOrder::xzy): return rotationY(c) * rotationZ(b) * rotationX(a);
    case (RotationOrder::yxz): return rotationZ(c) * rotationX(b) * rotationY(a);
    case (RotationOrder::yzx): return rotationX(c) * rotationZ(b) * rotationY(a);
    case (RotationOrder::zxy): return rotationY(c) * rotationX(b) * rotationZ(a);
    case (RotationOrder::zyx): return rotationX(c) * rotationY(b) * rotationZ(a);
    case (RotationOrder::zyz): return rotationZ(c) * rotationY(b) * rotationZ(a);
    case (RotationOrder::zxz): return rotationZ(c) * rotationX(b) * rotationZ(a);
    }

    // To avoid compiler warnings
    LDPLAB_LOG_WARNING("RTSCPU context %i: Encountered unknown rotation "\
        "order, assumes xyz instead.", m_context->instance_uid);
    return rotz * roty * rotx;
}

#endif
