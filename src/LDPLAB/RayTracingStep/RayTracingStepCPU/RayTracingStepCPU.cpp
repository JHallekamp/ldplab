#include "RayTracingStepCPU.hpp"
#include "Context.hpp"

#include "../../Utils/Assert.hpp"

#include <glm/ext.hpp>

ldplab::Mat3 getRotationMatrix(double rx, double ry, double rz)
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

    return rotz * roty * rotx;
}

void ldplab::rtscpu::RayTracingStepCPU::execute(
    const SimulationState& input, RayTracingStepOutput& output)
{
    LDPLAB_ASSERT(input.particles.size() == m_context.particles.size());
    
    for (size_t i = 0; i < input.particles.size(); ++i)
    {
        m_context->particle_transformations[i].w2p_translation =
            -input.particles[i].position;
        m_context->particle_transformations[i].p2w_translation =
            input.particles[i].position;
        m_context->particle_transformations[i].w2p_rotation_scale =
            getRotationMatrix(
                -input.particles[i].orientation.x,
                -input.particles[i].orientation.y,
                -input.particles[i].orientation.z);
        m_context->particle_transformations[i].p2w_scale_rotation =
            getRotationMatrix(
                input.particles[i].orientation.x, 
                input.particles[i].orientation.y, 
                input.particles[i].orientation.z);
    }

    
}
