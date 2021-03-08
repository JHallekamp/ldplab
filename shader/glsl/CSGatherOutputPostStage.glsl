// Part of the WWU/LDPLAB Project
// Compute Shader: Reset buffer
#version 460 core

// Local work group size
layout(local_size_x = 256) in;

// Data structures
struct OutputData
{
    // xyz: Force vector
    // w: UNUSED
    dvec4 force;
    // xyz: Torque vector
    // w: UNUSED
    dvec4 torque;
};

// Temporary data
layout(std430, binding = 0) readonly buffer tempData
{ OutputData temp_data[]; };

// Gather result
layout(std430, binding = 1) buffer resultData
{ OutputData result_data[]; };

// Uniforms
uniform uint num_particles;

// Shader main
void main()
{
    // Calculate the global index
    const uint gi =
        gl_WorkGroupID.x * gl_WorkGroupSize.x + gl_LocalInvocationID.x;

    if (gi == 0)
    {
        for (uint i = 0; i < num_particles; ++i)
        {
            result_data[i].force += temp_data[i].force;
            result_data[i].torque += temp_data[i].torque;
        }
    }
}
