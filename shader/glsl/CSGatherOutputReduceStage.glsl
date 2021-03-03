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
layout(std430, binding = 0) buffer tempData
{ OutputData temp_data[]; };

// Uniforms
uniform uint source_offset;
uniform uint buffer_size;
uniform uint num_particles;

// Shader main
void main()
{
    // Calculate the global index
    const uint gi =
        gl_WorkGroupID.x * gl_WorkGroupSize.x + gl_LocalInvocationID.x;

    uint dst_offset = gi * num_particles;
    uint src_offset = dst_offset + source_offset;
    if (src_offset >= buffer_size)
        return;

    for (uint i = 0; i < num_particles; ++i)
    {
        temp_data[dst_offset + i].force += temp_data[src_offset + i].force;
        temp_data[dst_offset + i].torque += temp_data[src_offset + i].torque;
    }
}
