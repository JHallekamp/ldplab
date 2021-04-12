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

// Output buffer data
layout(std430, binding = 0) readonly buffer outputData
{ OutputData output_data[]; };

// Ray data
layout(std430, binding = 1) readonly buffer rayIndexData
{ int ray_index_data[]; };

// Temporary data
layout(std430, binding = 2) buffer tempData
{ OutputData temp_data[]; };

// Uniforms
uniform uint num_rays_per_buffer;
uniform uint num_particles;

// Shader main
void main()
{
    // Calculate the global index
    const uint gi =
        gl_WorkGroupID.x * gl_WorkGroupSize.x + gl_LocalInvocationID.x;

    if (gi >= num_rays_per_buffer)
        return;

    // Fill temp data
    OutputData null_data;
    null_data.force = dvec4(0, 0, 0, 0);
    null_data.torque = dvec4(0, 0, 0, 0);
    for (uint i = 0; i < num_particles; ++i)
    {
        temp_data[gi * num_particles + i] =
            (ray_index_data[gi] == i) ? output_data[gi] : null_data;
    }
}
