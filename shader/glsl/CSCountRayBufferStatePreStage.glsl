// Part of the WWU/LDPLAB Project
// Compute Shader: Count buffer state
#version 460 core

// Local work group size
layout(local_size_x = 256) in;

// Ray data
layout(std430, binding = 0) readonly buffer rayIndexData
{ int ray_index_data[]; };

// Temp data
layout(std430, binding = 1) buffer tempData
{ ivec2 temp_data[]; };

// Uniforms
uniform uint num_rays_per_buffer;
uniform uint num_particles;

// Shader main
void main()
{
    // Calculate the global index
    const uint gi =
        gl_WorkGroupID.x * gl_WorkGroupSize.x + gl_LocalInvocationID.x;

    // Check if global index is within range
    if (gi >= num_rays_per_buffer)
        return;

    // Copy data into temp buffer
    temp_data[gi].x = (ray_index_data[gi] >= 0) ? 1 : 0;
    temp_data[gi].y = (ray_index_data[gi] >= int(num_particles)) ? 1 : 0;
}
