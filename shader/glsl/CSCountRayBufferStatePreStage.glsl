// Part of the WWU/LDPLAB Project
// Compute Shader: Count buffer state
#version 460 core

// Local work group size
layout(local_size_x = 256) in;

// Ray data
layout(std430, binding = 0) readonly buffer rayIndexData1
{ int ray_index_data1[]; };

// Ray data
layout(std430, binding = 1) readonly buffer rayIndexData2
{ int ray_index_data2[]; };

// Temp data
layout(std430, binding = 2) buffer tempData
{ ivec2 temp_data[]; };

// Uniforms
uniform uint num_rays_per_buffer;
uniform bool second_ray_index_buffer;
uniform uint threshold1;
uniform uint threshold2;

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
    temp_data[gi].x = (ray_index_data1[gi] >= int(threshold1)) ? 1 : 0;
    if (second_ray_index_buffer)
        temp_data[gi].y = (ray_index_data2[gi] >= int(threshold1)) ? 1 : 0;
    else
        temp_data[gi].y = (ray_index_data1[gi] >= int(threshold2)) ? 1 : 0;
}
