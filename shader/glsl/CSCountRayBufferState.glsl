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

// Output
layout(std430, binding = 2) writeonly buffer rayStatusCount
{
    int num_active_rays;
    int num_world_space_rays;
};

// Uniforms
uniform uint num_rays_per_buffer;
uniform uint num_particles;

// Shader main
void main()
{
    // Calculate the global index
    const uint gi =
        gl_WorkGroupID.x * gl_WorkGroupSize.x + gl_LocalInvocationID.x;

    temp_data[gi].x = (ray_index_data[gi] >= 0) ? 1 : 0;
    temp_data[gi].y = (ray_index_data[gi] >= int(num_particles)) ? 1 : 0;
    // Calculate the first offset
    uint offset = (num_rays_per_buffer / 2) + uint(mod(num_rays_per_buffer, 2));
    // Calculate the first limit
    uint lim = num_rays_per_buffer;
    // Gather the scattered data from the temp data
    while(gi + offset < lim && lim > 1)
    {
        temp_data[gi] += temp_data[gi + offset];
        lim = offset;
        offset = (lim / 2) + uint(mod(lim, 2));
    }

    if (gi == 0)
    {
        num_active_rays = temp_data[0].x;
        num_world_space_rays = temp_data[0].y;
    }
}
