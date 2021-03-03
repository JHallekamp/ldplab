// Part of the WWU/LDPLAB Project
// Compute Shader: Count buffer state
#version 460 core

// Local work group size
layout(local_size_x = 256) in;

// Temp data
layout(std430, binding = 0) readonly buffer tempData
{ ivec2 temp_data[]; };

// Output
layout(std430, binding = 1) writeonly buffer rayStatusCount
{
    int num_active_rays;
    int num_world_space_rays;
};

// Shader main
void main()
{
    // Calculate the global index
    const uint gi =
        gl_WorkGroupID.x * gl_WorkGroupSize.x + gl_LocalInvocationID.x;

    if (gi == 0)
    {
        num_active_rays = temp_data[0].x;
        num_world_space_rays = temp_data[0].y;
    }
}
