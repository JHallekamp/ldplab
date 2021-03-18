// Part of the WWU/LDPLAB Project
// Compute Shader: Reset buffer
#version 460 core

// Local work group size
layout(local_size_x = 128) in;

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

// Intersection indices
layout(std430, binding = 0) buffer intersectionIndexData
{ int intersection_index[]; };

// Output buffer data
layout(std430, binding = 1) buffer outputData
{ OutputData output_data[]; };

// Property data
uniform uint num_rays_per_buffer;

// Shader main
void main()
{
    // Calculate the ray index
    const uint ri =
        gl_WorkGroupID.x * gl_WorkGroupSize.x + gl_LocalInvocationID.x;

    // Check if local work item is within bounds
    if (ri >= num_rays_per_buffer)
        return;

    // Reset values
    intersection_index[ri] = -1;
    output_data[ri].force.xyz = dvec3(0);
    output_data[ri].torque.xyz = dvec3(0);
}
