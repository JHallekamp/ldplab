// Part of the WWU/LDPLAB Project
// Compute Shader: Reset buffer
#version 460 core

// Local work group size
layout(local_size_x = 64) in;

// Intersection indices
layout(std430, binding = 0) buffer intersectionIndexData
{ int intersection_index[]; };

// Output buffer data
layout(std430, binding = 1) buffer outputForceData
{ dvec4 output_force_scattered[]; };
layout(std430, binding = 2) buffer outputTorqueData
{ dvec4 output_torque_scattered[]; };

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
    output_force_scattered[ri] = dvec4(0);
    output_torque_scattered[ri] = dvec4(0);
}
