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

// Gather result
layout(std430, binding = 3) buffer resultData
{ OutputData result_data[]; };

// Uniforms
uniform uint num_rays_per_buffer;
uniform uint num_particles;

// Shader main
void main()
{
    // Calculate the global index
    const uint gi =
        gl_WorkGroupID.x * gl_WorkGroupSize.x + gl_LocalInvocationID.x;

    // Fill temp data
    OutputData null_data;
    null_data.force = dvec4(0, 0, 0, 0);
    null_data.torque = dvec4(0, 0, 0, 0);
    for (uint i = 0; i < num_particles; ++i)
    {
        temp_data[gi + i] =
            (ray_index_data[gi] == i) ? output_data[gi] : null_data;
        // Calculate the first offset
        uint offset = (num_rays_per_buffer / 2) + uint(mod(num_rays_per_buffer, 2));
        // Calculate the first limit
        uint lim = num_rays_per_buffer;
        // Gather the scattered data from the temp data
        while(gi + offset < lim && lim > 1)
        {
            temp_data[gi].force += temp_data[gi + offset].force;
            temp_data[gi].torque += temp_data[gi + offset].torque;
            lim = offset;
            offset = (lim / 2) + uint(mod(lim, 2));
        }
        if (gi == 0)
        {
            result_data[i].force += temp_data[0].force;
            result_data[i].torque += temp_data[0].torque;
        }
    }
}
