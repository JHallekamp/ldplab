// Part of the WWU/LDPLAB Project
// Compute Shader: Count buffer state
#version 460 core

// Local work group size
layout(local_size_x = 256) in;

// Temp data
layout(std430, binding = 0) buffer tempData
{ ivec2 temp_data[]; };

// Uniforms
uniform uint source_offset;
uniform uint buffer_size;

// Shader main
void main()
{
    // Calculate the global index
    const uint dest =
        gl_WorkGroupID.x * gl_WorkGroupSize.x + gl_LocalInvocationID.x;

    // Compute
    const uint source = dest + source_offset;
    if (source < buffer_size)
        temp_data[dest] += temp_data[source];
}
