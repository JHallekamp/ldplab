// Part of the WWU/LDPLAB Project
// Compute Shader: LinearIndexGradientRodParticlePropagation
#version 460 core

// Local work group size
layout(local_size_x = 1) in;

// Ray data
layout(std430, binding = 0) buffer rayPositionData { dvec3 position[]; };
layout(std430, binding = 1) buffer rayDirectionData { dvec3 direction[]; };
layout(std430, binding = 2) buffer rayIntensityData { double intensity[]; };

// Number if rays
uniform uint num_rays;

// Main method
void main()
{

}
