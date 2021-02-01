// Part of the WWU/LDPLAB Project
// Compute Shader: LinearIndexGradientRodParticlePropagation
#version 460 core

// Local work group size
layout(local_size_x = 1) in;

// Ray buffer data
layout(std430, binding = 0) buffer rayIndexData { int ray_index[]; };
layout(std430, binding = 1) buffer rayOriginnData { dvec3 ray_origin[]; };
layout(std430, binding = 2) buffer rayDirectionData { dvec3 ray_direction[]; };
layout(std430, binding = 3) buffer rayIntensityData { double ray_intensity[]; };

// Intersectino buffer data
layout(std430, binding = 4) buffer intersectionPointData
{ dvec3 intersection_position[]; };
layout(std430, binding = 5) buffer intersectionNormalData
{ dvec3 intersection_normal[]; };

// Output buffer data
layout(std430, binding = 6) buffer outputForceData { dvec3 output_force[]; };
layout(std430, binding = 7) buffer outputTorqueData { dvec3 output_torque[]; };

// Particle data
layout(std430, binding = 8) buffer particleCylinderRadiusData
{ double particle_cylinder_radius[]; };
layout(std430, binding = 9) buffer particleCylinderLengthData
{ double particle_cylinder_length[]; };
layout(std430, binding = 10) buffer particleSphereRadiusData
{ double particle_sphere_radius[]; };
layout(std430, binding = 11) buffer particleCapOriginData
{ dvec3 particle_cap_origin[]; };
layout(std430, binding = 12) buffer particleIndentationOriginData
{ dvec3 particle_indentation_origin[]; };

// Particle material data
layout(std430, binding = 13) buffer particleMaterialIndexOfRefrectionSumTermData
{ double particle_material_index_of_refraction_sum_term[]; };
layout(std430, binding = 14) buffer particleMaterialDirectionTimesGradient
{ dvec3 particle_material_direction_times_gradient[]; };

// Property data
uniform uint num_rays_per_buffer;
uniform double parameter_initial_step_size;
uniform double parameter_epsilon;

// Port ParticleMaterialLinearOneDirectional::indexOfRefraction(const Vec3&)
double particleMaterialIndexOfRefrection(const int pi, const dvec3 pos)
{
    return particle_material_index_of_refraction_sum_term[pi] +
        dot(particle_material_direction_times_gradient[pi], pos);
}

// Port LinearIndexGradientRodParticlePropagation::rk45
double rk45(
    const int pi,
    const dvec3 x_w,
    const dvec3 x_r,
    const double h,
    inout dvec3 x_new_w,
    inout dvec3 x_new_r)
{

}

// Main method
void main()
{
    // Calculate the ray index
    const uint ri =
        gl_WorkGroupID.x * gl_WorkGroupSize.x + gl_LocalInvocationID.x;

    // Load the particle index
    const int pi = ray_index[ri];

    // Check if shader run is legal
    if ((gi >= num_rays_per_buffer) || (pi < 0))
        return;

    // Setting up varibles of the differential equation
    dvec3 arg_x_r = ray_origin[ri];
    dvec3 arg_x_w = ray_direction[ri] * indexOfRefraction(pi, arg_x_r);
    dvec3 arg_x_new_r, arg_x_new_w;

    // Setup iteration
    bool intersected = false;
    double h = parameter_initial_step_size;
    double error = 0;
    while (!intersected)
    {
        // Use rk45
        error = rk45(pi, arg_x_w, arg_x_r, h, arg_x_new_w, arg_x_new_r);
        if (error )
    }
}
