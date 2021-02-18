// Part of the WWU/LDPLAB Project
// Compute Shader: UnpolirzedLight1DLinearIndexGradientInteraction
#version 460 core

// Local work group size
layout(local_size_x = 128) in;

// Data structures
struct RayData
{
    // xyz: Ray origin
    // w: Ray intensity
    dvec4 origin_and_intensity;
    // xyz: Ray direction
    // w: Minimum bounding volume distance
    dvec4 direction_and_min_bounding_volume_distance;
};

struct IntersectionData
{
    // xyz: Intersection point
    // w: UNUSED
    dvec4 position;
    // xyz: Intersection normal
    // w: UNUSED
    dvec4 normal;
};

struct OutputData
{
    // xyz: Force vector
    // w: UNUSED
    dvec4 force;
    // xyz: Torque vector
    // w: UNUSED
    dvec4 torque;
};

struct ParticleMaterialLinearOneDirectional
{
    // xyz: Direction times gradient
    // w: Index of refraction sum term
    dvec4 direction_times_gradient_and_ior_sum_term;
};

// Ray buffer data
layout(std430, binding = 0) readonly buffer rayData
{ RayData ray_data[]; };
layout(std430, binding = 1) readonly buffer rayIndexData
{ int ray_index[]; };

// Reflected rays
layout(std430, binding = 2) buffer reflectedRayData
{ RayData reflected_ray_data[]; };
layout(std430, binding = 3) buffer reflectedRayIndexData
{ int reflected_ray_index[]; };

// Refracted rays
layout(std430, binding = 4) buffer refractedRayData
{ RayData refracted_ray_data[]; };
layout(std430, binding = 5) buffer refractedRayIndexData
{ int refracted_ray_index[]; };

// Intersection buffer data
layout(std430, binding = 6) readonly buffer intersectionData
{ IntersectionData intersection_data[]; };

// Particle material data
layout(std430, binding = 7) readonly buffer materialData
{ ParticleMaterialLinearOneDirectional material_data[]; };

// Output buffer data
layout(std430, binding = 8) buffer outputForceData
{ OutputData output_scattered[]; };

// Property data
uniform uint num_rays_per_buffer;
uniform uint num_particles;
uniform double parameter_medium_reflection_index;
uniform double parameter_intensity_cutoff;
uniform bool inner_particle_rays;

// Port ParticleMaterialLinearOneDirectional::indexOfRefraction(const Vec3&)
double particleMaterialIndexOfRefrection(const int pi, const dvec3 pos)
{
    return material_data[pi].direction_times_gradient_and_ior_sum_term.w +
        dot(material_data[pi].direction_times_gradient_and_ior_sum_term.xyz, pos);
}

// Port UnpolirzedLight1DLinearIndexGradientInteraction::reflectance
double reflectance(double cos_a, double cos_b, double n_r)
{
    const double cos2_a = cos_a * cos_a;
    const double cos2_b = cos_b * cos_b;
    const double t = (cos2_a + cos2_b) + (n_r + 1.0 / n_r) * cos_a * cos_b;
    return (cos2_a - cos2_b) * (cos2_a - cos2_b) / (t * t);
}

// Shader main
void main()
{
    // Calculate the ray index
    const uint ri =
        gl_WorkGroupID.x * gl_WorkGroupSize.x + gl_LocalInvocationID.x;

    // Check if local work item is within bounds
    if (ri >= num_rays_per_buffer)
        return;

    // Get particle index
    const int pi = ray_index[ri];

    // Check if particle index is legal
    if (pi >= num_particles)
        return;
    else if (pi < 0)
    {
        reflected_ray_index[ri] = -1;
        refracted_ray_index[ri] = -1;
        return;
    }

    double n_r = parameter_medium_reflection_index /
        particleMaterialIndexOfRefrection(pi, intersection_data[ri].position.xyz);
    if (inner_particle_rays)
        n_r = 1.0 / n_r;

    const dvec3 ray_origin = ray_data[ri].origin_and_intensity.xyz;
    const double ray_intensity = ray_data[ri].origin_and_intensity.w;
    const dvec3 ray_direction =
        ray_data[ri].direction_and_min_bounding_volume_distance.xyz;

    const double cos_a = -dot(ray_direction, intersection_data[ri].normal.xyz);
    if (1.0 - n_r * n_r * (1.0 - cos_a * cos_a) >= 0.0)
    {
        const double cos_b = sqrt(1.0 - n_r * n_r * (1.0 - cos_a * cos_a));
        const double R = reflectance(cos_a, cos_b, n_r);

        // Refracted ray
        refracted_ray_data[ri].origin_and_intensity.w = ray_intensity * (1.0 - R);
        if (refracted_ray_data[ri].origin_and_intensity.w > parameter_intensity_cutoff)
        {
            refracted_ray_data[ri].origin_and_intensity.xyz =
                intersection_data[ri].position.xyz;
            refracted_ray_data[ri].direction_and_min_bounding_volume_distance.xyz =
                n_r * ray_direction +
                intersection_data[ri].normal.xyz * (-cos_b + n_r * cos_a);
            refracted_ray_data[ri].direction_and_min_bounding_volume_distance.w = 0.0;
            refracted_ray_index[ri] = pi;

            output_scattered[ri].force.xyz +=
                refracted_ray_data[ri].origin_and_intensity.w * (ray_direction -
                 refracted_ray_data[ri].direction_and_min_bounding_volume_distance.xyz);
            // output_torque_scattered
        }
        else
        {
            refracted_ray_index[ri] = -1;
            const dvec3 delta_direction = (n_r - 1.0) * ray_direction +
                intersection_data[ri].normal.xyz * (cos_b - n_r * cos_a);
            output_scattered[ri].force.xyz +=
                refracted_ray_data[ri].origin_and_intensity.w *
                delta_direction;
            // output_torque_scattered
        }

        // Reflacted ray
        reflected_ray_data[ri].origin_and_intensity.w = ray_intensity * R;
        if (reflected_ray_data[ri].origin_and_intensity.w > parameter_intensity_cutoff)
        {
            reflected_ray_data[ri].origin_and_intensity.xyz =
                intersection_data[ri].position.xyz;
            reflected_ray_data[ri].direction_and_min_bounding_volume_distance.xyz =
                ray_direction + intersection_data[ri].normal.xyz * 2.0 * cos_a;
            reflected_ray_data[ri].direction_and_min_bounding_volume_distance.w = 0.0;
            reflected_ray_index[ri] = pi;

            output_scattered[ri].force.xyz +=
                reflected_ray_data[ri].origin_and_intensity.w * (ray_direction -
                 reflected_ray_data[ri].direction_and_min_bounding_volume_distance.xyz);
            // output_torque_scattered
        }
        else
        {
            reflected_ray_index[ri] = -1;
            dvec3 delta_direction = intersection_data[ri].normal.xyz * (-2.0 * cos_a);
            output_scattered[ri].force.xyz +=
                reflected_ray_data[ri].origin_and_intensity.w * delta_direction;
            // output_torque_scattered
        }
    }
    else
    {
        // Total reflected ray
        refracted_ray_index[ri] = -1;

        reflected_ray_index[ri] = pi;
        reflected_ray_data[ri].origin_and_intensity.xyz =
            intersection_data[ri].position.xyz;
        reflected_ray_data[ri].origin_and_intensity.w = ray_intensity;
        reflected_ray_data[ri].direction_and_min_bounding_volume_distance.xyz =
            ray_direction + intersection_data[ri].normal.xyz * 2.0 * cos_a;
        reflected_ray_data[ri].direction_and_min_bounding_volume_distance.w = 0.0;

        output_scattered[ri].force.xyz += ray_intensity * (ray_direction -
            reflected_ray_data[ri].direction_and_min_bounding_volume_distance.xyz);
        // output_torque_scattered
    }
}
