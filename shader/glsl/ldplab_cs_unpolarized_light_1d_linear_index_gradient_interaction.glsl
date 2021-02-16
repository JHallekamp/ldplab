// Part of the WWU/LDPLAB Project
// Compute Shader: UnpolirzedLight1DLinearIndexGradientInteraction
#version 460 core

// Local work group size
layout(local_size_x = 64) in;

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

// 6

// Ray buffer data
layout(std430, binding = 0) readonly buffer rayOriginData
{ dvec4 ray_origin[]; };
layout(std430, binding = 1) readonly buffer rayDirectionData
{ dvec4 ray_direction[]; };
layout(std430, binding = 2) readonly buffer rayIntensityData
{ double ray_intensity[]; };
layout(std430, binding = 3) readonly buffer rayIndexData
{ int ray_index[]; };

// Reflected rays
layout(std430, binding = 4) buffer reflectedRayOriginData
{ dvec4 reflected_ray_origin[]; };
layout(std430, binding = 5) buffer reflectedRayDirectionData
{ dvec4 reflected_ray_direction[]; };
layout(std430, binding = 6) buffer reflectedRayIntensityData
{ double reflected_ray_intensity[]; };
layout(std430, binding = 7) buffer reflectedRayIndexData
{ int reflected_ray_index[]; };
layout(std430, binding = 8) buffer reflectedRayMinBoundingVolumeDistanceData
{ double reflected_ray_min_bounding_volume_distance[]; };

// Refracted rays
layout(std430, binding = 9) buffer refractedRayOriginData
{ dvec4 refracted_ray_origin[]; };
layout(std430, binding = 10) buffer refractedRayDirectionData
{ dvec4 refracted_ray_direction[]; };
layout(std430, binding = 11) buffer refractedRayIntensityData
{ double refracted_ray_intensity[]; };
layout(std430, binding = 12) buffer refractedRayIndexData
{ int refracted_ray_index[]; };
layout(std430, binding = 13) buffer refractedRayMinBoundingVolumeDistanceData
{ double refracted_ray_min_bounding_volume_distance[]; };

// Intersectino buffer data
layout(std430, binding = 14) readonly buffer intersectionPointData
{ dvec4 intersection_position[]; };
layout(std430, binding = 15) readonly buffer intersectionNormalData
{ dvec4 intersection_normal[]; };

// Particle material data
layout(std430, binding = 16) readonly buffer particleMaterialIndexOfRefrectionSumTermData
{ double particle_material_index_of_refraction_sum_term[]; };
layout(std430, binding = 17) readonly buffer particleMaterialDirectionTimesGradient
{ dvec4 particle_material_direction_times_gradient[]; };

// Output buffer data
layout(std430, binding = 18) buffer outputForceData
{ dvec4 output_force_scattered[]; };
layout(std430, binding = 19) buffer outputTorqueData
{ dvec4 output_torque_scattered[]; };

// Property data
uniform uint num_rays_per_buffer;
uniform uint num_particles;
uniform double parameter_medium_reflection_index;
uniform double parameter_intensity_cutoff;
uniform bool inner_particle_rays;

// Port ParticleMaterialLinearOneDirectional::indexOfRefraction(const Vec3&)
double particleMaterialIndexOfRefrection(const int pi, const dvec3 pos)
{
    return particle_material_index_of_refraction_sum_term[pi] +
        dot(particle_material_direction_times_gradient[pi].xyz, pos);
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
        particleMaterialIndexOfRefrection(pi, intersection_position[ri].xyz);
    if (inner_particle_rays)
        n_r = 1.0 / n_r;
    const double cos_a = -dot(ray_direction[ri], intersection_normal[ri]);
    if (1.0 - n_r * n_r * (1.0 - cos_a * cos_a) >= 0.0)
    {
        const double cos_b = sqrt(1.0 - n_r * n_r * (1.0 - cos_a * cos_a));
        const double R = reflectance(cos_a, cos_b, n_r);

        // Refracted ray
        refracted_ray_intensity[ri] = ray_intensity[ri] * (1.0 - R);
        if (refracted_ray_intensity[ri] > parameter_intensity_cutoff)
        {
            refracted_ray_index[ri] = ray_index[ri];
            refracted_ray_min_bounding_volume_distance[ri] = 0.0;
            refracted_ray_origin[ri] = intersection_position[ri];
            refracted_ray_direction[ri] = n_r * ray_direction[ri] +
                intersection_normal[ri] * (-cos_b + n_r * cos_a);

            output_force_scattered[ri] += refracted_ray_intensity[ri] *
                (ray_direction[ri] - refracted_ray_direction[ri]);
            // output_torque_scattered
        }
        else
        {
            refracted_ray_index[ri] = -1;
            dvec4 delta_direction = (n_r - 1.0) * ray_direction[ri] +
                intersection_normal[ri] * (cos_b - n_r * cos_a);
            output_force_scattered[ri] += refracted_ray_intensity[ri] *
                delta_direction;
            // output_torque_scattered
        }

        // Reflacted ray
        reflected_ray_intensity[ri] = ray_intensity[ri] * R;
        if (reflected_ray_intensity[ri] > parameter_intensity_cutoff)
        {
            reflected_ray_index[ri] = ray_index[ri];
            reflected_ray_min_bounding_volume_distance[ri] = 0.0;
            reflected_ray_origin[ri] = intersection_position[ri];
            reflected_ray_direction[ri] = ray_direction[ri] +
                intersection_normal[ri] * 2.0 * cos_a;

            output_force_scattered[ri] += reflected_ray_intensity[ri] *
                (ray_direction[ri] - reflected_ray_direction[ri]);
            // output_torque_scattered
        }
        else
        {
            reflected_ray_index[ri] = -1;
            dvec4 delta_direction = intersection_normal[ri] * (-2.0 * cos_a);
            output_force_scattered[ri] += reflected_ray_intensity[ri] *
                delta_direction;
            // output_torque_scattered
        }
    }
    else
    {
        // Total reflected ray
        refracted_ray_index[ri] = -1;

        reflected_ray_index[ri] = ray_index[ri];
        reflected_ray_min_bounding_volume_distance[ri] = 0.0;
        reflected_ray_origin[ri] = intersection_position[ri];
        reflected_ray_direction[ri] = ray_direction[ri] +
            intersection_normal[ri] * 2.0 * cos_a;
        reflected_ray_intensity[ri] = ray_intensity[ri];

        output_force_scattered[ri] += reflected_ray_intensity[ri] *
            (ray_direction[ri] - reflected_ray_direction[ri]);
        // output_torque_scattered
    }
}
