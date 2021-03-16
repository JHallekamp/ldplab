// Part of the WWU/LDPLAB Project
// Compute Shader: RayBoundingSphereIntersectionTestStageBruteForce
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

struct BoundingSphereData
{
    // xyz: Sphere center
    // w: Sphere radiusgit
    dvec4 center_and_radius;
};

struct Transformation
{
    dmat4 rotate_scale;
    dmat4 translate;
};

// Ray buffer data
layout(std430, binding = 0) buffer rayData
{ RayData ray_data[]; };
layout(std430, binding = 1) buffer rayIndexData
{ int ray_index[]; };

// Bounding sphere data
layout(std430, binding = 2) readonly buffer sphereData
{ BoundingSphereData sphere_data[]; };

// World to particle transformation
layout(std430, binding = 3) readonly buffer w2pTransformationData
{ Transformation w2p_data[]; };

// Property data
uniform uint num_rays_per_buffer;
uniform uint num_particles;

// Shader main
void main()
{
    // Calculate the ray index
    const uint ri =
        gl_WorkGroupID.x * gl_WorkGroupSize.x + gl_LocalInvocationID.x;

    // Check if local work item is within bounds
    if (ri >= num_rays_per_buffer)
        return;

    // Get ray particle index
    const int pi = ray_index[ri];

    // Check if ray index is legal
    if (pi < int(num_particles))
        return;

    double min_d = -1.0;
    int min_i = 0;

    const dvec3 ray_origin = ray_data[ri].origin_and_intensity.xyz;
    const dvec3 ray_direction =
        ray_data[ri].direction_and_min_bounding_volume_distance.xyz;
    const double ray_min_bounding_sphere_dist =
        ray_data[ri].direction_and_min_bounding_volume_distance.w;
    for (int i = 0; i < num_particles; ++i)
    {
        const dvec3 oc = ray_origin - sphere_data[i].center_and_radius.xyz;
        const double sphere_radius = sphere_data[i].center_and_radius.w;
        const double q = dot(oc, oc) - sphere_radius * sphere_radius;
        if (q < 1e-9)
            continue;

        const double p = dot(ray_direction, oc);
        const double discriminant = p * p - q;
        if (discriminant < 1e-9)
            continue;

        const double dist = -p - sqrt(discriminant);
        if (dist <= ray_min_bounding_sphere_dist)
            continue;

        if (dist < min_d || min_d < 0)
        {
            min_d = dist;
            min_i = i;
        }
    }

    if (min_d < 0)
    {
        // ray exists scene
        ray_index[ri] = -1;
    }
    else
    {
        // ray hits particle min_i bounding volume
        ray_index[ri] = min_i;
        ray_data[ri].origin_and_intensity.xyz =
            (
             w2p_data[min_i].rotate_scale *
             w2p_data[min_i].translate *
             dvec4(ray_data[ri].origin_and_intensity.xyz, 1)).xyz;
        ray_data[ri].direction_and_min_bounding_volume_distance.xyz =
            normalize((w2p_data[min_i].rotate_scale *
                dvec4(ray_data[ri].origin_and_intensity.xyz, 1)).xyz);
        ray_data[ri].direction_and_min_bounding_volume_distance.w = min_d;
    }
}
