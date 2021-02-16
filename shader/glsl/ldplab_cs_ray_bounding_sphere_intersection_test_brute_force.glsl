// Part of the WWU/LDPLAB Project
// Compute Shader: RayBoundingSphereIntersectionTestStageBruteForce
#version 460 core

// Local work group size
layout(local_size_x = 64) in;

// Ray buffer data
layout(std430, binding = 0) buffer rayOriginData
{ dvec4 ray_origin[]; };
layout(std430, binding = 1) buffer rayDirectionData
{ dvec4 ray_direction[]; };
layout(std430, binding = 2) buffer rayIndexData
{ int ray_index[]; };
layout(std430, binding = 3) buffer rayMinBoundingVolumeDistanceData
{ double ray_min_bounding_volume_distance[]; };

// Bounding sphere data
layout(std430, binding = 4) readonly buffer boundingSphereCenterData
{ dvec4 bounding_sphere_center[]; }
layout(std430, binding = 5) readonly buffer boundingSphereRadiusData
{ double bounding_sphere_radius[]; }

// World to particle transformation
layout(std430, binding = 6) readonly buffer transformWorld2ParticleMatData
{ dmat4 transform_w2p_mat[]; }
layout(std430, binding = 7) readonly buffer transformWorld2ParticleVecData
{ dvec4 transform_w2p_vec[]; }

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

    // Check if ray index is legal
    if (ray_index[ri] == -1 || ray_index[ri] < num_particles)
        return;

    double min_d = -1.0;
    int min_i = 0;

    const dvec3 origin = ray_origin[ri].xyz;
    for (uint i = 0; i < num_particles; ++i)
    {
        const dvec3 oc = origin - bounding_sphere_center[i].xyz;
        const double q = glm::dot(oc, oc) -
            bounding_sphere_radius[i] * bounding_sphere_radius[i];
        if (q < 1e-9)
            continue;

        const double p = dot(ray_direction[ri], oc);
        const double discriminant = p * p - q;
        if (discriminant < 1e-9)
            continue;

        const double dist = -p - sqrt(discriminant);
        if (dist <= ray_min_bounding_volume_distance[ri])
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
        ray_min_bounding_volume_distance[ri] = min_d;
        ray_origin[ri] =
            transform_w2p_mat[min_i] * (ray_origin[ri] + transform_w2p_vec[min_i]);
        ray_direction[ri] =
            normalize(transform_w2p_mat[min_i] * ray_direction[ri]);
    }
}
