// Part of the WWU/LDPLAB Project
// Compute Shader: RodParticleIntersectionTest
#version 460 core

// Local work group size
layout(local_size_x = 64) in;

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

struct ParticleGeometryRodParticle
{
    double cap_origin_z;
    double indentation_origin_z;
    double cylinder_radius;
    double cylinder_length;
    double sphere_radius;
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

// Intersection buffer data
layout(std430, binding = 2) buffer intersectionData
{ IntersectionData intersection_data[]; };
layout(std430, binding = 3) buffer intersectionIndexData
{ int intersection_index[]; };

// Particle data
layout(std430, binding = 4) readonly buffer particleData
{ ParticleGeometryRodParticle particle_data[]; };

// Particle to world transformation
layout(std430, binding = 5) readonly buffer p2wTransformationData
{ Transformation p2w_data[]; };

// Uniforms
uniform uint num_rays_per_buffer;
uniform uint num_particles;

// Port RodParticleIntersectionTest::cylinderIntersection
bool cylinderIntersection(
    const uint ri,
    const int pi,
    inout double dist_min,
    inout double dist_max)
{
    const double p = dot(ray_data[ri].origin_and_intensity.xy,
        ray_data[ri].direction_and_min_bounding_volume_distance.xy) /
        dot(ray_data[ri].direction_and_min_bounding_volume_distance.xy,
            ray_data[ri].direction_and_min_bounding_volume_distance.xy);
    const double q = (dot(ray_data[ri].origin_and_intensity.xy,
        ray_data[ri].origin_and_intensity.xy) -
        particle_data[pi].sphere_radius * particle_data[pi].sphere_radius) /
        dot(ray_data[ri].direction_and_min_bounding_volume_distance.xy,
            ray_data[ri].direction_and_min_bounding_volume_distance.xy);

    const double discriminant = p * p - q;
    if (discriminant < 0.0)
        return false;
    dist_min = -p - sqrt(discriminant);
    dist_max = -p + sqrt(discriminant);
    return true;
}

// Port RodParticleIntersectionTest::sphereIntersection
bool sphereIntersection(
    const uint ri,
    const double sphere_origin_z,
    const double sphere_radius,
    inout double dist_min,
    inout double dist_max)
{
    const dvec3 o_minus_c =
        ray_data[ri].origin_and_intensity.xyz - dvec3(0, 0, sphere_origin_z);
    const double p =
        dot(ray_data[ri].direction_and_min_bounding_volume_distance.xyz, o_minus_c);
    const double q = dot(o_minus_c.xyz, o_minus_c.xyz) -
        sphere_radius * sphere_radius;

    const double discriminant = p * p - q;
    if (discriminant < 1e-9)
        return false;
    dist_min = -p - sqrt(discriminant);
    dist_max = -p + sqrt(discriminant);
    return true;
}

// Port RodParticleIntersectionTest::capIntersection
bool capIntersection(
    const uint ri,
    const int pi)
{
    if (particle_data[pi].indentation_origin_z + particle_data[pi].sphere_radius < 1e-3)
    {
        // Kappa is too small (or 0) and therefore assume the shape as perfect
        // cylinder.
        if (ray_data[ri].direction_and_min_bounding_volume_distance.z == 0)
            return false;
        const double t =
            (particle_data[pi].cylinder_length - ray_data[ri].origin_and_intensity.z) /
            ray_data[ri].direction_and_min_bounding_volume_distance.z;
        if (t <= 1e-9)
            return false;
        intersection_data[ri].position.xyz = ray_data[ri].origin_and_intensity.xyz +
            t * ray_data[ri].direction_and_min_bounding_volume_distance.xyz;
        if (dot(intersection_data[ri].position.xy, intersection_data[ri].position.xy) >
            particle_data[pi].cylinder_radius * particle_data[pi].cylinder_radius)
            return false;
        intersection_data[ri].normal.xyz = dvec3(0, 0, 1);
        return true;
    }

    double isec_first = 0;
    double isec_second = 0;
    if (sphereIntersection(
        ri,
        particle_data[pi].cap_origin_z,
        particle_data[pi].sphere_radius,
        isec_first,
        isec_second))
    {
        if (isec_first < 0)
            return false;
        intersection_data[ri].position.xyz = ray_data[ri].origin_and_intensity.xyz +
            isec_first * ray_data[ri].direction_and_min_bounding_volume_distance.xyz;
        if (intersection_data[ri].position.z > particle_data[pi].cylinder_length &&
            intersection_data[ri].position.z <= particle_data[pi].cap_origin_z +
                particle_data[pi].sphere_radius)
        {
            intersection_data[ri].normal.xyz = normalize(
                intersection_data[ri].position.xyz -
                dvec3(0, 0, particle_data[pi].cap_origin_z));
            return true;
        }
        else
            return false;
    }
    else
        return false;
}

// Port RodParticleIntersectionTest::indentationIntersection
bool indentationIntersection(
    const uint ri,
    const int pi)
{
    if (particle_data[pi].indentation_origin_z + particle_data[pi].sphere_radius < 1e-3)
    {
        // Kappa is too small (or 0) and therefore assume the shape as perfect
        // cylinder.
        if (ray_data[ri].direction_and_min_bounding_volume_distance.z == 0)
            return false;
        const double t = -ray_data[ri].origin_and_intensity.z /
            ray_data[ri].direction_and_min_bounding_volume_distance.z;
        if (t <= 1e-9)
            return false;
        intersection_data[ri].position.xyz =
            ray_data[ri].origin_and_intensity.xyz +
            t * ray_data[ri].direction_and_min_bounding_volume_distance.xyz;
        if (dot(intersection_data[ri].position.xy, intersection_data[ri].position.xy) >
            particle_data[pi].cylinder_radius * particle_data[pi].cylinder_radius)
            return false;
        intersection_data[ri].normal.xyz = dvec3(0, 0, -1);
        return true;
    }

    double isec_first = 0;
    double isec_second = 0;
    if (sphereIntersection(
        ri,
        particle_data[pi].indentation_origin_z,
        particle_data[pi].sphere_radius,
        isec_first,
        isec_second))
    {
        intersection_data[ri].position.xyz = ray_data[ri].origin_and_intensity.xyz +
            isec_second * ray_data[ri].direction_and_min_bounding_volume_distance.xyz;
        if (intersection_data[ri].position.z > 0 &&
            intersection_data[ri].position.z <= particle_data[pi].indentation_origin_z +
                particle_data[pi].sphere_radius)
        {
            intersection_data[ri].normal.xyz = normalize(
                dvec3(0, 0, particle_data[pi].indentation_origin_z) -
                intersection_data[ri].position.xyz);
            return true;
        }
        else
            return false;
    }
    else
        return false;
}

// Port RodParticleIntersectionTest::bottomTopIntersection
bool bottomTopIntersection(
    const uint ri,
    const int pi)
{
    if (ray_data[ri].origin_and_intensity.z <= 0)
    {
        if (ray_data[ri].direction_and_min_bounding_volume_distance.z <= 0)
            return false;
        else
            return indentationIntersection(ri, pi);
    }

    const double particle_height =
        particle_data[pi].cap_origin_z + particle_data[pi].sphere_radius;
    if (ray_data[ri].origin_and_intensity.z >= particle_height)
    {
        if (ray_data[ri].direction_and_min_bounding_volume_distance.z >= 0)
            return false;
        else
            return capIntersection(ri, pi);
    }

    const double dist =
        length(dvec3(0, 0, particle_data[pi].indentation_origin_z) -
            ray_data[ri].origin_and_intensity.xyz);
    if (dist <= particle_data[pi].sphere_radius + 1e-9)
        return indentationIntersection(ri, pi);
    else
        return capIntersection(ri, pi);
}

// Port RodParticleIntersectionTest::intersectionTest
bool intersectionTest(const uint ri, const int pi)
{
    double isec_first;
    double isec_second;

    if (cylinderIntersection(ri, pi, isec_first, isec_second))
    {
        if (isec_first >= 0)
        {
            intersection_data[ri].position.xyz =
                ray_data[ri].origin_and_intensity.xyz +
                isec_first * ray_data[ri].direction_and_min_bounding_volume_distance.xyz;
            if (intersection_data[ri].position.z >= 0 &&
                intersection_data[ri].position.z <= particle_data[pi].cylinder_length)
            {
                intersection_data[ri].normal.xy = normalize(intersection_data[ri].position.xy);
                intersection_data[ri].normal.zw = dvec2(0.0);
                return true;
            }
            else if (intersection_data[ri].position.z < 0.0)
                return indentationIntersection(ri, pi);
            else
                return capIntersection(ri, pi);
        }
        else if (isec_second > 0)
            return bottomTopIntersection(ri, pi);
    }
    else
    {
        double dist =
            dot(ray_data[ri].origin_and_intensity.xy, ray_data[ri].origin_and_intensity.xy);
        if (dist <= particle_data[pi].cylinder_radius * particle_data[pi].cylinder_radius)
            return bottomTopIntersection(ri, pi);
    }

    return false;
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
    if (pi < 0 ||
        pi >= num_particles ||
        pi == intersection_index[ri])
        return;

    // Perform intersection test
    if (intersectionTest(ri, pi))
    {
        intersection_index[ri] = pi;
    }
    else
    {
        ray_data[ri].origin_and_intensity.xyz = (
            p2w_data[pi].translate *
            p2w_data[pi].rotate_scale *
            dvec4(ray_data[ri].origin_and_intensity.xyz, 1)).xyz;
        ray_data[ri].direction_and_min_bounding_volume_distance.xyz =
            normalize(p2w_data[pi].rotate_scale *
            dvec4(ray_data[ri].direction_and_min_bounding_volume_distance.xyz, 1)).xyz;
        ray_index[ri] = int(num_particles);
    }
}
