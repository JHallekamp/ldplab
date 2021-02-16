// Part of the WWU/LDPLAB Project
// Compute Shader: RodParticleIntersectionTest
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

// Intersection buffer data
layout(std430, binding = 3) buffer intersectionPointData
{ dvec4 intersection_position[]; };
layout(std430, binding = 4) buffer intersectionNormalData
{ dvec4 intersection_normal[]; };
layout(std430, binding = 5) buffer intersectionIndexData
{ int intersection_index[]; };

// Particle data
layout(std430, binding = 6) readonly buffer particleCylinderRadiusData
{ double particle_cylinder_radius[]; };
layout(std430, binding = 7) readonly buffer particleCylinderLengthData
{ double particle_cylinder_length[]; };
layout(std430, binding = 8) readonly buffer particleSphereRadiusData
{ double particle_sphere_radius[]; };
layout(std430, binding = 9) readonly buffer particleCapOriginData
{ dvec4 particle_cap_origin[]; };
layout(std430, binding = 10) readonly buffer particleIndentationOriginData
{ dvec4 particle_indentation_origin[]; };

// Particle to world transformation
layout(std430, binding = 11) readonly buffer transformParticle2WorldMatData
{ dmat4 transform_p2w_mat[]; };
layout(std430, binding = 12) readonly buffer transformParticle2WorldVecData
{ dvec4 transform_p2w_vec[]; };

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
    const double p = dot(ray_origin[ri].xy, ray_direction[ri].xy) /
        dot(ray_direction[ri].xy, ray_direction[ri].xy);
    const double q = (dot(ray_origin[ri].xy, ray_origin[ri].xy) -
        particle_sphere_radius[pi] * particle_sphere_radius[pi]) /
        dot(ray_direction[ri].xy, ray_direction[ri].xy);

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
    const dvec4 sphere_origin,
    const double sphere_radius,
    inout double dist_min,
    inout double dist_max)
{
    const dvec4 o_minus_c = ray_origin[ri] - sphere_origin;
    const double p = dot(ray_direction[ri].xyz, o_minus_c.xyz);
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
    if (particle_indentation_origin[pi].z + particle_sphere_radius[pi] < 1e-3)
    {
        // Kappa is too small (or 0) and therefore assume the shape as perfect
        // cylinder.
        if (ray_direction[ri].z == 0)
            return false;
        const double t = (particle_cylinder_length[pi] - ray_origin[ri].z) /
            ray_direction[ri].z;
        if (t <= 1e-9)
            return false;
        intersection_position[ri] = ray_origin[ri] + t * ray_direction[ri];
        if (dot(intersection_position[ri].xy, intersection_position[ri].xy) >
            particle_cylinder_radius[pi] * particle_cylinder_radius[pi])
            return false;
        intersection_normal[ri].xyz = dvec3(0, 0, 1);
        return true;
    }

    double isec_first = 0;
    double isec_second = 0;
    if (sphereIntersection(
        ri,
        particle_cap_origin[pi],
        particle_sphere_radius[pi],
        isec_first,
        isec_second))
    {
        if (isec_first < 0)
            return false;
        intersection_position[ri] = ray_origin[ri] + isec_first *
            ray_direction[ri];
        if (intersection_position[ri].z > particle_cylinder_length[pi] &&
            intersection_position[ri].z <= particle_cap_origin[pi].z +
                particle_sphere_radius[pi])
        {
            intersection_normal[ri] = normalize(intersection_position[ri] -
                particle_cap_origin[pi]);
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
    if (particle_indentation_origin[pi].z + particle_sphere_radius[pi] < 1e-3)
    {
        // Kappa is too small (or 0) and therefore assume the shape as perfect
        // cylinder.
        if (ray_direction[ri].z == 0)
            return false;
        const double t = -ray_origin[ri].z / ray_direction[ri].z;
        if (t <= 1e-9)
            return false;
        intersection_position[ri] = ray_origin[ri] + t * ray_direction[ri];
        if (dot(intersection_position[ri].xy, intersection_position[ri].xy) >
            particle_cylinder_radius[pi] * particle_cylinder_radius[pi])
            return false;
        intersection_normal[ri].xyz = dvec3(0, 0, -1);
        return true;
    }

    double isec_first = 0;
    double isec_second = 0;
    if (sphereIntersection(
        ri,
        particle_indentation_origin[pi],
        particle_sphere_radius[pi],
        isec_first,
        isec_second))
    {
        intersection_position[ri] = ray_origin[ri] + isec_second *
            ray_direction[ri];
        if (intersection_position[ri].z > 0 &&
            intersection_position[ri].z <= particle_indentation_origin[pi].z +
                particle_sphere_radius[pi])
        {
            intersection_normal[ri] = normalize(particle_indentation_origin[pi] -
                intersection_position[ri]);
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
    if (ray_origin[ri].z <= 0)
    {
        if (ray_direction[ri].z <= 0)
            return false;
        else
            return indentationIntersection(ri, pi);
    }

    const double particle_height =
        particle_cap_origin[pi].z + particle_sphere_radius[pi];
    if (ray_origin[ri].z >= particle_height)
    {
        if (ray_direction[ri].z >= 0)
            return false;
        else
            return capIntersection(ri, pi);
    }

    const double dist =
        length(particle_indentation_origin[pi].xyz - ray_origin[ri].xyz);
    if (dist <= particle_sphere_radius[pi] + 1e-9)
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
            intersection_position[ri] = ray_origin[ri] + isec_first *
                ray_direction[ri];
            if (intersection_position[ri].z >= 0 &&
                intersection_position[ri].z <= particle_cylinder_length[pi])
            {
                intersection_normal[ri].xy = normalize(intersection_position[ri].xy);
                intersection_normal[ri].zw = dvec2(0.0);
                return true;
            }
            else if (intersection_position[ri].z < 0.0)
                return indentationIntersection(ri, pi);
            else
                return capIntersection(ri, pi);
        }
        else if (isec_second > 0)
            return bottomTopIntersection(ri, pi);
    }
    else
    {
        double dist = dot(ray_origin[ri].xy, ray_origin[ri].xy);
        if (dist <= particle_cylinder_radius[pi] * particle_cylinder_radius[pi])
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
        intersection_index[ri] = ray_index[ri];
    }
    else
    {
        ray_origin[ri] =
            transform_p2w_mat[pi] * ray_origin[ri] + transform_p2w_vec[pi];
        ray_direction[ri] = normalize(transform_p2w_mat[pi] * ray_direction[ri]);
        ray_index[ri] = int(num_particles);
    }
}
