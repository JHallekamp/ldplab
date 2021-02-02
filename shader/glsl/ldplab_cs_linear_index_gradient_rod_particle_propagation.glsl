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
layout(std430, binding = 6) buffer outputForceData
{ dvec3 output_force_scattered[]; };
layout(std430, binding = 7) buffer outputTorqueData
{ dvec3 output_torque_scattered[]; };

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
uniform double parameter_safety_factor;

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
    // Define constant errors
    const double alpha[6] = float[](
            0.0, 1.0/4.0, 3.0/8.0, 12.0/13.0, 1.0, 1.0/2.0
        );
    const double beta[36] = float[](
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ,
            1.0 / 4.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            3.0 / 32.0, 9.0 / 32.0, 0.0, 0.0, 0.0, 0.0,
            1932.0 / 2197.0, (-7200.0) / 2197.0, 7296.0 / 2197.0, 0.0, 0.0, 0.0,
            439.0 / 216.0, -8.0, 3680.0 / 513.0, (-845.0) / 4104.0, 0.0, 0.0,
            (-8.0) / 27.0, 2.0, (-3544.0) / 2565.0, 1859.0 / 4104.0, (-11.0) / 40.0, 0.0
        );
    const double c[6] = float[](
            25.0 / 216.0, 0.0, 1408.0 / 2565.0, 2197.0 / 4104.0, (-1.0) / 5.0, 0.0
        );
    const double c_star[6] = float[](
             16.0 / 135.0, 0.0, 6656.0 / 12825.0, 28561.0 / 56430.0, (-9.0) / 50.0, 2.0 / 55.0
        );
    const double c_err[6] = float[](
            -0.00277778,  0.0 ,  0.02994152,  0.02919989, -0.02 , -0.03636364
        );

    // Setup
    dvec3 k_w[6];
    dvec3 k_r[6];
    dvec3 err_w = dvec3(0, 0, 0);
    dvec3 err_r = dvec3(0, 0, 0);
    dvec3 x_step_w;
    dvec3 x_step_r;
    x_new_w = dvec3(0, 0, 0);
    x_new_r = dvec3(0, 0, 0);

    // Calculate
    for (int i = 0; i < 6; ++i)
    {
        x_step_w = x_w;
        x_step_r = x_r;
        for (int j = 0; j < i; ++j)
        {
            const double hb = h * beta[i * 6 + j];
            x_step_w += k_w[j] * hb;
            x_step_r += k_r[j] * hb;
        }
        // eikonal(particle, x_step)
        k_w[i] = particle_material_direction_times_gradient[pi];
        k_r[i] = x_step_w / particleMaterialIndexOfRefrection(pi, x_step_r);

        err_w += k_w[i] * c_err[i];
        err_r += k_r[i] * c_err[i];

        x_new_w += k_w[i] * c_star[i];
        x_new_r += k_r[i] * c_star[i];
    }

    x_new_w = (x_new_w * h) + x_w;
    x_new_r = (x_new_r * h) + x_r;
    err_w *= h;
    err_r *= h;

    // return error.absoluteMax();
    return max(
        max( max(abs(err_w.x), abs(err_w.y)), abs(err_w.z) ),
        max( max(abs(err_r.x), abs(err_r.y)), abs(err_r.z) )
        );
}

// Port LinearIndexGradientRodParticlePropagation::isOutsideParticle
bool isOutsideParticle(const int pi, const dvec3 r)
{
    if (dot(r.xy, r.xy) >
        particle_cylinder_radius[pi] * particle_cylinder_radius[pi])
    {
        return true;
    }

    if (r.z >= 0 &&
        r.z <= particle_cap_origin[pi].z + particle_sphere_radius[pi])
    {
        if (r.z > particle_cylinder_length[pi])
        {
            if (dot(r.xy, r.xy) >
                particle_sphere_radius[pi] * particle_sphere_radius[pi] -
                pow(r.z - particle_cap_origin.z, 2.0))
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else if (r.z <
            particle_indentation_origin[pi].z + particle_sphere_radius[pi])
        {
            if (dot(r.xy, r.xy) <
                particle_sphere_radius[pi] * particle_sphere_radius[pi] -
                pow(r.z - particle_indentation_origin.z, 2.0))
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else
            return false;
    }
    else
        return true;
}

// Port LinearIndexGradientRodParticlePropagation::intersection
void intersection(
    const uint ri,
    const int pi,
    const dvec3 x_w,
    const dvec3 x_r)
{

}

// Port LinearIndexGradientRodParticlePropagation::cylinderIntersection
bool cylinderIntersection(
    const uint ri,
    const int pi,
    const dvec3 origin,
    const dvec3 direction)
{
    const double p =
        dot(origin.xy, direction.xy) / dot(direction.xy, direction.xy);
    const double q =
        (dot(origin.xy, origin.xy) -
         particle_cylinder_radius[pi] * particle_cylinder_radius[pi]) /
        dot(direction.xy, direction.xy);
    const double discriminant = p * p - q;

    if (discriminant < 0.0)
        return false;
    const double t = -p + sqrt(discriminant);
    if (t <= 1e-9)
        return false;

    intersection_position[ri] = origin + direction * t;
    if (intersection_position[ri].z >= 0 &&
        intersection_position[ri].z <= particle_cylinder_length[pi])
    {
        intersection_normal[ri] = normalize(dvec3(
            -intersection_position[ri].x,
            -intersection_position[ri].y,
            0 ));
        return true;
    }
    return false;
}

// Port LinearIndexGradientRodParticlePropagation::capIntersection
bool capIntersection(
    const uint ri,
    const int pi,
    const dvec3 origin,
    const dvec3 direction)
{
    if (particle_indentation_origin[pi].z + particle_sphere_radius[pi] < 1e-3)
    {
        // Kappa is too small (or 0) and therefore assume the shape as perfect
        // cylinder.
        if (direction.z == 0)
            return false;
        const double t = (particle_cylinder_length[pi] - origin.z) / direction.z;
        if (t <= 1e-9)
            return false;
        intersection_position[ri] = origin + t * direction;
        if (dot(intersection_position[ri].xy, intersection_position[ri].xy) >
            particle_cylinder_radius[pi] * particle_cylinder_radius[pi])
            return false;
        intersection_normal[ri] = dvec3(0, 0, 1);
        return true;
    }

    const dvec3 o_minus_c = origin - particle_indentation_origin[pi];
    const double p = dot(direction, o_minus_c);
    const double q = dot(o_minus_c, o_minus_c) -
        particle_sphere_radius[pi] * particle_sphere_radius[pi];
    const double discriminant = p * p - q;

    if (discriminant < 0.0)
        return false;
    const double t = -p + sqrt(discriminant);
    if (t <= 1e-9)
        return false;

    intersection_position[ri] = origin + t * direction;
    if (intersection_position[ri].z > 0 &&
        intersection_position[ri].z <=
            particle_indentation_origin[pi].z + particle_sphere_radius[pi])
    {
        intersection_normal[ri] =
            normalize(intersection_position[ri] - particle_indentation_origin[pi]);
        return true;
    }
    return false;
}

// Port LinearIndexGradientRodParticlePropagation::indentationIntersection
bool indentationIntersection(
    const uint ri,
    const int pi,
    const dvec3 origin,
    const dvec3 direction)
{
    if (particle_indentation_origin[pi].z + particle_sphere_radius[pi] < 1e-3)
    {
        // Kappa is too small (or 0) and therefore assume the shape as perfect
        // cylinder.
        if (direction.z == 0)
            return false;
        const double t = -origin.z / direction.z;
        if (t <= 1e-9)
            return false;
        intersection_position[ri] = origin + t * direction;
        if (dot(intersection_position[ri].xy, intersection_position[ri].xy) >
            particle_cylinder_radius[pi] * particle_cylinder_radius[pi])
            return false;
        intersection_normal[ri] = dvec3(0, 0, -1);
        return true;
    }

    const dvec3 o_minus_c = origin - particle_cap_origin[pi];
    const double p = dot(direction, o_minus_c);
    const double q = dot(o_minus_c, o_minus_c) -
        particle_sphere_radius[pi] * particle_sphere_radius[pi];
    const double discriminant = p * p - q;

    if (discriminant < 0.0)
        return false;
    const double t = -p + sqrt(discriminant);
    if (t <= 1e-9)
        return false;

    intersection_position[ri] = origin + t * direction;
    if (intersection_position[ri].z > particle_cylinder_length[pi] &&
        intersection_position[ri].z <=
            particle_cap_origin[pi].z + particle_sphere_radius[pi])
    {
        intersection_normal[ri] =
            normalize(particle_cap_origin[pi] - intersection_position[ri]);
        return true;
    }
    return false;
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
    dvec3 x_r = ray_origin[ri];
    dvec3 x_w = ray_direction[ri] * indexOfRefraction(pi, x_r);
    dvec3 x_new_r:
    dvec3 x_new_w;

    // Setup iteration
    double h = parameter_initial_step_size;
    double error = 0;
    while (true)
    {
        // Use rk45
        error = rk45(pi, x_w, x_r, h, x_new_w, x_new_r);
        if (error <= parameter_epsilon)
        {
            if (isOutsideParticle(pi, x_new_r))
            {
                // new ray direction
                x_w = normalize(x_w);
                output_force_scattered[ri] +=
                    ray_intensity[ri] * (x_w - ray_direction[ri]);
                //output_torque_scattered[ri] += ray_intensity[ri] * cross();
                intersection(ri, pi, x_w, x_r);
                ray_direction[ri] = x_w;
                ray_origin[ri]  = x_r;
                return;
            }
            else
            {
                x_w = x_new_w;
                x_r = x_new_r;
                // pow only genTypes not genDTypes?
                h = parameter_safety_factor * h *
                    pow(parameter_epsilon / error, 0.2);
            }
        }
        else
        {
            // pow only genTypes not genDTypes?
            h = parameter_safety_factor * h *
                pow(parameter_epsilon / error, 0.25);
        }
    }
}
