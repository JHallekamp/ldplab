#include "ImplInnerParticlePropagation.hpp"

#include "../../Utils/Log.hpp"

#include <LDPLAB/RayTracingStep/CPU/IGenericGeometry.hpp>
#include <complex>
#include <cmath>

ldplab::rtscpu::EikonalSolverRK4LinearIndexGradient::EikonalSolverRK4LinearIndexGradient(
    RK4Parameter parameters)
    :
    m_parameters{ parameters }
{
}

void ldplab::rtscpu::EikonalSolverRK4LinearIndexGradient::execute(
    RayBuffer& ray_data, 
    IntersectionBuffer& intersection_data, 
    OutputBuffer& output_data, 
    const std::vector<std::shared_ptr<IGenericGeometry>>& geometry_data, 
    const std::vector<std::shared_ptr<IParticleMaterial>>& material_data, 
    const std::vector<Vec3>& center_of_mass, 
    const SimulationParameter& simulation_parameter, 
    void* stage_dependent_data)
{
    LDPLAB_LOG_TRACE("RTSCPU %i: Execute inner particle ray propagation "\
        "on batch buffer %i",
        getParentRayTracingStepUID(), ray_data.uid);

    for (size_t i = 0; i < ray_data.size; i++)
    {
        if (ray_data.index_data[i] <= simulation_parameter.ray_invalid_index ||
            ray_data.index_data[i] >= simulation_parameter.ray_world_space_index)
            continue;

        const size_t particle_index = ray_data.index_data[i];
        rayPropagation(
            particle_index,
            ray_data.ray_data[i],
            intersection_data.point[i],
            intersection_data.normal[i],
            geometry_data[particle_index],
            material_data[particle_index],
            center_of_mass[particle_index],
            output_data);
    }

    LDPLAB_LOG_TRACE("RTSCPU %i: Inner particle ray propagation on "\
        "buffer %i completed",
        getParentRayTracingStepUID(),
        ray_data.uid);
}

void ldplab::rtscpu::EikonalSolverRK4LinearIndexGradient::
rayPropagation(
    size_t particle_index,
    Ray& ray,
    Vec3& inter_point,
    Vec3& inter_normal,
    const std::shared_ptr<IGenericGeometry>& particle_geometry,
    const std::shared_ptr<IParticleMaterial>& particle_material,
    const Vec3& particle_center_of_mass,
    OutputBuffer& output) const
{
    bool intersected = false;
    bool is_inside = false;
    Arg x{
        ray.direction * particle_material->indexOfRefraction(ray.origin),
        ray.origin };
    Arg x_new{};
    while (!intersected)
    {
        rk4(particle_material, x, m_parameters.step_size, x_new);
        intersected = particle_geometry->intersectSegment(
            x.r,
            x_new.r,
            inter_point,
            inter_normal,
            is_inside);
        if (intersected || !is_inside)
        {
            if (!intersected)
            {
                // The following check is neccessary to test if the ray hits in
                // an extreme point (volume so small that it lies within the 
                // epsilon region). If that is the case, we assume the ray 
                // tunnels through the particle.
                bool intersect_outside = false;
                Vec3 t_ip, t_in;
                intersected = particle_geometry->intersectRay(
                    ray,
                    t_ip,
                    t_in,
                    intersect_outside);
                if (!intersected || intersect_outside)
                {
                    // We have found a case, where the ray tunnels through the
                    // Particle. We use the original ray and invalidate the 
                    // surface normal.
                    inter_point = ray.origin;
                    inter_normal = Vec3(0, 0, 0);
                }
                else
                {
                    // We have found a case, where the initial ray is bend 
                    // out of the particle in the initial step due to the 
                    // particle material gradient. In this case we assume that 
                    // it hits in the original intersection point.
                    // We use the previous normal and have to flip it, to 
                    // ensure that we have correct behaviour in the interaction
                    // stage.
                    // To receive the previous normal, we simply perform the
                    // Intersection test again, but this time we reverse the
                    // segment directions. Then we flip the normal.
                    particle_geometry->intersectSegment(
                        x_new.r,
                        x.r,
                        inter_point,
                        inter_normal,
                        is_inside);
                    inter_point = ray.origin;
                    inter_normal = -inter_normal;
                    ray.direction = glm::normalize(x.w);
                }
            }
            else
            {
                ray.direction = glm::normalize(x.w);
                const double nx = particle_material->indexOfRefraction(x.r);
                const double ny = particle_material->indexOfRefraction(inter_point);
                const Vec3 delta_momentum = (nx - ny) * ray.direction;
                const Vec3 r = inter_point - particle_center_of_mass;
                output.force[particle_index] += ray.intensity *
                    delta_momentum;
                output.torque[particle_index] += ray.intensity *
                    glm::cross(r, delta_momentum);
            }
            return;
        }
        else
        {
            const double nx = particle_material->indexOfRefraction(x.r);
            const double ny = particle_material->indexOfRefraction(x_new.r);
            const Vec3 t_old_direction = glm::normalize(x.w);
            const Vec3 t_new_direction = glm::normalize(x_new.w);
            const Vec3 delta_momentum = nx * t_old_direction -
                ny * t_new_direction;
            const Vec3 r = x_new.r - particle_center_of_mass;
            output.force[particle_index] += ray.intensity *
                delta_momentum;
            output.torque[particle_index] += ray.intensity *
                glm::cross(r, delta_momentum);
            x = x_new;
        }
    }
}

void ldplab::rtscpu::EikonalSolverRK4LinearIndexGradient::rk4(
    const std::shared_ptr<IParticleMaterial>& particle_material, 
    const Arg& x, 
    const double h, 
    Arg& x_new) const
{
    const ParticleMaterialLinearOneDirectional* raw_material =
        static_cast<ParticleMaterialLinearOneDirectional*>(particle_material.get());
    Arg k[4]{};
    const double beta[4] = { 1.0, 0.5, 0.5, 1.0 };
    const double c[4] = { 1.0, 2.0, 2.0, 1.0 };
    x_new = { {0,0,0}, {0,0,0} };
    for (size_t i = 0; i < 4; ++i)
    {
        Arg x_step = x;
        if (i > 0)
        {
            const double hb = h * beta[i];
            x_step.w.x += k[i - 1].w.x * hb;
            x_step.w.y += k[i - 1].w.y * hb;
            x_step.w.z += k[i - 1].w.z * hb;
            x_step.r.x += k[i - 1].r.x * hb;
            x_step.r.y += k[i - 1].r.y * hb;
            x_step.r.z += k[i - 1].r.z * hb;
        }
        k[i].w = raw_material->direction_times_gradient;
        const double index_of_refraction =
            1.0 / raw_material->indexOfRefraction(x_step.r);
        k[i].r.x = x_step.w.x * index_of_refraction;
        k[i].r.y = x_step.w.y * index_of_refraction;
        k[i].r.z = x_step.w.z * index_of_refraction;

        if (c[i] != 0.0)
        {
            x_new.w.x += k[i].w.x * c[i];
            x_new.w.y += k[i].w.y * c[i];
            x_new.w.z += k[i].w.z * c[i];
            x_new.r.x += k[i].r.x * c[i];
            x_new.r.y += k[i].r.y * c[i];
            x_new.r.z += k[i].r.z * c[i];
        }
    }
    x_new *= h / 6.0;
    x_new += x;
}

namespace
{
    ldplab::Vec3c normalize(ldplab::Vec3c vec)
    {
        const std::complex<double> norm =
            std::sqrt(std::conj(vec.x) * vec.x +
            std::conj(vec.y) * vec.y +
            std::conj(vec.z) * vec.z);
        if (norm.real() < 1e-10)
            return { 0,0,0 };
        return vec / norm;
    }
    std::complex<double> dot(ldplab::Vec3c vec1, ldplab::Vec3c vec2)
    {
        const std::complex<double> dot =
            std::conj(vec2.x) * vec1.x +
            std::conj(vec2.y) * vec1.y +
            std::conj(vec2.z) * vec1.z;
        return dot;
    }

}

ldplab::rtscpu::EikonalSolverRK4LinearIndexGradientPolarization::EikonalSolverRK4LinearIndexGradientPolarization(
    RK4Parameter parameters)
    :
    m_parameters{ parameters }
{
}

void ldplab::rtscpu::EikonalSolverRK4LinearIndexGradientPolarization::execute(
    RayBuffer& ray_data,
    IntersectionBuffer& intersection_data,
    OutputBuffer& output_data,
    const std::vector<std::shared_ptr<IGenericGeometry>>& geometry_data,
    const std::vector<std::shared_ptr<IParticleMaterial>>& material_data,
    const std::vector<Vec3>& center_of_mass,
    const SimulationParameter& simulation_parameter,
    void* stage_dependent_data)
{
    LDPLAB_LOG_TRACE("RTSCPU %i: Execute inner particle ray propagation "\
        "on batch buffer %i",
        getParentRayTracingStepUID(), ray_data.uid);

    auto& polarization = ((default_factories::InitialStageHomogenousPolarizedLightBoundingSphereProjectionFactory::PolarizationData*)
        stage_dependent_data)->polarization_buffers[ray_data.depth];



    for (size_t i = 0; i < ray_data.size; i++)
    {
        if (ray_data.index_data[i] <= simulation_parameter.ray_invalid_index ||
            ray_data.index_data[i] >= simulation_parameter.ray_world_space_index)
            continue;

        const size_t particle_index = ray_data.index_data[i];
        rayPropagation(
            particle_index,
            ray_data.ray_data[i],
            intersection_data.point[i],
            intersection_data.normal[i],
            polarization.polarization_data[i],
            geometry_data[particle_index],
            material_data[particle_index],
            center_of_mass[particle_index],
            output_data);
    }

    LDPLAB_LOG_TRACE("RTSCPU %i: Inner particle ray propagation on "\
        "buffer %i completed",
        getParentRayTracingStepUID(),
        ray_data.uid);
}

void ldplab::rtscpu::EikonalSolverRK4LinearIndexGradientPolarization::
rayPropagation(
    size_t particle_index,
    Ray& ray,
    Vec3& inter_point,
    Vec3& inter_normal,
    default_factories::InitialStageHomogenousPolarizedLightBoundingSphereProjectionFactory::Polarization& stokes,
    const std::shared_ptr<IGenericGeometry>& particle_geometry,
    const std::shared_ptr<IParticleMaterial>& particle_material,
    const Vec3& particle_center_of_mass,
    OutputBuffer& output) const
{
    bool intersected = false;
    bool is_inside = false;
    Arg x{
        ray.direction * particle_material->indexOfRefraction(ray.origin),
        ray.origin,
        convertStokesParameterToPolarizationVector(stokes, ray.direction) };
    Arg x_new{ {0,0,0}, {0,0,0}, {0,0,0} };

    while (!intersected)
    {
        rk4(particle_material, x, m_parameters.step_size, x_new);
        intersected = particle_geometry->intersectSegment(
            x.r,
            x_new.r,
            inter_point,
            inter_normal,
            is_inside);
        if (intersected || !is_inside)
        {
            if (!intersected)
            {
                // The following check is neccessary to test if the ray hits in
                // an extreme point (volume so small that it lies within the 
                // epsilon region). If that is the case, we assume the ray 
                // tunnels through the particle.
                bool intersect_outside = false;
                Vec3 t_ip, t_in;
                intersected = particle_geometry->intersectRay(
                    ray,
                    t_ip,
                    t_in,
                    intersect_outside);
                if (!intersected || intersect_outside)
                {
                    // We have found a case, where the ray tunnels through the
                    // Particle. We use the original ray and invalidate the 
                    // surface normal.
                    inter_point = ray.origin;
                    inter_normal = Vec3(0, 0, 0);
                }
                else
                {
                    // We have found a case, where the initial ray is bend 
                    // out of the particle in the initial step due to the 
                    // particle material gradient. In this case we assume that 
                    // it hits in the original intersection point.
                    // We use the previous normal and have to flip it, to 
                    // ensure that we have correct behaviour in the interaction
                    // stage.
                    // To receive the previous normal, we simply perform the
                    // Intersection test again, but this time we reverse the
                    // segment directions. Then we flip the normal.
                    particle_geometry->intersectSegment(
                        x_new.r,
                        x.r,
                        inter_point,
                        inter_normal,
                        is_inside);
                    inter_point = ray.origin;
                    inter_normal = -inter_normal;
                    ray.direction = glm::normalize(x.q);

                    stokes.direction = glm::normalize(glm::cross(ray.direction, ray.direction + Vec3{ 1,1,1 }));
                    stokes.stokes_parameter = convertPolarizationVectorToStokesParameter(
                        x.u, stokes, ray.direction);
                    stokes.stokes_parameter.x = ray.intensity;
                }
            }
            else
            {
                ray.direction = glm::normalize(x.q);
                const double nx = particle_material->indexOfRefraction(x.r);
                const double ny = particle_material->indexOfRefraction(inter_point);
                const Vec3 delta_momentum = (nx - ny) * ray.direction;
                const Vec3 r = inter_point - particle_center_of_mass;
                output.force[particle_index] += ray.intensity *
                    delta_momentum;
                output.torque[particle_index] += ray.intensity *
                    glm::cross(r, delta_momentum);


                stokes.direction = glm::normalize(glm::cross(ray.direction, ray.direction + Vec3{ 1,1,1 }));
                stokes.stokes_parameter = convertPolarizationVectorToStokesParameter(
                    x.u, stokes, ray.direction);
                stokes.stokes_parameter.x = ray.intensity;
            }
            return;
        }
        else
        {
            const double nx = particle_material->indexOfRefraction(x.r);
            const double ny = particle_material->indexOfRefraction(x_new.r);
            const Vec3 t_old_direction = glm::normalize(x.q);
            const Vec3 t_new_direction = glm::normalize(x_new.q);
            const Vec3 delta_momentum = nx * t_old_direction -
                ny * t_new_direction;
            const Vec3 r = x_new.r - particle_center_of_mass;
            output.force[particle_index] += ray.intensity *
                delta_momentum;
            output.torque[particle_index] += ray.intensity *
                glm::cross(r, delta_momentum);
            x = x_new;
        }
    }
}

void ldplab::rtscpu::EikonalSolverRK4LinearIndexGradientPolarization::rk4(
    const std::shared_ptr<IParticleMaterial>& particle_material,
    const Arg& x,
    const double h,
    Arg& x_new) const
{
    const ParticleMaterialLinearOneDirectional* raw_material =
        static_cast<ParticleMaterialLinearOneDirectional*>(particle_material.get());

   auto D = [raw_material](const Vec3& pos) { return raw_material->indexOfRefraction(pos) * raw_material->direction_times_gradient; };
  //const Vec3 a = h * D(x.r);
  //const Vec3 b = h * D(x.r + (h/2.0)*x.q + (h/8.0)*a);
  //const Vec3 c = h * D(x.r + h*x.q + (h/2.0)*b);
  //x_new.r = x.r + h * (x.q + (a + 2.0 * b) / 6.0);
  //x_new.q = x.q + (a + 4.0 * b + c) / 6.0;
  //
  //
  //const Vec3 dn = D(x.r) / raw_material->indexOfRefraction(x.r);
  //const std::complex<double> dot = (std::conj(x.u.x) * dn.x + std::conj(x.u.y) * dn.y + std::conj(x.u.z) * dn.z)*h;
  //const Vec3c k1 = { x.q.x * dot, x.q.y * dot, x.q.z * dot};
  //
  //const Vec3 x1 = x.r + x.q * (h*0.5);
  //const Vec3c q1 = x.q + a*0.5;
  //const Vec3c u1 = x.u + k1 * static_cast<std::complex<double>>(h / 2);
  //const Vec3 dn = D(x1) / raw_material->indexOfRefraction(x1);
  //const std::complex<double> dot2 = (std::conj(u1.x) * dn.x + std::conj(u1.y) * dn.y + std::conj(u1.z) * dn.z)*h;
  //const Vec3c k2 = { q1.x * dot2, q1.y * dot2, q1.z * dot2 };
  //
  //const Vec3 x2 = x.r + x1*2;
  //const Vec3c q2 = x.q + a * 0.5;
  //const Vec3c u2 = x.u + k1 * static_cast<std::complex<double>>(h / 2);
  //const Vec3 dn = D(x1) / raw_material->indexOfRefraction(x1);
  //const std::complex<double> dot2 = (std::conj(u1.x) * dn.x + std::conj(u1.y) * dn.y + std::conj(u1.z) * dn.z) * h;
  //const Vec3c k2 = { q1.x * dot2, q1.y * dot2, q1.z * dot2 };
  //
  //const Vec3c k3 = f(x + k2 * (h / 2), h) * h;
  //const Vec3c k4 = f(x + k3 * h, h) * h;
    auto f = [raw_material, D](const Arg& x, const double h)
    {
        const Vec3 r = x.q;
        const Vec3 q = D(x.r);
        const Vec3 dn = D(x.r) / raw_material->indexOfRefraction(x.r);
        const std::complex<double> a = -x.u.x * dn.x - x.u.y * dn.y - x.u.z * dn.z;
        const Vec3c u = static_cast<Vec3c>(x.q) * a;
        return Arg{q, r, u};
    };
    
    const Arg k1 = f(x, h)*h;
    const Arg k2 = f(x + k1*(h/2), h) * h;
    const Arg k3 = f(x + k2 * (h / 2), h) * h;
    const Arg k4 = f(x + k3 * h, h) * h;
    x_new = x + (k1 + k2 * 2 + k3 * 2 + k4) / 6;
    x_new.u = normalize(x_new.u);
}

ldplab::Vec3c ldplab::rtscpu::EikonalSolverRK4LinearIndexGradientPolarization::
    convertStokesParameterToPolarizationVector(
        const default_factories::InitialStageHomogenousPolarizedLightBoundingSphereProjectionFactory::Polarization& stokes,
        const Vec3& ray_direction)
{
    using namespace std::complex_literals;
    const double I = std::sqrt(stokes.stokes_parameter.y * stokes.stokes_parameter.y + 
        stokes.stokes_parameter.z * stokes.stokes_parameter.z +
        stokes.stokes_parameter.a * stokes.stokes_parameter.a);

    const double Ex = std::sqrt(I + stokes.stokes_parameter.y)/4.0;
    const double Ey = std::sqrt(I - stokes.stokes_parameter.y)/4.0;
    double eps;
    if (Ex == 0 || Ey == 0)
        eps = 0;
    else if (std::abs(stokes.stokes_parameter.z) > 1e-9)
        eps = std::asin(stokes.stokes_parameter.a / (2.0 * Ex * Ey));
    else if (std::abs(stokes.stokes_parameter.a) > 1e-9)
        eps = std::acos(stokes.stokes_parameter.z / (2.0 * Ex * Ey));
    else
        eps = std::atan(stokes.stokes_parameter.a/stokes.stokes_parameter.z);
    const Vec3c i = glm::normalize(glm::cross(stokes.direction,ray_direction));
    const Vec3c j = stokes.direction;
    const Vec3c u = static_cast<std::complex<double>>(Ex)*i + (Ey*std::exp(1i * eps))*j;
    return normalize(u);
}

ldplab::Vec4 
    ldplab::rtscpu::EikonalSolverRK4LinearIndexGradientPolarization::
    convertPolarizationVectorToStokesParameter(
        const Vec3c& polarization,
        const default_factories::InitialStageHomogenousPolarizedLightBoundingSphereProjectionFactory::Polarization& stokes,
        const Vec3& ray_direction)
{
    const double I = std::sqrt(stokes.stokes_parameter.y * stokes.stokes_parameter.y +
        stokes.stokes_parameter.z * stokes.stokes_parameter.z +
        stokes.stokes_parameter.a * stokes.stokes_parameter.a);

    const Vec3c i = glm::normalize(glm::cross(stokes.direction, ray_direction));
    const Vec3c j = stokes.direction;
    const Vec3c pol =  static_cast<std::complex<double>>(I) * polarization;
    const std::complex<double> cEx = dot(pol, i);
    const std::complex<double> cEy = dot(pol, j);
    const double eps = std::arg(cEy) - std::arg(cEy);
    const double Ex = std::abs(cEx);
    const double Ey = std::abs(cEy);

    return { stokes.stokes_parameter.x, Ex * Ex - Ey * Ey, 2 * Ex * Ey * cos(eps),2 * Ex * Ey * sin(eps) };
}