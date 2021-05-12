#include "RayParticleInteractionStage.hpp"

#include "Context.hpp"
#include "Data.hpp"

#include "../../Utils/Log.hpp"

#include <glm/glm.hpp>


ldplab::rtscpu::UnpolirzedLight1DLinearIndexGradientInteraction::
    UnpolirzedLight1DLinearIndexGradientInteraction(
        Context& context)
    :
    m_context{ context }
{
    LDPLAB_LOG_INFO("RTSCPU context %i: "\
        "UnpolirzedLight1DLinearIndexGradientInteraction instance created",
        m_context.uid);
}

void ldplab::rtscpu::UnpolirzedLight1DLinearIndexGradientInteraction::execute(
    const IntersectionBuffer& intersection,
    const RayBuffer& rays,
    RayBuffer& reflected_rays,
    RayBuffer& refracted_rays,
    OutputBuffer& output)
{
    LDPLAB_LOG_TRACE("RTSCPU context %i: Execute ray particle interaction "\
        "on batch buffer %i",
        m_context.uid, rays.uid);

    reflected_rays.inner_particle_rays = rays.inner_particle_rays;
    refracted_rays.inner_particle_rays = !rays.inner_particle_rays;

    reflected_rays.active_rays = 0;
    refracted_rays.active_rays = 0;

    for (size_t i = 0; i < rays.size; i++)
    {
        if (rays.index_data[i] < 0)
        {
            reflected_rays.index_data[i] = -1;
            refracted_rays.index_data[i] = -1;
            continue;
        }
        else if (rays.index_data[i] >= m_context.particles.size())
            continue;

        // Check if the intersection normal is 0, in which case the ray will 
        // be written into the transmission buffer without changes. This is
        // done because in the inner particle propagation stage, there can
        // occur situations where a ray is tangent to the geometry, in which
        // case it will intersect it, but no inner particle propagation will
        // actually occur. To ensure correct behavior in such case, the 
        // intersection normal is set to 0.
        if (intersection.normal[i] == Vec3(0, 0, 0))
        {
            reflected_rays.index_data[i] = -1;
            refracted_rays.index_data[i] = rays.index_data[i];
            refracted_rays.ray_data[i] = rays.ray_data[i];
            refracted_rays.min_bounding_volume_distance_data[i] = 0.0;
            refracted_rays.active_rays++;
            continue;
        }

        const ParticleMaterialLinearOneDirectional* material =
            (ParticleMaterialLinearOneDirectional*)m_context.
            particles[rays.index_data[i]].material.get();

        const Ray& ray = rays.ray_data[i];
        Ray& reflected_ray = reflected_rays.ray_data[i];
        Ray& refracted_ray = refracted_rays.ray_data[i];
        const Vec3& inter_point = intersection.point[i];
        const Vec3& inter_normal = intersection.normal[i];

        real_t nr,nx,ny;
        if (rays.inner_particle_rays)
        {
            nx = material->indexOfRefraction(inter_point);
            ny = m_context.parameters.medium_reflection_index;
            nr = nx / ny;
        }
        else
        {
            nx = m_context.parameters.medium_reflection_index;
            ny = material->indexOfRefraction(inter_point); 
            nr = nx / ny;
        }

        real_t cos_a = -glm::dot(ray.direction, inter_normal);

        if (1.0 - nr * nr * (1.0 - cos_a * cos_a) >= 0)
        {
            real_t cos_b = std::sqrt(1.0 - nr * nr * (1.0 - cos_a * cos_a));
            real_t R = reflectance(cos_a, cos_b, nr);

            // refracted ray
            refracted_ray.intensity = ray.intensity * (1.0 - R);
            if (refracted_ray.intensity > 
                m_context.parameters.intensity_cutoff)
            {
                refracted_rays.index_data[i] = rays.index_data[i];
                refracted_rays.min_bounding_volume_distance_data[i] = 0.0;
                refracted_ray.origin = inter_point;
                refracted_ray.direction = nr * ray.direction +
                    inter_normal * (-cos_b + nr * cos_a);
                refracted_rays.active_rays++;
                
                const Vec3 delta_momentum = nx * ray.direction - 
                    ny * refracted_ray.direction;
                const Vec3 r = inter_point -
                    m_context.particles[rays.index_data[i]].centre_of_mass;
                output.force[rays.index_data[i]] += refracted_ray.intensity * 
                    delta_momentum;
                output.torque[rays.index_data[i]] += refracted_ray.intensity *
                    glm::cross(r, delta_momentum);
            }
            else
            {
                refracted_rays.index_data[i] = -1;
                const Vec3 delta_momentum = inter_normal * (ny * cos_b - nx * cos_a);
                const Vec3 r = inter_point -
                    m_context.particles[rays.index_data[i]].centre_of_mass;
                output.force[rays.index_data[i]] += refracted_ray.intensity *
                    delta_momentum;
                output.torque[rays.index_data[i]] += refracted_ray.intensity *
                    glm::cross(r, delta_momentum);
            }
            // reflected ray
            reflected_ray.intensity = ray.intensity * R;
            if (reflected_ray.intensity > m_context.parameters.intensity_cutoff)
            {
                reflected_rays.index_data[i] = rays.index_data[i];
                reflected_rays.min_bounding_volume_distance_data[i] = 0.0;
                reflected_ray.origin = inter_point;
                reflected_ray.direction = 
                    ray.direction + inter_normal * cos_a * static_cast<real_t>(2);
                reflected_rays.active_rays++;

                const Vec3 delta_momentum = nx * 
                    (ray.direction - reflected_ray.direction);
                const Vec3 r = inter_point -
                    m_context.particles[rays.index_data[i]].centre_of_mass;
                output.force[rays.index_data[i]] += reflected_ray.intensity *
                    delta_momentum;
                output.torque[rays.index_data[i]] += reflected_ray.intensity *
                    glm::cross(r, delta_momentum);
            }
            else
            {
                reflected_rays.index_data[i] = -1;
                const Vec3 delta_momentum = inter_normal * (nx * cos_a * -static_cast<real_t>(2));
                const Vec3 r = inter_point -
                    m_context.particles[rays.index_data[i]].centre_of_mass;
                output.force[rays.index_data[i]] += reflected_ray.intensity *
                    delta_momentum;
                output.torque[rays.index_data[i]] += reflected_ray.intensity *
                    glm::cross(r, delta_momentum);
            }
        }
        else
        {
            refracted_rays.index_data[i] = -1;
            // total reflected ray
            reflected_rays.index_data[i] = rays.index_data[i];
            reflected_rays.min_bounding_volume_distance_data[i] = 0.0;
            reflected_ray.origin = inter_point;
            reflected_ray.direction = ray.direction + 
                inter_normal * cos_a * static_cast<real_t>(2);
            reflected_ray.intensity = ray.intensity;
            reflected_rays.active_rays++;

            const Vec3 delta_momentum = nx * 
                (ray.direction - reflected_ray.direction);
            const Vec3 r = inter_point - 
                m_context.particles[rays.index_data[i]].centre_of_mass;
            output.force[rays.index_data[i]] += reflected_ray.intensity *
                delta_momentum;
            output.torque[rays.index_data[i]] += reflected_ray.intensity *
                glm::cross(r, delta_momentum);
        }
    }

    LDPLAB_LOG_TRACE("RTSCPU context %i: Ray particle interaction on batch "\
        "buffer %i executed, buffer %i now holds %i reflected rays, buffer "\
        "%i now holds %i refracted rays",
        m_context.uid, 
        rays.uid, 
        reflected_rays.uid, 
        reflected_rays.active_rays, 
        refracted_rays.uid, 
        refracted_rays.active_rays);
}

ldplab::real_t ldplab::rtscpu::UnpolirzedLight1DLinearIndexGradientInteraction::
    reflectance(
    real_t cos_alpha, real_t cos_beta, real_t n_r)
{
    real_t cos2_a = cos_alpha * cos_alpha;
    real_t cos2_b = cos_beta * cos_beta;
    return (cos2_a - cos2_b) * (cos2_a - cos2_b) /
        (((cos2_a + cos2_b) + (n_r + 1 / n_r) * cos_alpha * cos_beta) * 
            ((cos2_a + cos2_b) + (n_r + 1 / n_r) * cos_alpha * cos_beta));
}