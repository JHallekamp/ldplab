#include "RayParticleInteractionStage.hpp"

#include "Context.hpp"
#include "Data.hpp"

#include "../../../Log.hpp"

#include <glm/glm.hpp>

ldplab::rtsgpu_ogl::UnpolirzedLight1DLinearIndexGradientInteraction::
    UnpolirzedLight1DLinearIndexGradientInteraction(
        std::shared_ptr<Context> context)
    :
    m_context{ context }
{
    LDPLAB_LOG_INFO("RTSGPU (OpenGL) context %i: "\
        "UnpolirzedLight1DLinearIndexGradientInteraction instance created",
        m_context->uid);
}

void ldplab::rtsgpu_ogl::UnpolirzedLight1DLinearIndexGradientInteraction::execute(
    const IntersectionBuffer& intersection,
    const RayBuffer& rays,
    RayBuffer& reflected_rays,
    RayBuffer& refracted_rays,
    OutputBuffer& output)
{
    LDPLAB_LOG_TRACE("RTSGPU (OpenGL) context %i: Execute ray particle interaction "\
        "on batch buffer %i",
        m_context->uid, rays.uid);

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
        else if (rays.index_data[i] >= m_context->particles.size())
            continue;

        const ParticleMaterialLinearOneDirectional* material =
            (ParticleMaterialLinearOneDirectional*)m_context->
            particles[rays.index_data[i]].material.get();

        const Vec4& ray_origin = rays.ray_origin_data[i];
        const Vec4& ray_direction = rays.ray_direction_data[i];
        const double ray_intensity = rays.ray_intensity_data[i];

        Vec4& reflected_ray_origin = reflected_rays.ray_origin_data[i];
        Vec4& reflected_ray_direction = reflected_rays.ray_direction_data[i];
        double& reflected_ray_intensity = reflected_rays.ray_intensity_data[i];
        
        Vec4& refracted_ray_origin = refracted_rays.ray_origin_data[i];
        Vec4& refracted_ray_direction = refracted_rays.ray_direction_data[i];
        double& refracted_ray_intensity = refracted_rays.ray_intensity_data[i];
        
        const Vec4& inter_point = intersection.point_data[i];
        const Vec4& inter_normal = intersection.normal_data[i];

        double nr = m_context->parameters.medium_reflection_index /
            material->indexOfRefraction(inter_point);
        if (rays.inner_particle_rays)
            nr = 1.0 / nr;
        double cos_a = -glm::dot(ray_direction, inter_normal);

        if (1.0 - nr * nr * (1.0 - cos_a * cos_a) >= 0)
        {
            double cos_b = std::sqrt(1.0 - nr * nr * (1.0 - cos_a * cos_a));
            double R = reflectance(cos_a, cos_b, nr);

            // refracted ray
            refracted_ray_intensity = ray_intensity * (1.0 - R);
            if (refracted_ray_intensity >
                m_context->parameters.intensity_cutoff)
            {
                refracted_rays.index_data[i] = rays.index_data[i];
                refracted_rays.min_bounding_volume_distance_data[i] = 0.0;
                refracted_ray_origin = inter_point;
                refracted_ray_direction = nr * ray_direction +
                    inter_normal * (-cos_b + nr * cos_a);
                refracted_rays.active_rays++;

                output.force_data[rays.index_data[i]] += refracted_ray_intensity *
                    (ray_direction - refracted_ray_direction);
                output.torque_data[rays.index_data[i]] += refracted_ray_intensity *
                    Vec4(glm::cross(
                        m_context->particles[rays.index_data[i]].centre_of_mass,
                        Vec3(ray_direction - refracted_ray_direction)), 0);
            }
            else
            {
                refracted_rays.index_data[i] = -1;
                Vec4 delta_direction = (nr - 1) * ray_direction + inter_normal * 
                    (cos_b - nr * cos_a);
                output.force_data[rays.index_data[i]] += refracted_ray_intensity *
                    delta_direction;
                output.torque_data[rays.index_data[i]] += refracted_ray_intensity *
                    Vec4(glm::cross(
                        m_context->particles[rays.index_data[i]].centre_of_mass,
                        Vec3(delta_direction)), 0);
            }
            // reflected ray
            reflected_ray_intensity = ray_intensity * R;
            if (reflected_ray_intensity > m_context->parameters.intensity_cutoff)
            {
                reflected_rays.index_data[i] = rays.index_data[i];
                reflected_rays.min_bounding_volume_distance_data[i] = 0.0;
                reflected_ray_origin = inter_point;
                reflected_ray_direction =
                    ray_direction + inter_normal * 2.0 * cos_a;
                reflected_rays.active_rays++;

                output.force_data[rays.index_data[i]] += reflected_ray_intensity *
                    (ray_direction - reflected_ray_direction);
                output.torque_data[rays.index_data[i]] += reflected_ray_intensity *
                    Vec4(glm::cross(
                        m_context->particles[rays.index_data[i]].centre_of_mass,
                        Vec3(ray_direction - reflected_ray_direction)), 0);
            }
            else
            {
                reflected_rays.index_data[i] = -1;
                Vec4 delta_direction = inter_normal * (-2.0 * cos_a);
                output.force_data[rays.index_data[i]] += reflected_ray_intensity *
                    delta_direction;
                output.torque_data[rays.index_data[i]] += reflected_ray_intensity *
                    Vec4(glm::cross(
                        m_context->particles[rays.index_data[i]].centre_of_mass,
                        Vec3(delta_direction)), 0);
            }
        }
        else
        {
            refracted_rays.index_data[i] = -1;
            // total reflected ray
            reflected_rays.index_data[i] = rays.index_data[i];
            reflected_rays.min_bounding_volume_distance_data[i] = 0.0;
            reflected_ray_origin = inter_point;
            reflected_ray_direction = ray_direction + inter_normal * 2.0 * cos_a;
            reflected_ray_intensity = ray_intensity;
            reflected_rays.active_rays++;

            output.force_data[rays.index_data[i]] += reflected_ray_intensity *
                (ray_direction - reflected_ray_direction);
            output.torque_data[rays.index_data[i]] += reflected_ray_intensity *
                Vec4(glm::cross(
                    m_context->particles[rays.index_data[i]].centre_of_mass,
                    Vec3(ray_direction - reflected_ray_direction)), 0);
        }
    }

    LDPLAB_LOG_TRACE("RTSGPU (OpenGL) context %i: Ray particle interaction on batch "\
        "buffer %i executed, buffer %i now holds %i reflected rays, buffer "\
        "%i now holds %i refracted rays",
        m_context->uid, 
        rays.uid, 
        reflected_rays.uid, 
        reflected_rays.active_rays, 
        refracted_rays.uid, 
        refracted_rays.active_rays);
}

double ldplab::rtsgpu_ogl::UnpolirzedLight1DLinearIndexGradientInteraction::
    reflectance(
    double cos_alpha, double cos_beta, double n_r)
{
    double cos2_a = cos_alpha * cos_alpha;
    double cos2_b = cos_beta * cos_beta;
    return (cos2_a - cos2_b) * (cos2_a - cos2_b) /
        (((cos2_a + cos2_b) + (n_r + 1 / n_r) * cos_alpha * cos_beta) *
         ((cos2_a + cos2_b) + (n_r + 1 / n_r) * cos_alpha * cos_beta));
}