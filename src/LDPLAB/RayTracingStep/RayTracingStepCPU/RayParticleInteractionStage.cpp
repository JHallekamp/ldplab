#include "RayParticleInteractionStage.hpp"

#include "Context.hpp"
#include "Data.hpp"

#include "../../Log.hpp"

#include "../../../../libs/glm/glm.hpp"

ldplab::rtscpu::UnpolirzedLight1DLinearIndexGradientInteraction::
    UnpolirzedLight1DLinearIndexGradientInteraction(
        std::shared_ptr<Context> context)
    :
    m_context{ context }
{
    LDPLAB_LOG_INFO("RTSCPU context %i: "\
        "UnpolirzedLight1DLinearIndexGradientInteraction instance created",
        m_context->uid);
}

void ldplab::rtscpu::UnpolirzedLight1DLinearIndexGradientInteraction::execute(
    const IntersectionBuffer& intersection,
    const RayBuffer& rays,
    RayBuffer& reflected_rays,
    RayBuffer& refracted_rays)
{
    LDPLAB_LOG_TRACE("RTSCPU context %i: Execute ray particle interaction"\
        "on batch buffer %i",
        m_context->uid, rays.uid);

    reflected_rays.inner_particle_rays = rays.inner_particle_rays;
    refracted_rays.inner_particle_rays = !rays.inner_particle_rays;

    for (size_t i = 0; i < rays.size; i++)
    {
        if (rays.index_data[i] < 0 &&
            rays.index_data[i] >= m_context->particles.size())
            continue;

        ParticleMaterialLinearOneDirectional* material =
            (ParticleMaterialLinearOneDirectional*)m_context->
            particles[rays.index_data[i]].material.get();

        Ray& ray = rays.ray_data[i];
        Ray& reflected_ray = reflected_rays.ray_data[i];
        Ray& refracted_ray = refracted_rays.ray_data[i];
        Vec3& inter_point = intersection.point[i];
        Vec3& inter_normal = intersection.normal[i];

        double nr = material->indexOfRefraction(inter_point);
        double cos_a = glm::dot(ray.direction, inter_normal);

        if (1 - nr * (1 - cos_a * cos_a) >= 0)
        {
            double cos_b = std::sqrt(1 - nr * (1 - cos_a * cos_a));
            double R = reflectance(cos_a, cos_b, nr);

            // refracted ray
            refracted_ray.intensity = ray.intensity * (1 - R);
            if (refracted_ray.intensity > m_context->intensity_cutoff)
            {
                refracted_ray.origin = inter_point;
                refracted_ray.direction = nr * ray.direction +
                    inter_normal * (-cos_b + nr * cos_a);
                refracted_rays.active_rays++;
            }
            else
                refracted_rays.index_data[i] = -1;

            // reflected ray
            reflected_ray.intensity = ray.intensity * R;
            if (ray.intensity > m_context->intensity_cutoff)
            {
                reflected_ray.origin = inter_point;
                reflected_ray.direction = ray.direction - inter_normal * 2.0 * cos_a;
                reflected_rays.active_rays++;
            }
            else
                reflected_rays.index_data[i] = -1;
        }
        else
        {
            // total reflected ray
            reflected_ray.origin = inter_point;
            reflected_ray.direction = ray.direction - inter_normal * 2.0 * cos_a;
            reflected_ray.intensity = ray.intensity;
            reflected_rays.active_rays++;
        }
    }

    LDPLAB_LOG_TRACE("RTSCPU context %i: Ray particle interaction on batch "\
        "buffer %i executed, buffer %i now holds %i reflected rays, buffer "\
        "%i now holds %i refracted rays",
        m_context->uid, 
        rays.uid, 
        reflected_rays.uid, 
        reflected_rays, 
        refracted_rays.uid, 
        refracted_rays);
}

double ldplab::rtscpu::UnpolirzedLight1DLinearIndexGradientInteraction::
    reflectance(
    double cos_alpha, double cos_beta, double n_r)
{
    double cos2_a = cos_alpha * cos_alpha;
    double cos2_b = cos_beta * cos_beta;
    return (cos2_a - cos2_b) * (cos2_a - cos2_b) /
        ((cos2_a + cos2_b) + (n_r + 1 / n_r) * cos_alpha * cos_beta) /
        ((cos2_a + cos2_b) + (n_r + 1 / n_r) * cos_alpha * cos_beta);
}