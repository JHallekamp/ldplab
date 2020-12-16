#include "RayParticleInteractionStage.hpp"

#include "Context.hpp"
#include "Data.hpp"

#include "../../SimulationState.hpp"
#include "../../Log.hpp"
#include "../../Utils/Assert.hpp"

#include "../../../../libs/glm/glm.hpp"

ldplab::rtscpu::UnpolirzedLight1DLinearIndexGradientInteraction::
    UnpolirzedLight1DLinearIndexGradientInteraction(
        std::shared_ptr<Context> context)
    :
    m_context{ context }
{
}

void ldplab::rtscpu::UnpolirzedLight1DLinearIndexGradientInteraction::execute(
    const SimulationState* state, 
    const size_t particle, 
    const IntersectionBuffer& intersection, 
    RayBuffer& input_outer_rays, 
    RayBuffer& inner_rays)
{
    ParticleMaterialLinearOneDirectional* material =
        (ParticleMaterialLinearOneDirectional*) m_context->
            particles[particle].material.get();

    input_outer_rays.branching_depth++;
    inner_rays.branching_depth = input_outer_rays.branching_depth;

    for (int i = 0; i < input_outer_rays.size; i++)
    {
        Ray& ray = input_outer_rays.data[i];
        Ray& refracted_ray = inner_rays.data[i];
        Vec3& inter_point = intersection.point[i];
        Vec3& inter_normal = intersection.normal[i];
        
        if (ray.intensity <= 0)
            continue;

        double nr = material->indexOfRefraction(inter_point);
        
        double cos_a = glm::dot(ray.direction, inter_normal);

        if (1 - nr * (1 - cos_a * cos_a) >= 0)
        {
            double cos_b = std::sqrt(1 - nr * (1 - cos_a * cos_a));
            double R = reflectance(cos_a, cos_b, nr);
            // refracted ray

            refracted_ray.intensity = ray.intensity * R;
            if (refracted_ray.intensity > m_context->intensity_cutoff)
            {
                refracted_ray.origin = inter_point;
                refracted_ray.direction = nr * ray.direction +
                    inter_normal * (-cos_b + nr * cos_a);
                refracted_ray.intensity = ray.intensity * R;

                inner_rays.num_rays++;
            }
            else
            {
                refracted_ray.intensity = -1;
            }
            // reflected ray
            ray.intensity = ray.intensity * (1 - R);
            if (ray.intensity > m_context->intensity_cutoff)
            {
                ray.origin = inter_point;
                ray.direction = ray.direction - inter_normal * 2.0 * cos_a;
            }
            else
            {
                ray.intensity = -1;
                input_outer_rays.num_rays--;
            }
        }
        else
        {
            // total reflected ray
            ray.origin = inter_point;
            ray.direction = ray.direction - inter_normal * 2.0 * cos_a;
        }
    }
    
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