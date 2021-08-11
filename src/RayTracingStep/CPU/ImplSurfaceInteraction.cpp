#include "ImplSurfaceInteraction.hpp"

#include "../../Utils/Log.hpp"

void ldplab::rtscpu::SurfaceInteraction::execute(
	const RayBuffer& input_ray_data, 
    const IntersectionBuffer& intersection_data, 
	RayBuffer& output_ray_data, 
    OutputBuffer& output_data, 
    double intensity_cutoff, 
	double medium_reflection_index,
    const std::vector<std::shared_ptr<IParticleMaterial>>& material_data,
    const std::vector<Vec3>& center_of_mass,
    InteractionPassType pass_type, 
    size_t pass_no, 
	const SimulationParameter& simulation_parameter, 
	void* stage_dependent_data)
{
    LDPLAB_LOG_TRACE("RTSCPU %i: Execute ray particle interaction "\
        "on batch buffer %i",
        getParentRayTracingStepUID(), input_ray_data.uid);

    if(pass_type == InteractionPassType::reflection)
        output_ray_data.inner_particle_rays = input_ray_data.inner_particle_rays;
    else
        output_ray_data.inner_particle_rays = !input_ray_data.inner_particle_rays;

    output_ray_data.active_rays = 0;

    for (size_t i = 0; i < input_ray_data.size; i++)
    {
        const int32_t particle_id = input_ray_data.index_data[i];
        output_ray_data.index_data[i] = -1;

        if (particle_id < 0 || particle_id >= simulation_parameter.num_particles)
        {
            output_ray_data.index_data[i] = -1;
            continue;
        }

        // Check if the intersection normal is 0, in which case the ray will 
        // be written into the transmission buffer without changes. This is
        // done because in the inner particle propagation stage, there can
        // occur situations where a ray is tangent to the geometry, in which
        // case it will intersect it, but no inner particle propagation will
        // actually occur. To ensure correct behavior in such case, the 
        // intersection normal is set to 0.
        if (intersection_data.normal[i] == Vec3(0, 0, 0))
        {
            if (pass_type == InteractionPassType::transmission)
            {
                output_ray_data.index_data[i] = particle_id;
                output_ray_data.ray_data[i] = input_ray_data.ray_data[i];
                output_ray_data.min_bounding_volume_distance_data[i] = 0.0;
                output_ray_data.active_rays++;
            }
            continue;
        }

        const Ray& ray = input_ray_data.ray_data[i];
        Ray& output_ray = output_ray_data.ray_data[i];
        const Vec3& inter_point = intersection_data.point[i];
        const Vec3& inter_normal = intersection_data.normal[i];

        const double nx = input_ray_data.inner_particle_rays ?
            material_data[particle_id]->indexOfRefraction(inter_point) :
            medium_reflection_index;
        const double ny = input_ray_data.inner_particle_rays ?
            medium_reflection_index :
            material_data[particle_id]->indexOfRefraction(inter_point);
        const double nr = nx / ny;
        const double cos_a = -glm::dot(ray.direction, inter_normal);

        if (1.0 - nr * nr * (1.0 - cos_a * cos_a) >= 0)
        {
            const bool reflection_pass = 
                (pass_type == InteractionPassType::reflection);
            const double cos_b = std::sqrt(1.0 - nr * nr * (1.0 - cos_a * cos_a));
            const double R = reflectance(cos_a, cos_b, nr);
            const Vec3 r = inter_point - center_of_mass[particle_id];
            Vec3 delta_momentum;
            output_ray.intensity = reflection_pass ?
                ray.intensity * R :
                ray.intensity * (1.0 - R);
            if (output_ray.intensity > intensity_cutoff)
            {
                ++output_ray_data.active_rays;
                output_ray_data.index_data[i] = particle_id;
                output_ray_data.min_bounding_volume_distance_data[i] = 0.0;
                output_ray.origin = inter_point;
                output_ray.direction = reflection_pass ?
                    ray.direction + inter_normal * 2.0 * cos_a :
                    inter_normal * (-cos_b + nr * cos_a);
                delta_momentum = reflection_pass ?
                    nx * (ray.direction - output_ray.direction) :
                    nx * ray.direction - ny * output_ray.direction;
            }
            else
            {
                output_ray_data.index_data[i] = -1;
                delta_momentum = reflection_pass ?
                    inter_normal * (nx * -2.0 * cos_a) :
                    inter_normal * (ny * cos_b - nx * cos_a);
            }
            output_data.force[particle_id] += output_ray.intensity *
                delta_momentum;
            output_data.torque[particle_id] += output_ray.intensity *
                glm::cross(r, delta_momentum);
        }
        else if(pass_type == InteractionPassType::reflection) // total reflected ray
        {
            output_ray_data.index_data[i] = particle_id;
            output_ray_data.min_bounding_volume_distance_data[i] = 0.0;
            output_ray.origin = inter_point;
            output_ray.direction = ray.direction + inter_normal * 2.0 * cos_a;
            output_ray.intensity = ray.intensity;
            output_ray_data.active_rays++;
            const Vec3 delta_momentum = nx *
                (ray.direction - output_ray.direction);
            const Vec3 r = inter_point - center_of_mass[particle_id];
            output_data.force[particle_id] += output_ray.intensity *
                delta_momentum;
            output_data.torque[particle_id] += output_ray.intensity *
                glm::cross(r, delta_momentum);
        }
    }

    LDPLAB_LOG_TRACE("RTSCPU %i: Ray particle interaction on batch "\
        "buffer %i executed, buffer %i now holds %i reflected rays, buffer "\
        "%i now holds %i refracted rays",
        getParentRayTracingStepUID(),
        input_ray_data.uid,
        output_ray_data.uid,
        output_ray_data.active_rays,
        output_ray_data.uid,
        output_ray_data.active_rays);
}

double ldplab::rtscpu::SurfaceInteraction::reflectance(
    double cos_alpha, double cos_beta, double n_r)
{
    double cos2_a = cos_alpha * cos_alpha;
    double cos2_b = cos_beta * cos_beta;
    return (cos2_a - cos2_b) * (cos2_a - cos2_b) /
        (((cos2_a + cos2_b) + (n_r + 1 / n_r) * cos_alpha * cos_beta) *
            ((cos2_a + cos2_b) + (n_r + 1 / n_r) * cos_alpha * cos_beta));
}