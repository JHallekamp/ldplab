#include "ImplSurfaceInteraction.hpp"

#include "../../Utils/Log.hpp"
#include <glm/glm.hpp>

#include <assert.h>

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
            continue;

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
                    nr * ray.direction + inter_normal * (-cos_b + nr * cos_a);
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
            ++output_ray_data.active_rays;
            output_ray_data.index_data[i] = particle_id;
            output_ray_data.min_bounding_volume_distance_data[i] = 0.0;
            output_ray.origin = inter_point;
            output_ray.direction = ray.direction + inter_normal * 2.0 * cos_a;
            output_ray.intensity = ray.intensity;
            const Vec3 delta_momentum = nx *
                (ray.direction - output_ray.direction);
            const Vec3 r = inter_point - center_of_mass[particle_id];
            output_data.force[particle_id] += output_ray.intensity *
                delta_momentum;
            output_data.torque[particle_id] += output_ray.intensity *
                glm::cross(r, delta_momentum);
        }
    }

    LDPLAB_LOG_TRACE("RTSCPU %i: Ray surface interaction on batch "\
        "buffer %i executed, buffer %i now holds %i %s rays",
        getParentRayTracingStepUID(),
        input_ray_data.uid,
        output_ray_data.uid,
        output_ray_data.active_rays,
        (pass_type == InteractionPassType::reflection ?
            "reflected" :
            "transmitted"));
}

double ldplab::rtscpu::SurfaceInteraction::reflectance(
    double cos_alpha, double cos_beta, double n_r) const
{
    const double cos2_a = cos_alpha * cos_alpha;
    const double cos2_b = cos_beta * cos_beta;
    return (cos2_a - cos2_b) * (cos2_a - cos2_b) /
        (((cos2_a + cos2_b) + (n_r + 1 / n_r) * cos_alpha * cos_beta) *
            ((cos2_a + cos2_b) + (n_r + 1 / n_r) * cos_alpha * cos_beta));
}

void ldplab::rtscpu::SurfaceInteractionPolarizedLight::execute(
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

    if (pass_type == InteractionPassType::reflection)
        output_ray_data.inner_particle_rays = input_ray_data.inner_particle_rays;
    else
        output_ray_data.inner_particle_rays = !input_ray_data.inner_particle_rays;

    output_ray_data.active_rays = 0;

    const auto& input_polarization = ((default_factories::InitialStageHomogenousPolarizedLightBoundingSphereProjectionFactory::PolarizationData*)
        stage_dependent_data)->polarization_buffers[input_ray_data.depth];
    auto output_polarization = ((default_factories::InitialStageHomogenousPolarizedLightBoundingSphereProjectionFactory::PolarizationData*)
        stage_dependent_data)->polarization_buffers[output_ray_data.depth];

    for (size_t i = 0; i < input_ray_data.size; i++)
    {
        const int32_t particle_id = input_ray_data.index_data[i];
        output_ray_data.index_data[i] = -1;

        if (particle_id < 0 || particle_id >= simulation_parameter.num_particles)
            continue;

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
        const auto& polarization = input_polarization.polarization_data[i];

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
            if (cos_a >= 1.0 - 1e-09)
                output_polarization.polarization_data[i] = polarization;
            else
                output_polarization.polarization_data[i] = rotatePolarizaitonInPlane(
                    polarization, inter_normal, ray.direction);

            output_polarization.polarization_data[i].stokes_parameter = reflection_pass ?
                reflactedPolarization(output_polarization.polarization_data[i].stokes_parameter, cos_a, cos_b, nr) :
                transmittedPolarization(output_polarization.polarization_data[i].stokes_parameter, cos_a, cos_b, nr);

            
            const Vec3 r = inter_point - center_of_mass[particle_id];
            Vec3 delta_momentum;
            output_ray.intensity =
                output_polarization.polarization_data[i].stokes_parameter.x;
            
            if (output_ray.intensity > intensity_cutoff)
            {
                ++output_ray_data.active_rays;
                output_ray_data.index_data[i] = particle_id;
                output_ray_data.min_bounding_volume_distance_data[i] = 0.0;
                output_ray.origin = inter_point;
                output_ray.direction = reflection_pass ?
                    ray.direction + inter_normal * 2.0 * cos_a :
                    nr * ray.direction + inter_normal * (-cos_b + nr * cos_a);
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
        else if (pass_type == InteractionPassType::reflection) // total reflected ray
        {
            ++output_ray_data.active_rays;
            output_polarization.polarization_data[i] = rotatePolarizaitonInPlane(
                polarization, inter_normal, ray.direction);
            reflactedPolarization(output_polarization.polarization_data[i].stokes_parameter, cos_a, 0, nr);
            output_ray_data.index_data[i] = particle_id;
            output_ray_data.min_bounding_volume_distance_data[i] = 0.0;
            output_ray.origin = inter_point;
            output_ray.direction = ray.direction + inter_normal * 2.0 * cos_a;
            output_ray.intensity = ray.intensity;
            const Vec3 delta_momentum = nx *
                (ray.direction - output_ray.direction);
            const Vec3 r = inter_point - center_of_mass[particle_id];
            output_data.force[particle_id] += output_ray.intensity *
                delta_momentum;
            output_data.torque[particle_id] += output_ray.intensity *
                glm::cross(r, delta_momentum);
        }
    }

    LDPLAB_LOG_TRACE("RTSCPU %i: Ray surface interaction on batch "\
        "buffer %i executed, buffer %i now holds %i %s rays",
        getParentRayTracingStepUID(),
        input_ray_data.uid,
        output_ray_data.uid,
        output_ray_data.active_rays,
        (pass_type == InteractionPassType::reflection ?
            "reflected" :
            "transmitted"));
}

double ldplab::rtscpu::SurfaceInteractionPolarizedLight::rp(
    const double cos_a, const double cos_b, double n_r) const
{
    return (cos_a - n_r * cos_b) / (cos_a + n_r * cos_b);
}

double ldplab::rtscpu::SurfaceInteractionPolarizedLight::rs(
    const double cos_a, const double cos_b, double n_r) const
{
    return (n_r * cos_a - cos_b) / (n_r * cos_a + cos_b);
}

double ldplab::rtscpu::SurfaceInteractionPolarizedLight::tp(
    const double cos_a, const double cos_b, double n_r) const
{
    return 2*n_r*cos_a / (cos_a + n_r *cos_b);
}

double ldplab::rtscpu::SurfaceInteractionPolarizedLight::ts(
    const double cos_a, const double cos_b, double n_r) const
{
    return 2 * n_r * cos_a / (n_r * cos_a + cos_b);
}

ldplab::rtscpu::default_factories::InitialStageHomogenousPolarizedLightBoundingSphereProjectionFactory::Polarization ldplab::rtscpu::SurfaceInteractionPolarizedLight::rotatePolarizaitonInPlane(
    const default_factories::InitialStageHomogenousPolarizedLightBoundingSphereProjectionFactory::Polarization& polarization, 
    const Vec3& normal, 
    const Vec3& light_direction) const
{
    const bool positive_direction = glm::dot(polarization.direction, normal) > 0;

    const Vec3 j = positive_direction ? glm::normalize(glm::cross(light_direction, normal)) : 
        glm::normalize(glm::cross(normal, light_direction));
    double cos_a = glm::dot(j,polarization.direction);
    if (cos_a > 1)
        cos_a = 1.0;
    else if (cos_a < -1.0)
        cos_a = -1.0;
    const double cos_2a = 2 * cos_a * cos_a - 1;
    const double sin_2a = cos_a > 0 ? std::sqrt(1 - cos_2a * cos_2a) : -std::sqrt(1 - cos_2a * cos_2a);
    
    const Vec4 S = {
        polarization.stokes_parameter.x,
        polarization.stokes_parameter.y * cos_2a + polarization.stokes_parameter.z * sin_2a,
        polarization.stokes_parameter.z * cos_2a - polarization.stokes_parameter.y * sin_2a,
        polarization.stokes_parameter.a};

    return { S, j, true };
}

ldplab::Vec4 ldplab::rtscpu::SurfaceInteractionPolarizedLight::reflactedPolarization(
    const Vec4& S,
    const double cos_a,
    const double cos_b,
    const double n_r) const
{
    Vec4 S_out;
    const double r_p = rp(cos_a, cos_b, n_r);
    const double r_s = rs(cos_a, cos_b, n_r);
    const double rp2 = r_p * r_p;
    const double rs2 = r_s * r_s;
    

    S_out.x = (rs2 + rp2) * 0.5 * S.x + (rs2 - rp2) * 0.5 * S.y;
    S_out.y = (rs2 - rp2) * 0.5 * S.x + (rs2 + rp2) * 0.5 * S.y;
    
    const double cos_ap = n_r / std::sqrt(n_r * n_r + 1);
    // ap < a
    if (cos_ap > cos_a)
    {
        if (n_r > 1)
        {
            const double cos_ac = std::sqrt(1-1/(n_r*n_r));
            const double sin2_a = std::pow(std::sqrt(1 - cos_a),2);
            // ac < a
            if (cos_ac > cos_a)
            {
                const double phase_shift = 2 *
                    std::atan(cos_a * std::sqrt(sin2_a- n_r*n_r)/sin2_a);
                S_out.z = cos(phase_shift) * r_s * r_p * S.z - 
                    sin(phase_shift) * r_s * r_p * S.a;
                S_out.a = sin(phase_shift) * r_s * r_p * S.z + 
                    cos(phase_shift) * r_s * r_p * S.a;
            }
            else
            {
                S_out.z = r_s * r_p * S.z;
                S_out.a = r_s * r_p * S.a;
            }
        }
        else
        {
            S_out.z = r_s * r_p * S.z;
            S_out.a = r_s * r_p * S.a;
        }
    }
    else
    {
        S_out.z = - r_s * r_p * S.z;
        S_out.a = - r_s * r_p * S.a;
    }
    return S_out;
}

ldplab::Vec4 ldplab::rtscpu::SurfaceInteractionPolarizedLight::transmittedPolarization(
    const Vec4& S, 
    const double cos_a, 
    const double cos_b, 
    const double n_r) const
{
    Vec4 S_out;

    const double t_p = tp(cos_a, cos_b, n_r);
    const double t_s = ts(cos_a, cos_b, n_r);
    const double tp2 = t_p * t_p;
    const double ts2 = t_s * t_s;
    const double area_factor = cos_b / (n_r * cos_a);

    const double temp = ts2 * area_factor + rs(cos_a, cos_b, n_r)* rs(cos_a, cos_b, n_r);

    S_out.x = area_factor * ((ts2 + tp2) * 0.5 * S.x + (ts2 - tp2) * 0.5 * S.y);
    S_out.y = area_factor * ((ts2 - tp2) * 0.5 * S.x + (ts2 + tp2) * 0.5 * S.y);
    S_out.z = area_factor * t_s * t_p * S.z;
    S_out.a = area_factor * t_s * t_p * S.a;
    return S_out;
}

