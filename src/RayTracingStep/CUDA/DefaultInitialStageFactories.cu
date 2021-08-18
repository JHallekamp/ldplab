#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include <LDPLAB/RayTracingStep/CUDA/DefaultInitialStageFactories.hpp>

#include "ImplInitialStage.hpp"
#include "../../Utils/Log.hpp"

ldplab::rtscuda::default_factories::InitialStageHomogenousLightBoundingSphereProjectionFactory::
InitialStageHomogenousLightBoundingSphereProjectionFactory(
	double light_resolution_per_world_unit)
	:
	m_light_resolution_per_world_unit{ light_resolution_per_world_unit }
{ }

std::string ldplab::rtscuda::default_factories::InitialStageHomogenousLightBoundingSphereProjectionFactory::
name()
{
    return "InitialStageHomogenousLightBoundingSphereProjection";
}

std::string ldplab::rtscuda::default_factories::InitialStageHomogenousLightBoundingSphereProjectionFactory::
implementationName() const
{
    return name();
}

bool ldplab::rtscuda::default_factories::InitialStageHomogenousLightBoundingSphereProjectionFactory::
userDefined() const
{
    return false;
}

bool ldplab::rtscuda::default_factories::InitialStageHomogenousLightBoundingSphereProjectionFactory::
checkCompability(
    const RayTracingStepCUDAInfo& step_info,
    const GlobalData::DeviceProperties& device_properties,
    const PipelineConfiguration& configuration,
    const ExperimentalSetup& setup,
    const InterfaceMapping& interface_mapping)
{
	for (size_t i = 0; i < setup.particles.size(); ++i)
	{
		if (setup.particles[i].bounding_volume->type() !=
			IBoundingVolume::Type::sphere)
		{
			LDPLAB_LOG_ERROR("RTSCUDA initial stage factory: Particle %i is "\
				"incomaptible, only spherical particle geometries allowed", i);
			return false;
		}
	}
	for (size_t i = 0; i < setup.light_sources.size(); ++i)
	{
		if (setup.light_sources[i].intensity_distribution->type() !=
			ILightDistribution::Type::homogeneous)
		{
			LDPLAB_LOG_ERROR("RTSCUDA initial stage factory: Light source %i is "\
				"incomaptible, only homogeneous light sources allowed", i);
			return false;
		}
	}
	return true;
}

std::shared_ptr<ldplab::rtscuda::IInitialStage>
ldplab::rtscuda::default_factories::InitialStageHomogenousLightBoundingSphereProjectionFactory::
create(
    const RayTracingStepCUDAInfo& step_info,
    const PipelineConfiguration& configuration,
    const GlobalData& global_data)
{
	DeviceBuffer<BoundingSphere> bounding_sphere_buffer;
	DeviceBuffer<InitialStageHomogenousLightBoundingSphereProjection::Rect> rect_buffer;
	DeviceBuffer<InitialStageHomogenousLightBoundingSphereProjection::HomogenousLightSource> light_buffer;
	DeviceBuffer<size_t> num_rays_buffer;
	DeviceBuffer<size_t> temp_num_rays_buffer;

	if (!bounding_sphere_buffer.allocate(global_data.experimental_setup.particles.size(), true))
		return nullptr;
	if (!rect_buffer.allocate(global_data.experimental_setup.particles.size(), true))
		return nullptr;
	if (!light_buffer.allocate(global_data.experimental_setup.light_sources.size(), true))
		return nullptr;
	const size_t num_pl = 
		global_data.experimental_setup.particles.size() *
		global_data.experimental_setup.light_sources.size();
	if (!num_rays_buffer.allocate(num_pl, false))
		return nullptr;
	if (!temp_num_rays_buffer.allocate(num_pl, false))
		return nullptr;

	const auto& setup = global_data.experimental_setup;
	for (size_t i = 0; i < global_data.experimental_setup.light_sources.size(); ++i)
	{
		auto& light = light_buffer.getHostBuffer()[i];
		light.origin = setup.light_sources[i].origin_corner;
		light.ray_direction = setup.light_sources[i].orientation;
		light.ray_intensity = static_cast<LightDistributionHomogeneous*>
			(setup.light_sources[i].intensity_distribution.get())->intensity /
			(m_light_resolution_per_world_unit * m_light_resolution_per_world_unit);
		light.width =
			ceil(setup.light_sources[i].horizontal_size *
			m_light_resolution_per_world_unit);
		light.x_axis =
			(glm::normalize(setup.light_sources[i].horizontal_direction) * 
				setup.light_sources[i].horizontal_size) / light.width;
		light.height =
			ceil(setup.light_sources[i].vertical_size *
			m_light_resolution_per_world_unit);
		light.y_axis =
			(glm::normalize(setup.light_sources[i].vertical_direction) * 
				setup.light_sources[i].vertical_size) / light.height;
	}
	if (!light_buffer.upload())
		return nullptr;

	std::shared_ptr<InitialStageHomogenousLightBoundingSphereProjection> initial_state =
		std::make_shared<InitialStageHomogenousLightBoundingSphereProjection>(
			m_light_resolution_per_world_unit,
			std::move(bounding_sphere_buffer),
			std::move(rect_buffer),
			std::move(light_buffer),
			std::move(num_rays_buffer),
			std::move(temp_num_rays_buffer));
	return initial_state;
}

#endif