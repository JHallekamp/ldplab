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
	const ExecutionModel& execution_model,
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
	const SharedStepData& shared_data)
{
	std::vector<InitialStageHomogenousLightBoundingSphereProjection::PerDeviceData> 
		per_device_data;

	for (size_t i = 0; i < shared_data.execution_model.device_contexts.size(); ++i)
	{
		const DeviceContext& dvctx = shared_data.execution_model.device_contexts[i];
		if (!dvctx.activateDevice())
			return nullptr;
		per_device_data.emplace_back();
		auto& pdp = per_device_data.back();

		const size_t num_particles = shared_data.experimental_setup.particles.size();
		const size_t num_lights = shared_data.experimental_setup.light_sources.size();
		const size_t num_pl = num_particles * num_lights;

		if (!pdp.bounding_spheres.allocate(num_particles, i == 0))
			return nullptr;
		if (!pdp.projection_buffer.allocate(num_particles, false))
			return nullptr;
		if (!pdp.light_source_buffer.allocate(num_lights, i == 0))
			return nullptr;
		if (!pdp.num_rays_buffer.allocate(num_pl, false))
			return nullptr;
		if (!pdp.temp_num_rays_buffer.allocate(num_pl, false))
			return nullptr;

		if (i == 0)
		{
			const auto& setup = shared_data.experimental_setup;
			for (size_t j = 0; j < num_lights; ++j)
			{
				auto& light = pdp.light_source_buffer.getHostBuffer()[j];
				light.origin = setup.light_sources[j].origin_corner;
				light.ray_direction = setup.light_sources[j].orientation;
				light.ray_intensity = static_cast<LightDistributionHomogeneous*>
					(setup.light_sources[j].intensity_distribution.get())->intensity /
					(m_light_resolution_per_world_unit * m_light_resolution_per_world_unit);
				light.width =
					ceil(setup.light_sources[j].horizontal_size *
						m_light_resolution_per_world_unit);
				light.x_axis =
					(glm::normalize(setup.light_sources[j].horizontal_direction) *
						setup.light_sources[j].horizontal_size) / light.width;
				light.height =
					ceil(setup.light_sources[j].vertical_size *
						m_light_resolution_per_world_unit);
				light.y_axis =
					(glm::normalize(setup.light_sources[j].vertical_direction) *
						setup.light_sources[j].vertical_size) / light.height;
			}
		}

		if (!pdp.light_source_buffer.uploadExt(
			per_device_data[0].light_source_buffer.getHostBuffer()))
			return nullptr;
	}

	std::shared_ptr<InitialStageHomogenousLightBoundingSphereProjection> initial_state =
		std::make_shared<InitialStageHomogenousLightBoundingSphereProjection>(
			m_light_resolution_per_world_unit,
			std::move(per_device_data));
	return initial_state;
}

#endif