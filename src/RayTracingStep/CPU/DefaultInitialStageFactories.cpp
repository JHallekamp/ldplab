#include <LDPLAB/RayTracingStep/CPU/DefaultInitialStageFactories.hpp>
#include "ImplInitialStage.hpp"

ldplab::rtscpu::default_factories::InitialStageHomogenousLightBoundingSphereProjectionFactory::
InitialStageHomogenousLightBoundingSphereProjectionFactory(
	double num_rays_per_world_space_unit)
	:
	m_num_rays_per_world_space_unit{ num_rays_per_world_space_unit }
{
}

std::string ldplab::rtscpu::default_factories::
	InitialStageHomogenousLightBoundingSphereProjectionFactory::name()
{
	return "InitialStageBoundingSpheresHomogenousLight";
}

std::string ldplab::rtscpu::default_factories::InitialStageHomogenousLightBoundingSphereProjectionFactory::
	implementationName() const
{
	return name();
}

bool ldplab::rtscpu::default_factories::InitialStageHomogenousLightBoundingSphereProjectionFactory::
	userDefined() const
{
	return false;
}

bool ldplab::rtscpu::default_factories::InitialStageHomogenousLightBoundingSphereProjectionFactory::
	checkCompability(
		const RayTracingStepCPUInfo& step_info,
		const PipelineConfiguration& configuration, 
		const ExperimentalSetup& setup, 
		const InterfaceMapping& interface_mapping)
{
	for (size_t i = 0; i < setup.particles.size(); ++i)
	{
		if (setup.particles[i].bounding_volume->type() !=
			IBoundingVolume::Type::sphere)
		{
			return false;
		}
	}
	for (size_t i = 0; i < setup.light_sources.size(); ++i)
	{
		if (setup.light_sources[i].intensity_distribution->type() !=
			ILightDistribution::Type::homogeneous)
		{
			return false;
		}
		if (setup.light_sources[i].polarization->type() !=
			ILightPolarisation::Type::unpolarized)
		{
			return false;
		}
	}
	return true;
}

std::shared_ptr<ldplab::rtscpu::IInitialStage> ldplab::rtscpu::default_factories::
	InitialStageHomogenousLightBoundingSphereProjectionFactory::create(
		const RayTracingStepCPUInfo& step_info,
		const PipelineConfiguration& configuration,
		const ExperimentalSetup& setup,
		const InterfaceMapping& interface_mapping)
{
	std::vector<LightSource> light_sources = setup.light_sources;
	return std::make_shared<InitialStageBoundingSpheresHomogenousLight>(
		std::move(light_sources), m_num_rays_per_world_space_unit);
}

std::shared_ptr<ldplab::rtscpu::default_factories::InitialStageHomogenousPolarizedLightBoundingSphereProjectionFactory::PolarizationData> ldplab::rtscpu::default_factories::InitialStageHomogenousPolarizedLightBoundingSphereProjectionFactory::
	getPolatizationData(
		const MemoryInfo& memory_info, 
		const RayTracingStepCPUInfo& step_info)
{
	if (memory_info.thread_idx < m_polarization_data_pointer.size())
		return m_polarization_data_pointer[memory_info.thread_idx];
	else
	{
		m_polarization_data_pointer.emplace_back(new PolarizationData);
		auto data = m_polarization_data_pointer.back();
		data->polarization_data.resize(
			memory_info.num_ray_buffers * step_info.number_rays_per_buffer);
		for (size_t i = 0; i < memory_info.num_ray_buffers; ++i)
		{
			data->polarization_buffers.emplace_back(i, step_info.number_rays_per_buffer);
			const size_t offset = i * step_info.number_rays_per_buffer;
			data->polarization_buffers.back().polarization_data = &data->polarization_data[offset];
		}
		return data;
	}
}

ldplab::rtscpu::default_factories::InitialStageHomogenousPolarizedLightBoundingSphereProjectionFactory::
	InitialStageHomogenousPolarizedLightBoundingSphereProjectionFactory(
		double num_rays_per_world_space_unit)
	:
	m_num_rays_per_world_space_unit{ num_rays_per_world_space_unit }
{
}

std::string ldplab::rtscpu::default_factories::
InitialStageHomogenousPolarizedLightBoundingSphereProjectionFactory::name()
{
	return "InitialStageBoundingSpheresHomogenousPolarizedLight";
}

std::string ldplab::rtscpu::default_factories::InitialStageHomogenousPolarizedLightBoundingSphereProjectionFactory::
	implementationName() const
{
	return name();
}

bool ldplab::rtscpu::default_factories::InitialStageHomogenousPolarizedLightBoundingSphereProjectionFactory::
	userDefined() const
{
	return false;
}

bool ldplab::rtscpu::default_factories::InitialStageHomogenousPolarizedLightBoundingSphereProjectionFactory::
	checkCompability(
	const RayTracingStepCPUInfo& step_info,
	const PipelineConfiguration& configuration,
	const ExperimentalSetup& setup,
	const InterfaceMapping& interface_mapping)
{
	for (size_t i = 0; i < setup.particles.size(); ++i)
	{
		if (setup.particles[i].bounding_volume->type() !=
			IBoundingVolume::Type::sphere)
		{
			return false;
		}
	}
	for (size_t i = 0; i < setup.light_sources.size(); ++i)
	{
		if (setup.light_sources[i].intensity_distribution->type() !=
			ILightDistribution::Type::homogeneous)
		{
			return false;
		}
		if (setup.light_sources[i].polarization->type() !=
			ILightPolarisation::Type::polarized)
		{
			return false;
		}
	}
	return true;
}

std::shared_ptr<ldplab::rtscpu::IInitialStage> ldplab::rtscpu::default_factories::
InitialStageHomogenousPolarizedLightBoundingSphereProjectionFactory::create(
	const RayTracingStepCPUInfo& step_info,
	const PipelineConfiguration& configuration,
	const ExperimentalSetup& setup,
	const InterfaceMapping& interface_mapping)
{
	std::vector<LightSource> light_sources = setup.light_sources;
	return std::make_shared<InitialStageBoundingSpheresHomogenousPolarizedLight>(
		std::move(light_sources), m_num_rays_per_world_space_unit);
}

bool ldplab::rtscpu::default_factories::
	InitialStageHomogenousPolarizedLightBoundingSphereProjectionFactory::
	createStageDependentData(
		const MemoryInfo& memory_info, 
		const RayTracingStepCPUInfo& step_info,
		const PipelineConfiguration& configuration,
		const ExperimentalSetup& setup,
		const InterfaceMapping& interface_mapping,
		std::shared_ptr<void>& stage_dependent_data)
{
	stage_dependent_data = getPolatizationData(memory_info, step_info);
	return true;
}
