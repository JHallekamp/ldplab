#include <LDPLAB/RayTracingStep/CPU/DefaultInitialStageFactories.hpp>
#include "ImplInitialStage.hpp"

ldplab::rtscpu::default_stages::InitialStageHomogenousLightBoundingSphereProjectionFactory::
InitialStageHomogenousLightBoundingSphereProjectionFactory(
	double num_rays_per_world_space_unit)
	:
	m_num_rays_per_world_space_unit{ num_rays_per_world_space_unit }
{
}

std::string ldplab::rtscpu::default_stages::
	InitialStageHomogenousLightBoundingSphereProjectionFactory::name()
{
	return "InitialStageBoundingSpheresHomogenousLight";
}

std::string ldplab::rtscpu::default_stages::InitialStageHomogenousLightBoundingSphereProjectionFactory::
	implementationName() const
{
	return name();
}

bool ldplab::rtscpu::default_stages::InitialStageHomogenousLightBoundingSphereProjectionFactory::
	userDefined() const
{
	return false;
}

bool ldplab::rtscpu::default_stages::InitialStageHomogenousLightBoundingSphereProjectionFactory::
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
	}
}
std::shared_ptr<ldplab::rtscpu::IInitialStage> ldplab::rtscpu::default_stages::
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
