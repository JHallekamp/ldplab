#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include <LDPLAB/RayTracingStep/CUDA/DefaultGenericGeometryFactories.hpp>

#include <LDPLAB/Constants.hpp>

#include "ImplGenericGeometry.hpp"

std::string ldplab::rtscuda::default_factories::GenericGeometrySphereFactory::
name()
{
	return "GenericGeometrySphere";
}

std::string ldplab::rtscuda::default_factories::GenericGeometrySphereFactory::
implementationName() const
{
	return name();
}

bool ldplab::rtscuda::default_factories::GenericGeometrySphereFactory::
userDefined() const
{
	return false;
}

bool ldplab::rtscuda::default_factories::GenericGeometrySphereFactory::
checkCompability(
	IParticleGeometry::Type geometry_type, 
	const RayTracingStepCUDAInfo& step_info, 
	const ExecutionModel& execution_model,
	const PipelineConfiguration& configuration,
	const ExperimentalSetup& setup, 
	const InterfaceMapping& interface_mapping)
{
	return (geometry_type == IParticleGeometry::Type::sphere);
}

std::shared_ptr<ldplab::rtscuda::IGenericGeometry> 
ldplab::rtscuda::default_factories::GenericGeometrySphereFactory::
create(
	const std::shared_ptr<IParticleGeometry>& particle_geometry, 
	const RayTracingStepCUDAInfo& step_info, 
	const DeviceProperties& device_properties, 
	const PipelineConfiguration& configuration, 
	const ExperimentalSetup& setup, 
	const InterfaceMapping& interface_mapping)
{
	DeviceBuffer<GeometrySphereData> geo_buffer;
	if (!geo_buffer.allocate(1, true))
		return nullptr;
	SphericalParticleGeometry* geo =
		static_cast<SphericalParticleGeometry*>(particle_geometry.get());
	geo_buffer.getHostBuffer()->radius = geo->radius;
	if (!geo_buffer.upload())
		return nullptr;
	return std::make_shared<GenericGeometrySphere>(std::move(geo_buffer));
}

std::string ldplab::rtscuda::default_factories::GenericGeometryRodFactory::
name()
{
	return "GenericGeometryRod";
}

std::string ldplab::rtscuda::default_factories::GenericGeometryRodFactory::
implementationName() const
{
	return name();
}

bool ldplab::rtscuda::default_factories::GenericGeometryRodFactory::
userDefined() const
{
	return false;
}

bool ldplab::rtscuda::default_factories::GenericGeometryRodFactory::
checkCompability(
	IParticleGeometry::Type geometry_type,
	const RayTracingStepCUDAInfo& step_info,
	const ExecutionModel& execution_model,
	const PipelineConfiguration& configuration,
	const ExperimentalSetup& setup,
	const InterfaceMapping& interface_mapping)
{
	return (geometry_type == IParticleGeometry::Type::rod_particle);
}

std::shared_ptr<ldplab::rtscuda::IGenericGeometry>
ldplab::rtscuda::default_factories::GenericGeometryRodFactory::
create(
	const std::shared_ptr<IParticleGeometry>& particle_geometry,
	const RayTracingStepCUDAInfo& step_info,
	const DeviceProperties& device_properties,
	const PipelineConfiguration& configuration,
	const ExperimentalSetup& setup,
	const InterfaceMapping& interface_mapping)
{
	DeviceBuffer<GeometryRodData> geo_buffer;
	if (!geo_buffer.allocate(1, true))
		return nullptr;
	RodParticleGeometry* geo =
		static_cast<RodParticleGeometry*>(particle_geometry.get());
	double h, sphere_radius;
	if (geo->kappa <= constant::rod_particle::min_kappa_threshold)
		h = sphere_radius = 0;
	else
	{
		h = geo->kappa * geo->cylinder_radius;
		sphere_radius = 0.5 *
			(h + geo->cylinder_radius * geo->cylinder_radius / h);
	}
	geo_buffer.getHostBuffer()->cylinder_length = geo->cylinder_length;
	geo_buffer.getHostBuffer()->cylinder_radius = geo->cylinder_radius;
	geo_buffer.getHostBuffer()->sphere_radius = sphere_radius;
	geo_buffer.getHostBuffer()->origin_cap =
		Vec3(0, 0, geo->cylinder_length + h - sphere_radius);
	geo_buffer.getHostBuffer()->origin_indentation =
		Vec3(0, 0, h - sphere_radius);
	if (!geo_buffer.upload())
		return nullptr;
	return std::make_shared<GenericGeometryRod>(std::move(geo_buffer));
}

#endif