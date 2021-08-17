#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include <LDPLAB/RayTracingStep/CUDA/DefaultGenericMaterialFactories.hpp>

#include <LDPLAB/RayTracingStep/CUDA/DeviceResource.hpp>

#include "ImplGenericMaterial.hpp"

std::string ldplab::rtscuda::default_factories::GenericMaterialLinearOneDirectionalFactory::
name()
{
	return "GenericMaterialLinearOneDirectional";
}

std::string ldplab::rtscuda::default_factories::GenericMaterialLinearOneDirectionalFactory::
implementationName() const
{
	return name();
}

bool ldplab::rtscuda::default_factories::GenericMaterialLinearOneDirectionalFactory::
userDefined() const
{
	return false;
}

bool ldplab::rtscuda::default_factories::GenericMaterialLinearOneDirectionalFactory::
checkCompability(
	IParticleMaterial::Type material_type, 
	const RayTracingStepCUDAInfo& step_info, 
	const GlobalData::DeviceProperties& device_properties, 
	const PipelineConfiguration& configuration, 
	const ExperimentalSetup& setup, 
	const InterfaceMapping& interface_mapping)
{
	return (material_type == IParticleMaterial::Type::linear_one_directional);
}

std::shared_ptr<ldplab::rtscuda::IGenericMaterial> ldplab::rtscuda::default_factories::
GenericMaterialLinearOneDirectionalFactory::
create(
	const std::shared_ptr<IParticleMaterial>& particle_material, 
	const RayTracingStepCUDAInfo& step_info, 
	const GlobalData::DeviceProperties& device_properties, 
	const PipelineConfiguration& configuration, 
	const ExperimentalSetup& setup, 
	const InterfaceMapping& interface_mapping)
{
	DeviceBuffer<MaterialLinearOneDirectionalData> material_buffer;
	if (!material_buffer.allocate(1, true))
		return nullptr;
	ParticleMaterialLinearOneDirectional* material =
		static_cast<ParticleMaterialLinearOneDirectional*>(particle_material.get());
	
	MaterialLinearOneDirectionalData* md = material_buffer.getHostBuffer();
	md->direction_times_gradient = material->direction * material->gradient;
	md->index_of_refraction_minus_partial_dot =
		material->index_of_refraction -
		glm::dot(md->direction_times_gradient, material->origin);
	//material_buffer.getHostBuffer()->direction = material->direction;
	//material_buffer.getHostBuffer()->gradient = material->gradient;
	//material_buffer.getHostBuffer()->index_of_refraction = material->index_of_refraction;
	//material_buffer.getHostBuffer()->origin = material->origin;
	if (!material_buffer.upload())
		return nullptr;
	std::shared_ptr<GenericMaterialLinearOneDirectional> generic_material =
		std::make_shared<GenericMaterialLinearOneDirectional>(std::move(material_buffer));
	return generic_material;
}

#endif