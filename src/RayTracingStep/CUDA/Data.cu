#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include <LDPLAB/RayTracingStep/CUDA/Data.hpp>

#include "../../Utils/Log.hpp"

#include <LDPLAB/RayTracingStep/CUDA/Data.hpp>
#include <LDPLAB/RayTracingStep/CUDA/Factories.hpp>
#include <LDPLAB/RayTracingStep/CUDA/DefaultBoundingVolumeIntersectionFactories.hpp>
#include <LDPLAB/RayTracingStep/CUDA/DefaultGenericGeometryFactories.hpp>
#include <LDPLAB/RayTracingStep/CUDA/DefaultGenericMaterialFactories.hpp>
#include <LDPLAB/RayTracingStep/CUDA/DefaultInitialStageFactories.hpp>
#include <LDPLAB/RayTracingStep/CUDA/DefaultInnerParticlePropagationFactories.hpp>
#include <LDPLAB/RayTracingStep/CUDA/DefaultParticleIntersectionFactories.hpp>
#include <LDPLAB/RayTracingStep/CUDA/DefaultSurfaceInteractionFactories.hpp>

#include "RayTracingStepCUDA.hpp"

namespace /*helper*/
{
	std::vector<ldplab::rtscuda::DeviceProperties> getDevices()
	{
		using namespace ldplab;
		using namespace rtscuda;
		int device_count = 0;
		cudaError_t error_id = cudaGetDeviceCount(&device_count);
		if (error_id != cudaSuccess)
		{
			LDPLAB_LOG_ERROR("RTSCUDA execution model: Failed to get cuda "\
				"device count, cudaGetDeviceCount returned error code %i: %s",
				error_id,
				cudaGetErrorString(error_id));
			return std::vector<DeviceProperties>();
		}
		if (device_count <= 0)
		{
			LDPLAB_LOG_ERROR("RTSCUDA execution model: Failed to find any cuda devices.");
			return std::vector<DeviceProperties>();
		}
		std::vector<DeviceProperties> device_properties;
		for (int i = 0; i < device_count; ++i)
		{
			cudaDeviceProp props;
			error_id = cudaGetDeviceProperties(&props, i);
			if (error_id != cudaSuccess)
			{
				LDPLAB_LOG_ERROR("RTSCUDA execution model: Failed to get "\
					"properties of cuda device %i, cudaGetDeviceProperties "\
					"returned error code %i: %s",
					i,
					error_id,
					cudaGetErrorString(error_id));
				return std::vector<DeviceProperties>();
			}
			device_properties.emplace_back();
			device_properties.back().id = i;
			device_properties.back().name = std::string(props.name);
			device_properties.back().max_block_size.x = props.maxThreadsDim[0];
			device_properties.back().max_block_size.y = props.maxThreadsDim[1];
			device_properties.back().max_block_size.z = props.maxThreadsDim[2];
			device_properties.back().max_grid_size.x = props.maxGridSize[0];
			device_properties.back().max_grid_size.y = props.maxGridSize[1];
			device_properties.back().max_grid_size.z = props.maxGridSize[2];
			device_properties.back().max_num_threads_per_block = props.maxThreadsPerBlock;
			device_properties.back().max_num_threads_per_mp = props.maxThreadsPerMultiProcessor;
			device_properties.back().num_mps = props.multiProcessorCount;
			device_properties.back().registers_per_block = props.regsPerBlock;
			device_properties.back().shared_mem_per_block = props.sharedMemPerBlock;
			device_properties.back().shared_mem_per_mp = props.sharedMemPerMultiprocessor;
			device_properties.back().warp_size = props.warpSize;
			LDPLAB_LOG_DEBUG("RTSCUDA execution model: Found cuda device "\
				"\"%s\" (device id %i)",
				device_properties.back().name.c_str(),
				device_properties.back().id);
		}
		return device_properties;
	}
}

bool ldplab::rtscuda::SharedStepData::allocateResources(
	const RayTracingStepCUDAInfo& info, 
	PipelineConfiguration& pipeline_config, 
	ExperimentalSetup&& param_setup, 
	InterfaceMapping&& param_interface_mapping)
{
	this->experimental_setup = std::move(param_setup);
	this->interface_mapping = std::move(param_interface_mapping);

	// Set simulation parameter
	simulation_parameter.intensity_cutoff = info.intensity_cutoff;
	simulation_parameter.max_branching_depth = info.maximum_branching_depth;
	simulation_parameter.num_particles = experimental_setup.particles.size();
	simulation_parameter.num_rays_per_batch = info.number_rays_per_batch;
	simulation_parameter.num_surface_interaction_reflection_passes =
		info.number_reflections;
	simulation_parameter.num_surface_interaction_transmission_passes =
		info.number_transmissions;
	simulation_parameter.ray_world_space_index = 
		simulation_parameter.num_particles;
	simulation_parameter.ray_invalid_index = -1;
	simulation_parameter.output_in_particle_space = 
		info.return_force_in_particle_coordinate_system;
	simulation_parameter.buffer_min_size = info.buffer_min_size;
	simulation_parameter.buffer_reorder_threshold = info.buffer_reorder_threshold;
	simulation_parameter.sort_ray_buffer =
		info.sort_ray_buffer;

	// Create device data
	for (size_t i = 0; i < per_device_data.size(); ++i)
	{
		if (!allocateDeviceData(info, per_device_data[i], pipeline_config))
		{
			LDPLAB_LOG_ERROR("RTSCUDA shared step data: Failed to allocate "\
				"device data for device %i", 
				per_device_data[i].associated_device_group);
			return false;
		}
	}
	return true;
}

bool ldplab::rtscuda::SharedStepData::allocateDeviceData(
	const RayTracingStepCUDAInfo& info,
	DeviceData& device_data,
	PipelineConfiguration& pipeline_config)
{
	DeviceContext& devctx = execution_model.device_contexts[
		device_data.associated_device_group];
	if (!devctx.activateDevice())
		return false;

	// Create generic geometries and materials
	bool error = false;
	std::map<const IParticleGeometry*, size_t> reusable_geometry;
	std::map<const IParticleMaterial*, size_t> reusable_material;
	for (size_t i = 0; i < experimental_setup.particles.size(); ++i)
	{
		auto reusable_geometry_index_it = reusable_geometry.find(
			experimental_setup.particles[i].geometry.get());
		if (reusable_geometry_index_it == reusable_geometry.end())
		{
			// No reusable geometry present
			auto geo_factory_it = pipeline_config.generic_geometries.find(
				experimental_setup.particles[i].geometry->type());
			if (geo_factory_it == pipeline_config.generic_geometries.end())
			{
				LDPLAB_LOG_ERROR("RTSCUDA shared step data: "\
					"Could not find generic geometry factory for particle "\
					"type \"%s\" in the given particle configuration",
					IParticleGeometry::typeToString(
						experimental_setup.particles[i].geometry->type()));
				error = true;
			}
			else
			{
				std::shared_ptr<IGenericGeometry> generic_geometry =
					geo_factory_it->second->create(
						experimental_setup.particles[i].geometry,
						info,
						device_data.device_properties,
						pipeline_config,
						experimental_setup,
						interface_mapping);
				if (generic_geometry == nullptr)
				{
					LDPLAB_LOG_ERROR("RTSCUDA shared step data: "\
						"Could not create generic geometry for particle %i "\
						"of type \"%s\"",
						experimental_setup.particles[i].uid,
						IParticleGeometry::typeToString(
							experimental_setup.particles[i].geometry->type()));
					error = true;
				}
				else
				{
					reusable_geometry.emplace(
						experimental_setup.particles[i].geometry.get(),
						device_data.particle_data_buffers.geometry_instances.size());
					device_data.particle_data_buffers.geometry_instances.
						emplace_back(std::move(generic_geometry));
				}
			}
		}
		else
		{
			// Reuse geometry
			device_data.particle_data_buffers.geometry_instances.emplace_back(
				device_data.particle_data_buffers.geometry_instances[
					reusable_geometry_index_it->second]);
		}

		auto reusable_material_index_it = reusable_material.find(
			experimental_setup.particles[i].material.get());
		if (reusable_material_index_it == reusable_material.end())
		{
			// No reusable material present
			auto mat_factory_it = pipeline_config.generic_materials.find(
				experimental_setup.particles[i].material->type());
			if (mat_factory_it == pipeline_config.generic_materials.end())
			{
				LDPLAB_LOG_ERROR("RTSCUDA shared step data: "\
					"Could not find generic material factory for particle "\
					"type \"%s\" in the given particle configuration",
					IParticleMaterial::typeToString(
						experimental_setup.particles[i].material->type()));
				error = true;
			}
			else
			{
				std::shared_ptr<IGenericMaterial> generic_material =
					mat_factory_it->second->create(
						experimental_setup.particles[i].material,
						info,
						device_data.device_properties,
						pipeline_config,
						experimental_setup,
						interface_mapping);
				if (generic_material == nullptr)
				{
					LDPLAB_LOG_ERROR("RTSCUDA shared step data: "\
						"Could not create generic material for particle %i "\
						"of type \"%s\"",
						experimental_setup.particles[i].uid,
						IParticleMaterial::typeToString(
							experimental_setup.particles[i].material->type()));
					error = true;
				}
				else
				{
					reusable_material.emplace(
						experimental_setup.particles[i].material.get(),
						device_data.particle_data_buffers.material_instances.size());
					device_data.particle_data_buffers.material_instances.
						emplace_back(std::move(generic_material));
				}
			}
		}
		else
		{
			// Reuse material
			device_data.particle_data_buffers.material_instances.emplace_back(
				device_data.particle_data_buffers.material_instances[
					reusable_material_index_it->second]);
		}
	}

	if (error)
		return false;

	// Allocate particle data
	auto& pt = device_data.particle_transformation_buffers;

	const size_t num_particles = experimental_setup.particles.size();
	error = error || !pt.p2w_transformation_buffer.allocate(num_particles, true);
	error = error || !pt.p2w_translation_buffer.allocate(num_particles, true);
	error = error || !pt.w2p_transformation_buffer.allocate(num_particles, true);
	error = error || !pt.w2p_translation_buffer.allocate(num_particles, true);
	if (error)
	{
		LDPLAB_LOG_ERROR("RTSCUDA shared step data: Failed to create particle "\
			"transformation device buffers.");
		return false;
	}

	auto& pd = device_data.particle_data_buffers;

	error = error || !pd.geometry_data_buffer.allocate(num_particles, true);
	error = error || !pd.intersect_ray_fptr_buffer.allocate(num_particles, true);
	error = error || !pd.intersect_segment_fptr_buffer.allocate(num_particles, true);
	error = error || !pd.index_of_refraction_fptr_buffer.allocate(num_particles, true);
	error = error || !pd.material_data_buffer.allocate(num_particles, true);
	if (error)
	{
		LDPLAB_LOG_ERROR("RTSCUDA shared step data: Failed to create particle "\
			"material and geometry device buffers.");
		return false;
	}

	for (size_t i = 0; i < num_particles; ++i)
	{
		pd.geometry_data_buffer.getHostBuffer()[i] =
			pd.geometry_instances[i]->getDeviceData();
		pd.intersect_ray_fptr_buffer.getHostBuffer()[i] =
			pd.geometry_instances[i]->getDeviceIntersectRayFunction();
		pd.intersect_segment_fptr_buffer.getHostBuffer()[i] =
			pd.geometry_instances[i]->getDeviceIntersectSegmentFunction();
		pd.index_of_refraction_fptr_buffer.getHostBuffer()[i] =
			pd.material_instances[i]->getDeviceIndexOfRefractionFunction();
		pd.material_data_buffer.getHostBuffer()[i] =
			pd.material_instances[i]->getDeviceData();
		error = error || (pd.geometry_data_buffer.getHostBuffer()[i] == nullptr);
		error = error || (pd.intersect_ray_fptr_buffer.getHostBuffer()[i] == nullptr);
		error = error || (pd.intersect_segment_fptr_buffer.getHostBuffer()[i] == nullptr);
		error = error || (pd.index_of_refraction_fptr_buffer.getHostBuffer()[i] == nullptr);
		error = error || (pd.material_data_buffer.getHostBuffer()[i] == nullptr);
	}
	if (error)
	{
		LDPLAB_LOG_ERROR("RTSCUDA shared step data: Failed to receive a device function "\
			"pointer for generic geometry or material functions.");
		return false;
	}

	error = error || !pd.geometry_data_buffer.upload();
	error = error || !pd.intersect_ray_fptr_buffer.upload();
	error = error || !pd.intersect_segment_fptr_buffer.upload();
	error = error || !pd.index_of_refraction_fptr_buffer.upload();
	error = error || !pd.material_data_buffer.upload();
	if (error)
	{
		LDPLAB_LOG_ERROR("RTSCUDA shared step data: Failed to upload buffers for "\
			"generic geometry or material data.");
		return false;
	}

	if (!pd.center_of_mass_buffer.allocate(num_particles, true))
	{
		LDPLAB_LOG_ERROR("RTSCUDA shared step data: Failed to allocate "\
			"particle center of mass device buffer.");
		return false;
	}

	for (size_t i = 0; i < num_particles; ++i)
	{
		pd.center_of_mass_buffer.getHostBuffer()[i] =
			experimental_setup.particles[i].centre_of_mass;
	}

	if (!pd.center_of_mass_buffer.upload())
	{
		LDPLAB_LOG_ERROR("RTSCUDA shared step data: Failed to upload "\
			"particle center of mass device buffer.");
		return false;
	}

	for (size_t i = 0; i < device_data.per_stream_data.size(); ++i)
	{
		if (!allocateStreamData(device_data.per_stream_data[i]))
		{
			LDPLAB_LOG_ERROR("RTSCUDA shared step data: Failed to allocate "\
				"stream data for stream %i on device %i",
				device_data.per_stream_data[i].associated_stream,
				device_data.associated_device_group);
			return false;
		}
	}
	return true;
}

bool ldplab::rtscuda::SharedStepData::allocateStreamData(
	StreamData& stream_data)
{
#ifdef NDEBUG
	constexpr size_t downloadable = 0;
#else
	constexpr size_t downloadable = 1;
#endif

	const size_t num_particles = simulation_parameter.num_particles;
	const size_t num_rays = simulation_parameter.num_rays_per_batch;
	const size_t branching_depth = simulation_parameter.max_branching_depth;
	bool error = false;

	// Create ray buffer
	error = error || !stream_data.ray_data_buffers.direction_buffers.allocate(
		num_rays, branching_depth + 2, downloadable);
	error = error || !stream_data.ray_data_buffers.intensity_buffers.allocate(
		num_rays, branching_depth + 2, downloadable);
	error = error || !stream_data.ray_data_buffers.min_bv_distance_buffers.allocate(
		num_rays, branching_depth + 2, downloadable);
	error = error || !stream_data.ray_data_buffers.origin_buffers.allocate(
		num_rays, branching_depth + 2, downloadable);
	error = error || !stream_data.ray_data_buffers.particle_index_buffers.allocate(
		num_rays, branching_depth + 2, downloadable);
	if (error)
	{
		LDPLAB_LOG_ERROR("RTSCUDA shared step data: Failed to allocate ray data "\
			"device buffers.");
		return false;
	}

	// Create intersection buffer
	error = error || !stream_data.intersection_data_buffers.normal_buffers.allocate(
		num_rays, branching_depth + 1, downloadable);
	error = error || !stream_data.intersection_data_buffers.particle_index_buffers.allocate(
		num_rays, branching_depth + 1, downloadable);
	error = error || !stream_data.intersection_data_buffers.point_buffers.allocate(
		num_rays, branching_depth + 1, downloadable);
	if (error)
	{
		LDPLAB_LOG_ERROR("RTSCUDA shared step data: Failed to allocate intersection "\
			"data device buffers.");
		return false;
	}

	// Create output buffer
	error = error || !stream_data.output_data_buffers.force_per_particle_buffer.allocate(
		num_particles, true);
	error = error || !stream_data.output_data_buffers.torque_per_particle_buffer.allocate(
		num_particles, true);
	error = error || !stream_data.output_data_buffers.force_per_ray_buffer.allocate(
		num_rays, branching_depth + 1, downloadable);
	error = error || !stream_data.output_data_buffers.torque_per_ray_buffer.allocate(
		num_rays, branching_depth + 1, downloadable);
	if (error)
	{
		LDPLAB_LOG_ERROR("RTSCUDA shared step data: Failed to allocate output "\
			"data device buffers.");
		return false;
	}

	return true;
}

bool ldplab::rtscuda::SharedStepData::createExecutionModel(
	std::shared_ptr<IExecutionModelInfo> info)
{
	bool result = false;
	switch (info->type())
	{
	case IExecutionModelInfo::Type::auto_construction:
		result = createExecutionDataAuto(
			*static_cast<ExecutionModelAutoConstructionInfo*>(info.get()));
		break;
	case IExecutionModelInfo::Type::explicit_configuration:
		result = createExecutionDataExplicit(
			*static_cast<ExecutionModelExplicitConfigInfo*>(info.get()));
		break;
	default:
		LDPLAB_LOG_ERROR("RTSCUDA execution model: Unknown type");
		return false;
	}
	if (!result)
		return false;
	for (size_t i = 0; i < per_device_data.size(); ++i)
	{
		DeviceData* device_data = &per_device_data[i];
		std::vector<size_t> stream_ids;
		for (size_t j = 0; j < device_data->per_stream_data.size(); ++j)
			stream_ids.push_back(device_data->per_stream_data[j].associated_stream);
		execution_model.device_contexts.push_back(std::move(DeviceContext(
			device_data->associated_device_group,
			device_data->device_properties,
			std::move(stream_ids))));
		if (!execution_model.device_contexts.back().activateDevice())
			return false;
		for (size_t j = 0; j < device_data->per_stream_data.size(); ++j)
		{
			auto smctx = StreamContext(
				execution_model.device_contexts.back(),
				*device_data,
				experimental_setup,
				interface_mapping,
				simulation_parameter,
				device_data->per_stream_data[j]);
			execution_model.stream_contexts.push_back(smctx);
			if (!execution_model.stream_contexts.back().allocate())
				return false;
		}
		LDPLAB_LOG_INFO("RTSCUDA execution model: LDPLAB is now using cuda "\
			"device %i \"%s\" in device group %i with %i streams",
			device_data->device_properties.id,
			device_data->device_properties.name.c_str(),
			device_data->associated_device_group,
			device_data->per_stream_data.size());

	}
	return true;
}

bool ldplab::rtscuda::SharedStepData::createExecutionDataAuto(
	ExecutionModelAutoConstructionInfo& info)
{
	auto device_properties = getDevices();
	if (device_properties.size() == 0)
		return false;
	size_t lim = 1;
	if (info.device_model == 
		ExecutionModelAutoConstructionInfo::DeviceModel::distribute_equally)
		lim = device_properties.size();
	size_t num_streams = 0;
	for (size_t i = 0; i < lim; ++i)
	{
		per_device_data.emplace_back();
		per_device_data.back().associated_device_group = i;
		per_device_data.back().device_properties = std::move(device_properties[i]);
		for (size_t j = 0; j < info.num_streams_per_device; ++j)
		{
			const size_t stream_id = num_streams++;
			per_device_data.back().stream_id_to_on_device_index.emplace(stream_id, j);
			per_device_data.back().per_stream_data.emplace_back();
			per_device_data.back().per_stream_data.back().associated_stream = stream_id;
		}
	}
	return true;
}

bool ldplab::rtscuda::SharedStepData::createExecutionDataExplicit(
	ExecutionModelExplicitConfigInfo& info)
{
	if (info.map_device_group_to_device_id.size() != info.num_device_groups)
	{
		LDPLAB_LOG_ERROR("RTSCUDA execution model: Mismatch between expected "\
			"and mapped number of device groups.");
		return false;
	}
	else if (info.map_stream_to_device_group.size() != info.num_streams)
	{
		LDPLAB_LOG_ERROR("RTSCUDA execution model: Mismatch between expected "\
			"and mapped number of streams.");
		return false;
	}

	for (auto it = info.map_stream_to_device_group.begin();
		it != info.map_stream_to_device_group.end();
		++it)
	{
		if (it->first >= info.num_streams)
		{
			LDPLAB_LOG_ERROR("RTSCUDA execution model: Mapped invalid stream.");
			return false;
		}
		if (it->second >= info.num_device_groups)
		{
			LDPLAB_LOG_ERROR("RTSCUDA execution model: Mapped stream to invalid "\
				"device group.");
			return false;
		}
	}

	auto device_properties = getDevices();
	if (device_properties.size() == 0)
		return false;
	for (auto it = info.map_device_group_to_device_id.begin();
		it != info.map_device_group_to_device_id.end();
		++it)
	{
		if (it->first >= info.num_device_groups)
		{
			LDPLAB_LOG_ERROR("RTSCUDA execution model: Mapped invalid device "\
				"group.");
			return false;
		}
		if (it->second < 0 || it->second >= device_properties.size())
		{
			LDPLAB_LOG_ERROR("RTSCUDA execution model: Mapped device group to "\
				"invalid device id.");
			return false;
		}
	}

	for (size_t i = 0; i < info.num_device_groups; ++i)
	{
		per_device_data.emplace_back();
		per_device_data.back().associated_device_group = i;
		per_device_data.back().device_properties = std::move(
			device_properties[info.map_device_group_to_device_id[i]]);
	}

	for (size_t i = 0; i < info.num_streams; ++i)
	{
		DeviceData& device_data = per_device_data[info.map_stream_to_device_group[i]];
		device_data.stream_id_to_on_device_index.emplace(
			i, device_data.per_stream_data.size());
		device_data.per_stream_data.emplace_back();
		device_data.per_stream_data.back().associated_stream = i;
	}

	return true;
}

#endif