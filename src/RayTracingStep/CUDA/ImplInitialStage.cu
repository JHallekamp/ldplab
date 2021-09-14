#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "ImplInitialStage.hpp"

namespace homogenous_light_bounding_sphere_projection
{
	using namespace ldplab;
	using namespace ldplab::rtscuda;
	__global__ void projectParticlesKernel(
		BoundingSphere* bounding_spheres,
		InitialStageHomogenousLightBoundingSphereProjection::Rect* projection_buffer,
		InitialStageHomogenousLightBoundingSphereProjection::HomogenousLightSource* light_buffer,
		size_t* temp_num_rays_buffer_ptr,
		double light_source_resolution_per_world_unit);
	__global__ void countTotalRaysKernelFirst(
		size_t* temp_num_rays_buffer_ptr,
		size_t num_blocks);
	__global__ void countTotalRaysKernelSecond(
		size_t* temp_num_rays_buffer_ptr,
		size_t* num_rays_buffer_ptr,
		size_t num_blocks,
		size_t num_rays_per_batch);
	__global__ void createBatchKernel(
		int32_t* ray_index_buffer,
		Vec3* ray_origin_buffer,
		Vec3* ray_direction_buffer,
		double* ray_intensity_buffer,
		double* ray_min_bv_dist_buffer,
		InitialStageHomogenousLightBoundingSphereProjection::Rect* projection_buffer,
		InitialStageHomogenousLightBoundingSphereProjection::HomogenousLightSource* light_buffer,
		size_t* num_rays_buffer,
		size_t num_particles,
		size_t num_light_sources,
		size_t num_rays_per_batch,
		size_t batch_no);
	__device__ size_t total_num_rays;
}

ldplab::rtscuda::InitialStageHomogenousLightBoundingSphereProjection::
	InitialStageHomogenousLightBoundingSphereProjection(
		double light_resolution_per_world_unit,
		std::vector<PerDeviceData>&& per_device_data)
	:
	m_light_resolution_per_world_unit{ light_resolution_per_world_unit },
	m_per_device_data{ std::move(per_device_data) }
{ }

void ldplab::rtscuda::InitialStageHomogenousLightBoundingSphereProjection::
	stepSetup(
		const SimulationState& simulation_state,
		SharedStepData& shared_data,
		DeviceContext& device_context)
{
	if (device_context.groupId() == 0)
	{
		BoundingSphere* spheres = m_per_device_data[0].bounding_spheres.getHostBuffer();
		for (size_t i = 0; i < shared_data.experimental_setup.particles.size(); ++i)
		{
			// Get the particle instance for particle i using the interface mapping
			const ParticleInstance& particle_instance =
				simulation_state.particle_instances.find(
					shared_data.interface_mapping.particle_index_to_uid.at(i))->second;
			// Get the bounding sphere in pspace
			spheres[i] = *static_cast<BoundingVolumeSphere*>(
				shared_data.experimental_setup.particles[i].bounding_volume.get());
			// Translate bounding volume center to world space
			const auto& p2w_transformation =
				shared_data.per_device_data[0].particle_transformation_buffers.
				p2w_transformation_buffer.getHostBuffer()[i];
			const auto& p2w_translation =
				shared_data.per_device_data[0].particle_transformation_buffers.
				p2w_translation_buffer.getHostBuffer()[i];
			spheres[i].center = p2w_transformation * spheres[i].center + p2w_translation;
		}
	}

	// Upload data
	PerDeviceData& pdp = m_per_device_data[device_context.groupId()];
	pdp.bounding_spheres.uploadExt(
		m_per_device_data[0].bounding_spheres.getHostBuffer());

	// Execute setup kernel
	using namespace homogenous_light_bounding_sphere_projection;
	const size_t grid_size = pdp.projection_buffer.bufferSize();
	const size_t block_size = pdp.light_source_buffer.bufferSize();
	const size_t mem_size = block_size * sizeof(size_t);
	projectParticlesKernel<<<grid_size, block_size>>> (
		pdp.bounding_spheres.getDeviceBuffer(),
		pdp.projection_buffer.getDeviceBuffer(),
		pdp.light_source_buffer.getDeviceBuffer(),
		pdp.temp_num_rays_buffer.getDeviceBuffer(),
		m_light_resolution_per_world_unit);
	countTotalRaysKernelFirst<<<grid_size, block_size, mem_size>>>(
		pdp.temp_num_rays_buffer.getDeviceBuffer(),
		grid_size);
	countTotalRaysKernelSecond<<<grid_size, block_size, mem_size>>> (
		pdp.temp_num_rays_buffer.getDeviceBuffer(),
		pdp.num_rays_buffer.getDeviceBuffer(),
		grid_size,
		shared_data.simulation_parameter.num_rays_per_batch);

	// Download the total number of rays
	size_t total_rays;
	if (cudaMemcpyFromSymbol(
		&total_rays,
		total_num_rays,
		sizeof(total_num_rays)) != cudaSuccess)
	{
		total_rays = 0;
	}
	else
		bool breakpoint = true;
	// Calculate the number of batches
	m_total_batch_count = 
		total_rays / shared_data.simulation_parameter.num_rays_per_batch +
		(total_rays % shared_data.simulation_parameter.num_rays_per_batch ? 1 : 0);
}

bool ldplab::rtscuda::InitialStageHomogenousLightBoundingSphereProjection::
	execute(
		StreamContext& stream_context,
		size_t batch_no,
		size_t initial_batch_buffer_index)
{
	if (batch_no >= m_total_batch_count)
		return false;

	using namespace homogenous_light_bounding_sphere_projection;
	const size_t block_size = 128;
	const size_t grid_size = 
		stream_context.simulationParameter().num_rays_per_batch / block_size +
		(stream_context.simulationParameter().num_rays_per_batch % block_size ? 1 : 0);
	PerDeviceData& pdp = m_per_device_data[stream_context.deviceGroup()];
	createBatchKernel<<<grid_size, block_size, 0, stream_context.cudaStream()>>>(
		stream_context.rayDataBuffers().particle_index_buffers.getDeviceBuffer(initial_batch_buffer_index),
		stream_context.rayDataBuffers().origin_buffers.getDeviceBuffer(initial_batch_buffer_index),
		stream_context.rayDataBuffers().direction_buffers.getDeviceBuffer(initial_batch_buffer_index),
		stream_context.rayDataBuffers().intensity_buffers.getDeviceBuffer(initial_batch_buffer_index),
		stream_context.rayDataBuffers().min_bv_distance_buffers.getDeviceBuffer(initial_batch_buffer_index),
		pdp.projection_buffer.getDeviceBuffer(),
		pdp.light_source_buffer.getDeviceBuffer(),
		pdp.num_rays_buffer.getDeviceBuffer(),
		stream_context.simulationParameter().num_particles,
		pdp.light_source_buffer.bufferSize(),
		stream_context.simulationParameter().num_rays_per_batch,
		batch_no);
	return true;
}

__global__ void homogenous_light_bounding_sphere_projection::
	projectParticlesKernel(
		BoundingSphere* bounding_spheres, 
		InitialStageHomogenousLightBoundingSphereProjection::Rect* projection_buffer,
		InitialStageHomogenousLightBoundingSphereProjection::HomogenousLightSource* light_buffer,
		size_t* temp_num_rays_buffer_ptr,
		double light_source_resolution_per_world_unit)
{
	const unsigned int projection_idx = threadIdx.x * blockDim.x + blockIdx.x;

	// ========================================================================
	// Part 1: Project sphere to plane
	InitialStageHomogenousLightBoundingSphereProjection::HomogenousLightSource light =
		light_buffer[threadIdx.x];
	BoundingSphere bs = bounding_spheres[blockIdx.x];

	// Assuming ray direction is always orthogonal to light plane
	const double t = glm::dot(light.ray_direction, light.origin - bs.center) /
		-glm::dot(light.ray_direction, light.ray_direction);
	InitialStageHomogenousLightBoundingSphereProjection::Rect projection;
	if (t < 0.0)
	{
		projection.x = -1;
		projection.y = -1;
		projection.height = 0;
		projection.width = 0;
	}
	else
	{
		const Vec3 cntr = bs.center - t * light.ray_direction - light.origin;
		const Vec2 projctr = Vec2{
		   glm::dot(cntr, glm::normalize(light.x_axis)),
		   glm::dot(cntr, glm::normalize(light.y_axis)) } *
		   light_source_resolution_per_world_unit;
		projection.x = static_cast<int>(
			projctr.x - bs.radius * light_source_resolution_per_world_unit);
		projection.y = static_cast<int>(
			projctr.y - bs.radius * light_source_resolution_per_world_unit);
		projection.width = static_cast<int>(ceil(2.0 * bs.radius * 
			light_source_resolution_per_world_unit));
		projection.height =static_cast<int>(ceil(2.0 * bs.radius *
			light_source_resolution_per_world_unit));
	}
	projection_buffer[projection_idx] = projection;

	// ========================================================================
	// Part 2: calculate projection size
	const unsigned int tid = threadIdx.x;
	temp_num_rays_buffer_ptr[projection_idx] =
		static_cast<size_t>(projection.width * projection.height);
}

__global__ void homogenous_light_bounding_sphere_projection::
	countTotalRaysKernelFirst(
		size_t* temp_num_rays_buffer_ptr,
		size_t num_blocks)
{
	using namespace ldplab;
	using namespace rtscuda;

	// Shared memory
	extern __shared__ size_t sbuf[];

	// ========================================================================
	// Part 1: Load initial sums
	const unsigned int tid = threadIdx.x;
	const unsigned int gid = blockIdx.x * blockDim.x + tid;
	sbuf[tid] = temp_num_rays_buffer_ptr[gid];
	__syncthreads();

	// ========================================================================
	// Part 2: Compute sums
	for (unsigned int i = 1; i < blockDim.x; ++i)
	{
		if (i == tid)
			sbuf[tid] += sbuf[tid - 1];
		__syncthreads();
	}

	// ========================================================================
	// Part 3: Write back results
	temp_num_rays_buffer_ptr[gid] = sbuf[tid];
}

__global__ void homogenous_light_bounding_sphere_projection::
	countTotalRaysKernelSecond(
		size_t* temp_num_rays_buffer_ptr,
		size_t* num_rays_buffer_ptr,
		size_t num_blocks, 
		size_t num_rays_per_batch)
{
	using namespace ldplab;
	using namespace rtscuda;

	// Shared memory
	extern __shared__ size_t sbuf[];

	// ========================================================================
	// Part 1: Load initial sums
	const unsigned int tid = threadIdx.x;
	const unsigned int gid = blockIdx.x * blockDim.x + tid;
	sbuf[tid] = temp_num_rays_buffer_ptr[gid];
	//__syncthreads();

	// ========================================================================
	// Part 2: Compute sums for each block up until this one
	for (unsigned int i = 0; i < blockIdx.x; ++i)
	{
		sbuf[tid] += temp_num_rays_buffer_ptr[(i + 1) * blockDim.x - 1];
		//__syncthreads();
	}

	// ========================================================================
	// Part 3: Write back results
	num_rays_buffer_ptr[gid] = sbuf[tid];
	if (blockIdx.x + 1== num_blocks && tid + 1 == blockDim.x)
		total_num_rays = sbuf[tid];
}

__global__ void homogenous_light_bounding_sphere_projection::
	createBatchKernel(
		int32_t* ray_index_buffer, 
		Vec3* ray_origin_buffer, 
		Vec3* ray_direction_buffer, 
		double* ray_intensity_buffer,
		double* ray_min_bv_dist_buffer, 
		InitialStageHomogenousLightBoundingSphereProjection::Rect* projection_buffer,
		InitialStageHomogenousLightBoundingSphereProjection::HomogenousLightSource* light_buffer,
		size_t* num_rays_buffer,
		size_t num_particles,
		size_t num_light_sources,
		size_t num_rays_per_batch, 
		size_t batch_no)
{
	// ========================================================================
	// Part 1: Find which projection to use for this instance using binary search
	const unsigned int gid =
		batch_no * num_rays_per_batch + blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int ri = blockIdx.x * blockDim.x + threadIdx.x;
	ray_index_buffer[ri] = -1;
	if (gid >= total_num_rays)
		return;
	size_t low = 0;
	size_t high = (num_particles * num_light_sources) - 1;
	size_t proj_idx, nr;
	InitialStageHomogenousLightBoundingSphereProjection::Rect proj;
	do
	{
		proj_idx = (high - low) / 2;
		nr = num_rays_buffer[proj_idx];
		proj = projection_buffer[proj_idx];
		if (gid < nr && gid >= nr - proj.width * proj.height)
			break;
		else if (gid < nr)
			high = proj_idx;
		else if (low < proj_idx)
			low = proj_idx;
		else
			low = proj_idx + 1;
	} while (low < high);

	// ========================================================================
	// Part 2: Find which ray to create
	const unsigned int lid = gid - (nr - proj.width * proj.height);
	const int xid = static_cast<int>(lid) % proj.width;
	const int yid = static_cast<int>(lid) / proj.width;
	if (proj.x + xid < 0 ||
		proj.y + yid < 0)
		return;
	else
	{
		// ====================================================================
		// Part 3: Check if ray is overlapped by other projection
		const size_t light_index = proj_idx % num_light_sources;
		for (size_t i = light_index; i < proj_idx; i += num_particles)
		{
			const auto tproj = projection_buffer[i];
			if (proj.x + xid >= tproj.x &&
				proj.x + xid < tproj.x + tproj.width &&
				proj.y + yid >= tproj.y &&
				proj.y + yid < tproj.y + tproj.height)
				return;
		}

		// Create ray
		InitialStageHomogenousLightBoundingSphereProjection::HomogenousLightSource
			light = light_buffer[light_index];
		ray_index_buffer[ri] = static_cast<int32_t>(num_particles);
		ray_origin_buffer[ri] = light.origin +
			static_cast<double>(proj.x + xid) * light.x_axis +
			static_cast<double>(proj.y + yid) * light.y_axis;
		ray_direction_buffer[ri] = light.ray_direction;
		ray_intensity_buffer[ri] = light.ray_intensity;
		ray_min_bv_dist_buffer[ri] = 0.0;
	}
}

#endif