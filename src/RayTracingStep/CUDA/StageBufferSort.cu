#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "StageBufferSort.hpp"

#include <unordered_set>

namespace
{
	__global__ void buildLocalRank(
		int32_t* ray_index,
		uint32_t* local_rank,
		uint32_t* block_size,
		size_t active_rays,
		size_t num_rays)
	{
		// Shared memory
		extern __shared__ uint32_t sbuf[];

		const unsigned int tid = threadIdx.x;
		const unsigned int gi = blockIdx.x * blockDim.x + tid;

		// Setup local shared buffer
		int32_t pi = -1;
		if (gi < num_rays)
			pi = ray_index[gi];

		if (gi >= active_rays)
			sbuf[tid] = pi < 0 ? 0 : 1;
		else
			sbuf[tid] = pi < 0 ? 1 : 0;
		__syncthreads();

		// Process block differently based on its position
		const unsigned int block_offset = blockIdx.x * blockDim.x;
		uint32_t rank = sbuf[tid];
		if (block_offset + blockDim.x <= active_rays || block_offset >= active_rays)
		{
			for (unsigned int i = 0; i < tid; ++i)
				rank += sbuf[i];
			// Write back size
			const unsigned int block_idx = blockIdx.x + (block_offset >= active_rays ? 1 : 0);
			if (tid == blockDim.x - 1)
				block_size[block_idx] = rank;
		}
		else
		{
			unsigned int i = 0;
			if (gi >= active_rays)
				i = active_rays - block_offset;
			for (; i < tid; ++i)
				rank += sbuf[i];
			// Write back size
			const unsigned int block_idx = blockIdx.x + (gi >= active_rays ? 1 : 0);
			if (tid == blockDim.x - 1 || tid == active_rays - block_offset - 1)
				block_size[block_idx] = rank;
		}

		// Write back rank
		if (gi < num_rays)
			local_rank[gi] = sbuf[tid] * rank;
	}

	__global__ void buildGlobalRank(
		uint32_t* local_rank,
		uint32_t* block_size,
		uint32_t* rank_index_range,
		size_t active_rays,
		size_t num_rays)
	{
		const unsigned int gi = blockIdx.x * blockDim.x + threadIdx.x;

		if (gi >= num_rays)
			return;
		
		// Reset range
		rank_index_range[gi] = 0;

		// Get local rank
		unsigned int rank = local_rank[gi];
		if (rank == 0)
			return;

		// Compute global rank
		unsigned int i = 0;
		if (gi >= active_rays)
			i = active_rays / blockDim.x + (active_rays % blockDim.x ? 1 : 0);
		const unsigned int lim = blockIdx.x + (gi >= active_rays ? 1 : 0);
		for (; i < lim; ++i)
			rank += block_size[i];
		
		// Write back index or global rank
		if (gi < active_rays)
			rank_index_range[rank - 1] = gi;
		else
			rank_index_range[gi] = rank;
	}

	__global__ void sortBuffer(
		uint32_t* rank_index_range,
		size_t active_rays,
		size_t num_rays,
		int32_t* ray_buffer_particle_index,
		ldplab::Vec3* ray_buffer_origin,
		ldplab::Vec3* ray_buffer_direction,
		double* ray_buffer_intensity,
		double* ray_buffer_min_bv_dist,
		int32_t* intersection_buffer_index,
		ldplab::Vec3* intersection_buffer_point,
		ldplab::Vec3* intersection_buffer_normal,
		ldplab::Vec3* output_buffer_force_per_ray,
		ldplab::Vec3* output_buffer_torque_per_ray)
	{
		using namespace ldplab;
		using namespace rtscuda;

		const unsigned int gi =
			blockIdx.x * blockDim.x + threadIdx.x + active_rays;

		if (gi >= num_rays)
			return;
		const int32_t rank = rank_index_range[gi];
		if (rank == 0)
			return;
		const int32_t target_idx = rank_index_range[rank - 1];

		// Move everything except output buffer
		ray_buffer_particle_index[target_idx] = ray_buffer_particle_index[gi];
		ray_buffer_origin[target_idx] = ray_buffer_origin[gi];
		ray_buffer_direction[target_idx] = ray_buffer_direction[gi];
		ray_buffer_intensity[target_idx] = ray_buffer_intensity[gi];
		ray_buffer_min_bv_dist[target_idx] = ray_buffer_min_bv_dist[gi];
		intersection_buffer_index[target_idx] = intersection_buffer_index[gi];
		intersection_buffer_point[target_idx] = intersection_buffer_point[gi];
		intersection_buffer_normal[target_idx] = intersection_buffer_normal[gi];
		output_buffer_force_per_ray[target_idx] = output_buffer_force_per_ray[gi];
		output_buffer_torque_per_ray[target_idx] = output_buffer_torque_per_ray[gi];
	}
}

size_t ldplab::rtscuda::BufferSort::execute(
	StreamContext& stream_context, 
	PipelineData& pipeline_data,
	size_t buffer_index,
	size_t active_rays,
	size_t prev_active_rays)
{
	const size_t block_size = 256;
	const size_t grid_size =
		prev_active_rays / block_size + (prev_active_rays % block_size ? 1 : 0);
	const size_t mem_size = block_size * sizeof(uint32_t);
	buildLocalRank<<<grid_size, block_size, mem_size, stream_context.cudaStream()>>>(
		stream_context.rayDataBuffers().particle_index_buffers.getDeviceBuffer(buffer_index),
		pipeline_data.buffer_sort_local_rank.getDeviceBuffer(),
		pipeline_data.buffer_sort_block_size.getDeviceBuffer(),
		active_rays,
		prev_active_rays);
	buildGlobalRank<<<grid_size, block_size, 0, stream_context.cudaStream()>>>(
		pipeline_data.buffer_sort_local_rank.getDeviceBuffer(),
		pipeline_data.buffer_sort_block_size.getDeviceBuffer(),
		pipeline_data.buffer_sort_rank_index_range.getDeviceBuffer(),
		active_rays,
		prev_active_rays);
	
	//pipeline_data.buffer_sort_block_size.download();
	//pipeline_data.buffer_sort_local_rank.download();
	//pipeline_data.buffer_sort_rank_index_range.download();
	//stream_context.rayDataBuffers().particle_index_buffers.download(0, buffer_index);
	//stream_context.rayDataBuffers().origin_buffers.download(0, buffer_index);
	//stream_context.intersectionDataBuffers().point_buffers.download(0, buffer_index);
	//stream_context.outputDataBuffers().force_per_ray_buffer.download(0, buffer_index);
	//bool breakp1 = true;
	//auto vec3_to_string = [&](const Vec3& v) {
	//	std::string ret;
	//	const char* bytes = reinterpret_cast<const char*>(&v);
	//	for (size_t i = 0; i < sizeof(Vec3); ++i)
	//		ret += bytes[i];
	//	return ret;
	//};
	//std::unordered_set <std::string> known_origins;
	//for (size_t i = 0; i < prev_active_rays; ++i)
	//{
	//	if (stream_context.rayDataBuffers().particle_index_buffers.getHostBuffer(0)[i] >= 0)
	//		known_origins.insert(vec3_to_string(stream_context.rayDataBuffers().origin_buffers.getHostBuffer(0)[i]));
	//}
	
	const size_t inactive_region_sz = prev_active_rays - active_rays;
	const size_t grid_size2 = 
		inactive_region_sz / block_size + (inactive_region_sz % block_size ? 1 : 0);
	sortBuffer<<<grid_size2, block_size, 0, stream_context.cudaStream()>>>(
		pipeline_data.buffer_sort_rank_index_range.getDeviceBuffer(),
		active_rays,
		prev_active_rays,
		stream_context.rayDataBuffers().particle_index_buffers.getDeviceBuffer(buffer_index),
		stream_context.rayDataBuffers().origin_buffers.getDeviceBuffer(buffer_index),
		stream_context.rayDataBuffers().direction_buffers.getDeviceBuffer(buffer_index),
		stream_context.rayDataBuffers().intensity_buffers.getDeviceBuffer(buffer_index),
		stream_context.rayDataBuffers().min_bv_distance_buffers.getDeviceBuffer(buffer_index),
		stream_context.intersectionDataBuffers().particle_index_buffers.getDeviceBuffer(buffer_index),
		stream_context.intersectionDataBuffers().point_buffers.getDeviceBuffer(buffer_index),
		stream_context.intersectionDataBuffers().normal_buffers.getDeviceBuffer(buffer_index),
		stream_context.outputDataBuffers().force_per_ray_buffer.getDeviceBuffer(buffer_index),
		stream_context.outputDataBuffers().torque_per_ray_buffer.getDeviceBuffer(buffer_index));
	
	//pipeline_data.buffer_sort_block_size.download();
	//pipeline_data.buffer_sort_local_rank.download();
	//pipeline_data.buffer_sort_rank_index_range.download();
	//stream_context.rayDataBuffers().particle_index_buffers.download(0, buffer_index);
	//stream_context.rayDataBuffers().origin_buffers.download(0, buffer_index);
	//stream_context.intersectionDataBuffers().point_buffers.download(0, buffer_index);
	//stream_context.outputDataBuffers().force_per_ray_buffer.download(0, buffer_index);
	//bool breakp2 = true;
	//for (size_t i = 0; i < active_rays; ++i)
	//{
	//	if (stream_context.rayDataBuffers().particle_index_buffers.getHostBuffer(0)[i] >= 0)
	//	{
	//		auto it = known_origins.find(
	//			vec3_to_string(stream_context.rayDataBuffers().origin_buffers.getHostBuffer(0)[i]));
	//		if (it == known_origins.end())
	//			printf("UNKNOWN ORIGIN!!!!!!!\n");
	//	}
	//}
	//for (auto it = known_origins.begin(); it != known_origins.end(); ++it)
	//{
	//	size_t found = 0;
	//	for (size_t i = 0; i < active_rays; ++i)
	//	{
	//		if (vec3_to_string(stream_context.rayDataBuffers().origin_buffers.getHostBuffer(0)[i]) == *it)
	//			++found;
	//	}
	//	if (found != 1)
	//		printf("SOMETHING WENT WRONG!!!!!! found: %i\n", found);
	//}
	
	return active_rays;
}

bool ldplab::rtscuda::BufferSort::allocateData(
	const SharedStepData& shared_data, 
	PipelineData& data)
{
#ifdef NDEBUG
	constexpr bool downloadable = false;
#else 
	constexpr bool downloadable = true;
#endif
	bool result = true;

	const size_t bufsz = shared_data.simulation_parameter.num_rays_per_batch;
	result = result && data.buffer_sort_local_rank.allocate(bufsz, downloadable);
	result = result && data.buffer_sort_rank_index_range.allocate(bufsz, downloadable);

	const size_t num_blocks = bufsz / 256 + (bufsz % 256 ? 1 : 0) + 1;
	result = result && data.buffer_sort_block_size.allocate(num_blocks, downloadable);

	return result;
}

#endif

