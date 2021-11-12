#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include <LDPLAB/RayTracingStep/CUDA/ParallelContext.hpp>
#include <LDPLAB/RayTracingStep/CUDA/Data.hpp>

#include "../../Utils/Log.hpp"

ldplab::rtscuda::StreamContext::CudaStreamWrapper::~CudaStreamWrapper()
{
	if (stream != 0)
	{
		cudaStreamDestroy(stream);
		stream = 0;
	}
}

ldplab::rtscuda::StreamContext::StreamContext(
	std::vector<DeviceContext>* device_ctx,
	const size_t device_group_id,
	const DeviceData& device_data,
	const ExperimentalSetup& experimental_setup,
	const InterfaceMapping& interface_mapping,
	const SimulationParameter& simulation_parameter,
	StreamData& stream_data)
	:
	m_stream_id{ stream_data.associated_stream },
	m_device_contexts{ device_ctx },
	m_device_group_id{ device_group_id },
	m_ray_data_buffers{ stream_data.ray_data_buffers },
	m_intersection_data_buffers{ stream_data.intersection_data_buffers },
	m_output_data_buffers{ stream_data.output_data_buffers },
	m_experimental_setup{ experimental_setup },
	m_interface_mapping{ interface_mapping },
	m_simulation_parameter{ simulation_parameter },
	m_particle_transformation_buffers{ device_data.particle_transformation_buffers },
	m_particle_data_buffers{ device_data.particle_data_buffers },
	m_device_properties{ device_data.device_properties },
	m_cuda_stream{ new CudaStreamWrapper() }
{ 
}

bool ldplab::rtscuda::StreamContext::synchronizeOnStream()
{
	return cudaStreamSynchronize(m_cuda_stream->stream) == cudaSuccess;
}

bool ldplab::rtscuda::StreamContext::allocate()
{
	cudaError_t err = cudaStreamCreate(&m_cuda_stream->stream);
	if (err != cudaSuccess)
	{
		LDPLAB_LOG_ERROR("RTSCUDA stream context %i: Failed to create "\
			"cuda stream, cuda returned error code %i: %s", 
			m_stream_id, 
			err,
			cudaGetErrorString(err));
		return false;
	}
	return true;
}

bool ldplab::rtscuda::DeviceContext::activateDevice() const
{
	LDPLAB_LOG_DEBUG("RTSCUDA device context %i: Set cuda device to %i",
		m_group_id,
		m_device_id);
	cudaError_t err = cudaSetDevice(m_device_id);
	if (err != cudaSuccess)
	{
		LDPLAB_LOG_ERROR("RTSCUDA device context %i: Failed to set "\
			"cuda device, cuda returned error code %i: %s",
			m_group_id,
			err,
			cudaGetErrorString(err));
		return false;
	}
	return true;
}

bool ldplab::rtscuda::DeviceContext::synchronizeOnDevice() const
{
	return cudaDeviceSynchronize() == cudaSuccess;
}

ldplab::rtscuda::DeviceContext::DeviceContext(
	size_t group_id,
	DeviceProperties& device_properties,
	std::vector<size_t>&& stream_ids)
	:
	m_group_id{ group_id },
	m_device_id{ device_properties.id },
	m_device_properties{ device_properties },
	m_stream_ids{ std::move(stream_ids) }
{ }

#endif