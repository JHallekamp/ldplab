#ifndef WWU_LDPLAB_RTSCUDA_PARALLEL_CONTEXT_HPP
#define WWU_LDPLAB_RTSCUDA_PARALLEL_CONTEXT_HPP

#include <cuda_runtime.h>
#include <vector>

namespace ldplab
{
    // Prototype    
    struct ExperimentalSetup;
     
    namespace rtscuda
    {
        enum class PipelineStageType
        {
            bounding_volume_intersection,
            initial_stage,
            inner_particle_propagation,
            particle_intersection,
            surface_interaction
        };

        // Prototypes
        struct InterfaceMapping;
        struct RayDataBuffers;
        struct IntersectionDataBuffers;
        struct OutputDataBuffers;
        struct SimulationParameter;
        struct ParticleTransformationBuffers;
        struct ParticleDataBuffers;
        struct DeviceProperties;
        struct DeviceData;
        struct StreamData;

        /** @brief Context for a device group. */
        class DeviceContext
        {
            friend struct SharedStepData;
        public:
            size_t groupId() const { return m_group_id; }
            const std::vector<size_t>& streams() const { return m_stream_ids; }
            size_t numStreams() const { return streams().size(); }

            const DeviceProperties& deviceProperties() const { return m_device_properties; }
            int deviceId() const { return m_device_id; }
            bool activateDevice() const;
            bool synchronizeOnDevice() const;
        private:
            DeviceContext(
                size_t group_id,
                DeviceProperties& device_properties,
                std::vector<size_t>&& stream_ids);
        private:
            size_t m_group_id;
            int m_device_id;
            DeviceProperties& m_device_properties;
            std::vector<size_t> m_stream_ids;
        };

        /**
         * @brief Context object used to coordinate parallel executions on
         *        multiple host threads and parallel cuda devices and/or 
         *        streams.
         * @details The LDPLAB RTSCUDA execution model consists of streams,
         *          device groups and execution groups. These concepts 
         *          determine how and where resources are allocated, used and
         *          shared. The execution model allows to execute multiple 
         *          pipeline runs in parallel and therefore reach better device
         *          workload, then a single pipeline executing one batch at a
         *          time would achieve.
         *          A stream can be viewed as a single "thread" of sorts, that
         *          executes the pipeline by sequentially launching cuda 
         *          kernels for one batch at a time.
         *          A device group is a group of streams that are executed on
         *          the same device. If only one device is present, then all
         *          streams therefore share the same device group.
         */
        class StreamContext
        {
            friend struct SharedStepData;
        public:
            ~StreamContext();
            size_t streamId() const { return m_stream_id; }
            size_t deviceGroup() const { return m_device_context.groupId(); }
            const DeviceContext& deviceContext() const { return m_device_context; }

            cudaStream_t cudaStream() const { return m_cuda_stream; }
            bool synchronizeOnStream();

            RayDataBuffers& rayDataBuffers() const { return m_ray_data_buffers; }
            IntersectionDataBuffers& intersectionDataBuffers() const { return m_intersection_data_buffers; }
            OutputDataBuffers& outputDataBuffers() const { return m_output_data_buffers; }

            const ExperimentalSetup& experimentalSetup() const { return m_experimental_setup; }
            const InterfaceMapping& interfaceMapping() const { return m_interface_mapping; }
            const SimulationParameter& simulationParameter() const { return m_simulation_parameter; }
            const ParticleTransformationBuffers& particleTransformationBuffers() const { return m_particle_transformation_buffers; }
            const ParticleDataBuffers& particleDataBuffers() const { return m_particle_data_buffers; }
            const DeviceProperties& deviceProperties() const { return m_device_properties; }
        private:
            StreamContext(
                const DeviceContext& device_ctx,
                const DeviceData& device_data,
                const ExperimentalSetup& experimental_setup,
                const InterfaceMapping& interface_mapping,
                const SimulationParameter& simulation_parameter,
                StreamData& stream_data);
            bool allocate();
        private:
            size_t m_stream_id;
            cudaStream_t m_cuda_stream;

            RayDataBuffers& m_ray_data_buffers;
            IntersectionDataBuffers& m_intersection_data_buffers;
            OutputDataBuffers& m_output_data_buffers;

            const ExperimentalSetup& m_experimental_setup;
            const InterfaceMapping& m_interface_mapping;
            const SimulationParameter& m_simulation_parameter;
            const ParticleTransformationBuffers& m_particle_transformation_buffers;
            const ParticleDataBuffers& m_particle_data_buffers;
            const DeviceProperties& m_device_properties;

            const DeviceContext& m_device_context;
        };
    }
}

#endif