#ifndef WWU_LDPLAB_RTSCUDA_DATA_HPP
#define WWU_LDPLAB_RTSCUDA_DATA_HPP

#include <cstdint>
#include <map>
#include <memory>
#include <vector>

#include <LDPLAB/ExperimentalSetup/ExperimentalSetup.hpp>
#include <LDPLAB/Geometry.hpp>
#include <LDPLAB/UID.hpp>
#include <LDPLAB/RayTracingStep/RayTracingStepCUDAInfo.hpp>
#include <LDPLAB/RayTracingStep/CUDA/PipelineConfiguration.hpp>

#include "DeviceResource.hpp"
#include "ExecutionModelInfo.hpp"
#include "IGenericGeometry.hpp"
#include "IGenericMaterial.hpp"
#include "ParallelContext.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        /**
         * @brief Holds mapping between internal indices and outward uids for
         *        various experimental setup components.
         * @details Components that exist outside of the ray tracing step,
         *          like for example particles, are identified by different
         *          simulation steps - like the ray tracing step - using their
         *          respective UID.
         *          Internally however the ray tracing step uses an index based
         *          identification to access and identify components directly
         *          and therefore more efficiently.
         *          Therefore the ray tracing step needs mappings between
         *          simulation wide UIDs and internally used indices for all
         *          relevant components, which are provided inside this
         *          structure.
         */
        struct InterfaceMapping
        {
            std::map<UID<Particle>, size_t> particle_uid_to_index;
            std::map<size_t, UID<Particle>> particle_index_to_uid;
        };

        /**
         * @brief Contains all device buffers used for storing ray data
         *        during pipeline execution.
         * @details Each buffer contains N data points, where N is the
         *          number of rays per batch.
         *          Each buffer range contains MB+2 buffers, where MB is
         *          the maximum branching depth (one for each branching
         *          layer plus one initial and one final dummy buffer).
         * @note The DeviceBufferRange does not provide host buffers for
         *       ray data.
         */
        struct RayDataBuffers
        {
            DeviceBufferRange<int32_t> particle_index_buffers;
            DeviceBufferRange<Vec3> origin_buffers;
            DeviceBufferRange<Vec3> direction_buffers;
            DeviceBufferRange<double> intensity_buffers;
            DeviceBufferRange<double> min_bv_distance_buffers;
        };

        /**
         * @brief Contains all device buffers used for storing intersection
         *        data during pipeline execution.
         * @details Each buffer contains N data points, where N is the
         *          number of rays per batch.
         *          Each buffer range contains MB+1 buffers, where MB is
         *          the maximum branching depth (one for each branching
         *          layer plus one initial).
         * @note The DeviceBufferRange does not provide host buffers for
         *       intersection data.
         */
        struct IntersectionDataBuffers
        {
            DeviceBufferRange<Vec3> point_buffers;
            DeviceBufferRange<Vec3> normal_buffers;
            DeviceBufferRange<int32_t> particle_index_buffers;
        };

        /**
         * @brief Contains all device buffers used for storing the
         *        collected output data during pipeline execution.
         * @note The per particle buffer variants have a host memory buffer
         *       attached, so that the final resulting data can be
         *       downloaded.
         */
        struct OutputDataBuffers
        {
            DeviceBuffer<Vec3> force_per_particle_buffer;
            DeviceBufferRange<Vec3> force_per_ray_buffer;
            DeviceBuffer<Vec3> torque_per_particle_buffer;
            DeviceBufferRange<Vec3> torque_per_ray_buffer;
        };

        /** @brief Information about the simulation. */
        struct SimulationParameter
        {
            double buffer_reorder_threshold;
            size_t buffer_min_size;
            double intensity_cutoff;
            size_t max_branching_depth;
            size_t num_particles;
            size_t num_rays_per_batch;
            size_t num_surface_interaction_reflection_passes;
            size_t num_surface_interaction_transmission_passes;
            int32_t ray_world_space_index;
            int32_t ray_invalid_index = -1;
            bool output_in_particle_space;
            bool sort_buffer_inner_particle_pass;
            bool sort_buffer_outer_particle_pass;
        };

        /** @brief Particle transformations. */
        struct ParticleTransformationBuffers
        {
            DeviceBuffer<Mat3> p2w_transformation_buffer;
            DeviceBuffer<Vec3> p2w_translation_buffer;
            DeviceBuffer<Mat3> w2p_transformation_buffer;
            DeviceBuffer<Vec3> w2p_translation_buffer;
        };

        /** @brief Particle data. */
        struct ParticleDataBuffers
        {
            DeviceBuffer<Vec3> center_of_mass_buffer;
            DeviceBuffer<void*> geometry_data_buffer;
            DeviceBuffer<void*> material_data_buffer;
            DeviceBuffer<IGenericGeometry::intersectRay>
                intersect_ray_fptr_buffer;
            DeviceBuffer<IGenericGeometry::intersectSegment>
                intersect_segment_fptr_buffer;
            DeviceBuffer<IGenericMaterial::indexOfRefraction>
                index_of_refraction_fptr_buffer;
            std::vector<std::shared_ptr<IGenericGeometry>> geometry_instances;
            std::vector<std::shared_ptr<IGenericMaterial>> material_instances;
        };

        /** @brief Device property data. */
        struct DeviceProperties
        {
            /** @brief The id of the device. */
            int id;
            /** @brief The name of the device. */
            std::string name;
            /** @brief Maximum dimension size of a thread block. */
            dim3 max_block_size;
            /** @brief Maximum dimension size of a grid. */
            dim3 max_grid_size;
            /** @brief Maximum number of threads per thread block. */
            unsigned int max_num_threads_per_block;
            /** @brief Maximum number of threads per multiprocessor. */
            unsigned int max_num_threads_per_mp;
            /** @brief Total shared memory amount per block. */
            unsigned int shared_mem_per_block;
            /** @brief Total shared memory amount per multiprocessor. */
            unsigned int shared_mem_per_mp;
            /** @brief The total number of abailable of registers per block. */
            unsigned int registers_per_block;
            /** @brief Number of multiprocessors on the device. */
            unsigned int num_mps;
            /** @brief Number of threads in a warp. */
            unsigned int warp_size;
        };

        /** @brief Holds the execution model. */
        struct ExecutionModel
        {
            friend class Factory;
        public:
            std::vector<StreamContext> stream_contexts;
            std::vector<DeviceContext> device_contexts;
        };

        // Prototype
        struct DeviceData;
        struct ExecutionData;
        struct StreamData;

        /** @brief Contains the shared ray tracing step data. */
        struct SharedStepData
        {
            friend class Factory;
        private:
            bool allocateResources(
                const RayTracingStepCUDAInfo& info,
                PipelineConfiguration& pipeline_config,
                ExperimentalSetup&& setup,
                InterfaceMapping&& interface_mapping);
            bool allocateDeviceData(
                const RayTracingStepCUDAInfo& info,
                DeviceData& device_data,
                PipelineConfiguration& pipeline_config);
            bool allocateStreamData(
                StreamData& stream_data);
            bool createExecutionModel(
                std::shared_ptr<IExecutionModelInfo> info);
            bool createExecutionDataAuto(
                ExecutionModelAutoConstructionInfo& info);
            bool createExecutionDataExplicit(
                ExecutionModelExplicitConfigInfo& info);
        public:
            /** @brief Shared data instance uid. */
            const UID<SharedStepData> instance_uid;
            /** @brief The experimental setup used to create the rts. */
            ExperimentalSetup experimental_setup;
            /** @brief Interface mapping between step and caller. */
            InterfaceMapping interface_mapping;
            /** @brief The parameter of the simulation. */
            SimulationParameter simulation_parameter;
            /** @brief The execution model. */
            ExecutionModel execution_model;
            /** @brief Contains data on the devices. */
            std::vector<DeviceData> per_device_data;
        };

        struct DeviceData
        {
            /** @brief Device group id. */
            size_t associated_device_group;
            /** @brief The device properties. */
            DeviceProperties device_properties;
            /** @brief Particle buffers. */
            ParticleDataBuffers particle_data_buffers;
            /** @brief Particle transformation data. */
            ParticleTransformationBuffers particle_transformation_buffers;
            /** @brief Maps stream id to per stream data index. */
            std::map<size_t, size_t> stream_id_to_on_device_index;
            /** @brief Data per stream. */
            std::vector<StreamData> per_stream_data;
        };

        struct StreamData
        {
            /** @brief Stream id. */
            size_t associated_stream;
            RayDataBuffers ray_data_buffers;
            IntersectionDataBuffers intersection_data_buffers;
            OutputDataBuffers output_data_buffers;
        };
    }
}

#endif