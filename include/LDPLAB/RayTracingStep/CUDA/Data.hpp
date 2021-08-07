#ifndef WWU_LDPLAB_RTSCUDA_DATA_HPP
#define WWU_LDPLAB_RTSCUDA_DATA_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <cstdint>
#include <map>
#include <memory>
#include <vector>

#include <LDPLAB/ExperimentalSetup/Particle.hpp>
#include <LDPLAB/Geometry.hpp>
#include <LDPLAB/UID.hpp>

#include "DeviceResource.hpp"
#include "IGenericGeometry.hpp"
#include "IGenericMaterial.hpp"

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
         * @brief Contains host structures for the device data that accompanies
         *        an individual batch.
         * @details Due to the potential parallel computation of multiple 
         *          batches (to achieve better workloads), there can be 
         *          multiple instances of this structure.
         *          The memory interface of LDPLAB rtscuda is more verbose by
         *          default than its rtscpu counterpart: While the pipeline is
         *          still supposed to own the memory, each individual stage
         *          receives total access to all present data. This is done to
         *          increase optimization potential.
         */
        struct BatchData
        {
            /** @brief Global index of the batch data instance. */
            size_t batch_data_index;
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
            } ray_data_buffers;
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
            } intersection_data_buffers;
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
                DeviceBuffer<Vec3> force_per_ray_buffer;
                DeviceBuffer<Vec3> torque_per_particle_buffer;
                DeviceBuffer<Vec3> torque_per_ray_buffer;
            } output_data_buffers;
        };

        /** 
         * @brief Contains batch independent data for both host and device 
         *        resources. 
         * @details Each pipeline stage receives a pointer to the global data
         *          instance of the pipeline.
         */
        struct GlobalData
        {
            const UID<GlobalData> instance_uid;
            /** @brief Contains batch data instances for each parallel batch. */
            std::vector<BatchData> batch_data;
            /** @brief The experimental setup used to create the rts. */
            ExperimentalSetup experimental_setup;
            /** @brief Interface mapping between step and caller. */
            InterfaceMapping interface_mapping;
            /** @brief Information about the simulation. */
            struct SimulationParameter
            {
                size_t max_branching_depth;
                size_t num_parallel_batches;
                size_t num_particles;
                size_t num_rays_per_batch;
                size_t num_surface_interaction_reflection_passes;
                size_t num_surface_interaction_transmission_passes;
                int32_t ray_world_space_index;
                int32_t ray_invalid_index = -1;
            } simulation_parameter;
            /** @brief Particle data. */
            struct ParticleDataBuffers
            {
                DeviceBuffer<Mat3> p2w_transformation_buffer;
                DeviceBuffer<Vec3> p2w_translation_buffer;
                DeviceBuffer<Mat3> w2p_transformation_buffer;
                DeviceBuffer<Vec3> w2p_translation_buffer;
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
            } particle_data_buffers;
            /** @brief Device property data. */
            struct DeviceProperties
            {
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
            } device_properties;
        };
    }
}

#endif
#endif