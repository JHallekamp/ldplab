#ifndef WWU_LDPLAB_RTSCUDA_DATA_DEPRECATED_HPP
#define WWU_LDPLAB_RTSCUDA_DATA_DEPRECATED_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <cstdint>
#include <vector>
#include <LDPLAB/ExperimentalSetup/Particle.hpp>
#include <LDPLAB/Geometry.hpp>
#include <cuda.h>

#include "CudaResource.hpp"
#include "GenericBoundingVolume.hpp"
#include "GenericParticleGeometry.hpp"
#include "GenericParticleMaterial.hpp"
#include "PipelineBoundingVolumeIntersection.hpp"
#include "PipelineInitial.hpp"
#include "PipelineInnerParticlePropagation.hpp"
#include "PipelineParticleInteraction.hpp"
#include "PipelineParticleIntersection.hpp"
#include "PipelineRayBufferReduce.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        /** @brief Holds ray buffer resources. */
        struct RayBufferResources
        {
            /** 
             * @brief Allocates the resources. 
             * @details If no allocation errors occur, this method is 
             *          guaranteed to allocate at least num_buffers ray buffer
             *          in device memory, each holding at least 
             *          num_rays_per_buffer elements. Device pointer to the
             *          allocated memory (per buffer) will be stored in the
             *          vector members of the caller instance.
             *          It will also fill and upload the device arrays holding
             *          said pointers to each individual buffer.
             * @param[in] num_buffers The number of individual ray buffers.
             * @param[in] num_rays_per_buffer The number of individualy rays
             *                                inside each ray buffer.
             * @returns true, if no error occured.
             */
            bool allocateResources(
                size_t num_buffers, 
                size_t num_rays_per_batch);
            /** @brief Index arrays per ray buffer. */
            std::vector<CudaPtr<int32_t>> index_buffers;
            /** @brief Ray origin vectors per ray buffer. */
            std::vector<CudaPtr<Vec3>> origin_buffers;
            /** @brief Ray direction vectors per ray buffer. */
            std::vector<CudaPtr<Vec3>> direction_buffers;
            /** @brief Ray intensity values per ray buffer. */
            std::vector<CudaPtr<double>> intensity_buffers;
            /** @brief Minimum bounding volume distance arrays per buffer. */
            std::vector<CudaPtr<double>> min_bv_dist_buffers;
            /** @brief Array with pointers to the index buffers. */
            CudaPtr<int32_t*> index_buffer_pointers;
            /** @brief Array with pointers to the origin buffers. */
            CudaPtr<Vec3*> origin_buffer_pointers;
            /** @brief Array with pointers to the direction buffers. */
            CudaPtr<Vec3*> direction_buffer_pointers;
            /** @brief Array with pointers to the intensity buffers. */
            CudaPtr<double*> intensity_buffer_pointers;
            /** @brief Array with pointers to the min bv dist buffers. */
            CudaPtr<double*> min_bv_dist_buffer_pointers;
        };

        /** @brief Holds intersection buffer resources. */
        struct IntersectionBufferResources
        {
            /**
             * @brief Allocates the buffer resources.
             * @param[in] num_rays_per_buffer The number of rays per buffer.
             * @returns true, if no error occured.
             */
            bool allocateResources(size_t num_rays_per_batch);
            /** @brief Buffer holding the intersection point per ray. */
            CudaPtr<Vec3> intersection_point_buffer;
            /** @brief Buffer holding the intersection normal per ray. */
            CudaPtr<Vec3> intersection_normal_buffer;
            /** @brief Buffer holding particle intersection indices. */
            CudaPtr<int32_t> intersection_particle_index_buffer;
        };

        /** @brief Holds output buffer resources. */
        struct OutputBufferResources
        {
            /**
             * @brief Allocates the buffer resources.
             * @param[in] num_particles The number of particles in the 
             *                          experimental setup.
             * @param[in] num_rays_per_buffer The number of rays per buffer.
             * @returns true, if no error occured.
             */
            bool allocateResources(
                size_t num_particles,
                size_t num_rays_per_batch);
            /** @brief Host buffer for particle force vectors. */
            std::vector<Vec3> host_force_per_particle;
            /** @brief Host buffer for particle torque vectors. */
            std::vector<Vec3> host_torque_per_particle;
            /** @brief Buffer holding force vectors per particle. */
            CudaPtr<Vec3> force_per_particle;
            /** @brief Buffer holding torque vectors per particles. */
            CudaPtr<Vec3> torque_per_particle;
            /** @brief Buffer holding  force vectors per ray. */
            CudaPtr<Vec3> force_per_ray;
            /** @brief Buffer holding torque vectors per ray. */
            CudaPtr<Vec3> torque_per_ray;
        };

        /** @brief Holds tranformation data resources. */
        struct TransformationResources
        {
            /**
            * @brief Allocates the device memory resources.
            * @param[in] num_particles The number of particles in the
            *                          experimental setup.
            * @returns true, if no error occured.
            */
            bool allocateResource(size_t num_particles);
            /** @brief Host buffer particle to world space transformations. */
            std::vector<Mat3> host_p2w_transformation;
            /** @brief Host buffer particle to world space translation. */
            std::vector<Vec3> host_p2w_translation;
            /** @brief Host buffer world to particle space transformations. */
            std::vector<Mat3> host_w2p_transformation;
            /** @brief Host buffer world to particle space translation. */
            std::vector<Vec3> host_w2p_translation;
            /** @brief Particle to world space transformations per particle. */
            CudaPtr<Mat3> p2w_transformation;
            /** @brief Particle to world space translations per particle. */
            CudaPtr<Vec3> p2w_translation;
            /** @brief World to particle space transformations per particle. */
            CudaPtr<Mat3> w2p_transformation;
            /** @brief Particle to world space translations per particle. */
            CudaPtr<Vec3> w2p_translation;
        };

        /** @brief Holds bounding volume data resources. */
        struct BoundingVolumeResources
        {
            /**
             * @brief Allocates the device memory resources.
             * @param[in] particles Vector of particles in the experimental
             *                      setup.
             * @returns true, if no error occured.
             */
            bool allocateResource(const std::vector<Particle>& particles);
            /** @brief Holding the generic resources. */
            std::vector<std::shared_ptr<GenericBoundingVolume>> bounding_volumes;
            /** @brief Array of generic bounding volume data per particle. */
            CudaPtr<GenericBoundingVolumeData> bounding_volume_per_particle;
        };

        /** @brief Holds particle data. */
        struct ParticleResources
        {
            /**
             * @brief Allocates the device memory resources.
             * @param[in] particles Vector of particles in the experimental
             *                      setup.
             * @returns true, if no error occured.
             */
            bool allocateResource(const std::vector<Particle>& particles);
            /** @brief Array of particle center of masses */
            CudaPtr<Vec3> center_of_mass_per_particle;
            /** @brief Holding the generic particle material resources. */
            std::vector<std::shared_ptr<GenericParticleMaterial>> materials;
            /** @brief Array of material per particle. */
            CudaPtr<GenericParticleMaterialData> material_per_particle;
            /** @brief Holding the generic resources. */
            std::vector<std::shared_ptr<GenericParticleGeometry>> geometries;
            /** @brief Array of geometry per particle. */
            CudaPtr<GenericParticleGeometryData> geometry_per_particle;
        private:
            bool allocateGeometries(const std::vector<Particle>& particles);
            bool allocateMaterials(const std::vector<Particle>& particles);
            bool allocateCenterOfMass(const std::vector<Particle>& particles);
        };

        /** @brief Holds other pipeline resources, like temporary buffers. */
        struct PipelineResources
        {
            /**
            * @brief Allocates the buffer resources.
            * @param[in] num_rays_per_buffer The number of rays per buffer.
            * @param[in] num_threads_per_block Number of threads per cuda block.
            * @returns true, if no error occured.
            */
            bool allocateResources(
                size_t num_rays_per_batch, 
                size_t num_threads_per_block);
            /** @brief Host side array holding reduction results. */
            std::vector<RayBufferReduceResult> host_reduction_result_buffer;
            /** @brief Array holding reduction results. */
            CudaPtr<RayBufferReduceResult> reduction_result_buffer;
        };

        /** @brief Contains kernel launch parameters. */
        struct KernelLaunchParameter
        {
            KernelLaunchParameter() : 
                grid_size{ 1, 1, 1 }, 
                block_size{ 1, 1, 1 }, 
                shared_memory_size{ 0 } 
            { }
            dim3 grid_size;
            dim3 block_size;
            unsigned int shared_memory_size;
        };

        /** @brief Resources on device. */
        struct DevicePipelineResources
        {
            // --------------------------------
            struct {
                int32_t** indices;
                Vec3** origins;
                Vec3** directions;
                double** intensities;
                double** min_bv_dists;
            } ray_buffer;
            // --------------------------------
            struct {
                Vec3* points;
                Vec3* normals;
                int32_t* isec_indices;
            } intersection_buffer;
            // --------------------------------
            struct {
                Vec3* force_per_ray;
                Vec3* torque_per_ray;
                Vec3* force_per_particle;
                Vec3* torque_per_particle;
            } output_buffer;
            // --------------------------------
            struct {
                Mat3* p2w_transformation;
                Vec3* p2w_translation;
                Mat3* w2p_transformation;
                Vec3* w2p_translation;
            } transformations;
            // --------------------------------
            struct {
                GenericBoundingVolumeData* per_particle;
            } bounding_volumes;
            // --------------------------------
            struct {
                Vec3* center_of_mass_per_particle;
                GenericParticleMaterialData* material_per_particle;
                GenericParticleGeometryData* geometry_per_particle;
            } particles;
            // --------------------------------
            struct {
                RayBufferReduceResult* reduction_result_buffer;
            } reduction;
            // --------------------------------
            struct {
                double intensity_cutoff;
                double light_source_resolution_per_world_unit;
                double medium_reflection_index;
                size_t max_branching_depth;
                size_t num_light_sources;
                size_t num_rays_per_batch;
                size_t num_particles;
                bool output_in_particle_space;
            } parameters;
            // --------------------------------
            struct {
                KernelLaunchParameter boundingVolumeIntersection;
                KernelLaunchParameter initialBufferSetup;
                KernelLaunchParameter bufferSetup;
                KernelLaunchParameter gatherOutput;
                KernelLaunchParameter createBatch;
                KernelLaunchParameter innerParticlePropagation;
                KernelLaunchParameter particleInteraction;
                KernelLaunchParameter particleIntersection;
                KernelLaunchParameter rayBufferReduceStep1;
                KernelLaunchParameter rayBufferReduceStep2;
            } launch_params;
        };

        /** @brief Contains pipeline stage kernel execution function pointer. */
        struct PipelineExecuteFunctions
        {
            pipelineExecuteBoundingVolumeIntersectionStage_t bounding_volume_intersection;
            pipelineExecuteInitialStage_t initial;
            pipelineExecuteInnerParticlePropagationStage_t inner_particle_propagation;
            pipelineExecuteParticleInteractionStage_t particle_interaction;
            pipelineExecuteParticleIntersectionStage_t particle_intersection;
        };
    }
}

#endif // LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#endif // WWU_LDPLAB_RTSCUDA_DATA_HPP