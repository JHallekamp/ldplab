#ifndef WWU_LDPLAB_RTSCUDA_PIPELINE_BOUNDING_VOLUME_INTERSECTION_HPP
#define WWU_LDPLAB_RTSCUDA_PIPELINE_BOUNDING_VOLUME_INTERSECTION_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <cuda_runtime.h>
#include <LDPLAB/Geometry.hpp>
#include <LDPLAB/RayTracingStep/RayTracingStepCUDAInfo.hpp>
#include <memory>

#include "GenericBoundingVolume.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        // Prototype
        struct Context;

        /**
         * @brief Typedefinition of ray bounding volume intersection stage.
         * @param[in] ray_index_buffer Buffer containing ray indices.
         * @param[in] ray_origin_buffer Buffer containing the ray origins.
         * @param[in] ray_direction_buffer Buffer containing the ray directions.
         * @param[in] ray_min_bv_dist_buffer Buffer containing the minimum
         *                                   distance to the last hit bounding
         *                                   sphere from the ray origin.
         * @param[in] num_rays Number of rays in the given ray buffers.
         * @param[in] bounding_volumes Buffer containing the generic bounding
         *                             volume data.
         * @param[in] w2p_transformation Buffer holding world to particle space
         *                               tranformation matrices.
         * @param[in] w2p_translation Buffer holding world to particle space
         *                            translation vectors.
         * @param[in] num_particles Number of particles in the experimental
         *                          setup.
         */
        typedef void (*pipelineBoundingVolumeIntersectionStageKernel_t)(
            int32_t* ray_index_buffer,
            Vec3* ray_origin_buffer,
            Vec3* ray_direction_buffer,
            double* ray_min_bv_dist_buffer,
            size_t num_rays_per_batch,
            GenericBoundingVolumeData* bounding_volumes,
            Mat3* w2p_transformation,
            Vec3* w2p_translation,
            size_t num_particles);

        /** 
         * @brief Abstract baseclass for bounding volume intersection test 
         *        pipeline stage.
         */
        class IPipelineBoundingVolumeIntersectionStage
        {
        public:
            virtual ~IPipelineBoundingVolumeIntersectionStage() { }
            /** @brief Creates an instance of a pipeline stage implementation. */
            static std::shared_ptr<IPipelineBoundingVolumeIntersectionStage>
                createInstance(
                    const RayTracingStepCUDAInfo& info, Context& context);
            /** @brief Gets called before the pipeline enters execution. */
            virtual void setup() { }
            /** @brief Provides the caller with a pointer to the kernel. */
            virtual pipelineBoundingVolumeIntersectionStageKernel_t
                getKernel() = 0;
            /** 
             * @brief Performs the ray bounding volume intersection test stage
             *        in the host bound pipeline.
             * @details Rays that are invalid (negative particle index) or are
             *          transformed into particle space are ignored by this 
             *          stage. Only rays in world space (positive particle
             *          index greater the total number of particles) are tested
             *          for intersections with bounding volumes.
             *          If a ray intersects a particle bounding volume then it
             *          is transformed to its particles local space and the
             *          particle index is set accordingly. Otherwise, if no
             *          particle bounding volume is intersected, the ray is
             *          set to be invalid.
             */
            virtual void execute(size_t ray_buffer_index) = 0;
        };

        /** 
         * @brief Bounding volume intersection stage bruteforcing the test by
         *        iterating over every bounding volume.
         */
        class PipelineBoundingVolumeIntersectionBruteforce :
            public IPipelineBoundingVolumeIntersectionStage
        {
        public:
            PipelineBoundingVolumeIntersectionBruteforce(Context& context);
            void execute(size_t ray_buffer_index) override;
            pipelineBoundingVolumeIntersectionStageKernel_t 
                getKernel() override;
        private:
            /** @brief Bounding volume intersection device kernel. */
            static __global__ void bvIntersectionKernel(
                int32_t* ray_index_buffer,
                Vec3* ray_origin_buffer,
                Vec3* ray_direction_buffer,
                double* ray_min_bv_dist_buffer,
                size_t num_rays_per_batch,
                GenericBoundingVolumeData* bounding_volumes,
                Mat3* w2p_transformation,
                Vec3* w2p_translation,
                size_t num_particles);
            /** @brief Device function pointer to the actual kernel. */
            static __device__ pipelineBoundingVolumeIntersectionStageKernel_t
                bv_intersection_kernel_ptr;
        private:
            Context& m_context;
        };
    }
}

#endif // LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#endif // WWU_LDPLAB_RTSCUDA_PIPELEINE_BOUNDING_VOLUME_INTERSECTION_HPP