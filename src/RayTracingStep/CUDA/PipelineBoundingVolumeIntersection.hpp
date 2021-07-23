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
        struct KernelLaunchParameter;
        struct DevicePipelineResources;

        typedef void(*pipelineExecuteBoundingVolumeIntersectionStage_t)(
            DevicePipelineResources& resources,
            size_t ray_buffer_index);

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
            virtual pipelineExecuteBoundingVolumeIntersectionStage_t
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
            /** @brief Returns the launch parameter for the kernel. */
            virtual KernelLaunchParameter getLaunchParameter() = 0;
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
            pipelineExecuteBoundingVolumeIntersectionStage_t
                getKernel() override;  
            KernelLaunchParameter getLaunchParameter() override;
        private:
            Context& m_context;
        };
    }
}

#endif // LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#endif // WWU_LDPLAB_RTSCUDA_PIPELEINE_BOUNDING_VOLUME_INTERSECTION_HPP