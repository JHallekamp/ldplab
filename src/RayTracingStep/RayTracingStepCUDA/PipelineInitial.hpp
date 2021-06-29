#ifndef WWU_LDPLAB_RTSCUDA_PIPELINE_INITIAL_HPP
#define WWU_LDPLAB_RTSCUDA_PIPELINE_INITIAL_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <cuda_runtime.h>
#include <LDPLAB/ExperimentalSetup/ExperimentalSetup.hpp>
#include <LDPLAB/Geometry.hpp>
#include <LDPLAB/RayTracingStep/RayTracingStepCUDAInfo.hpp>
#include <memory>

#include "CudaResource.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        // Prototype
        struct Context;
        struct KernelLaunchParameter;
        struct DevicePipelineResources;

        typedef bool(*pipelineExecuteInitialStage_t)(
            DevicePipelineResources& resources,
            size_t initial_ray_buffer_index,
            size_t batch_no);
        
        /** @brief Abstract baseclass for the initial stage. */
        class IPipelineInitialStage
        {
        public:
            virtual ~IPipelineInitialStage() { }
            /** @brief Creates an instance of a pipeline stage implementation. */
            static std::shared_ptr<IPipelineInitialStage> createInstance(
                    const ExperimentalSetup& setup,
                    const RayTracingStepCUDAInfo& info, 
                    Context& context);
            /** @brief Returns the execution kernel. */
            virtual pipelineExecuteInitialStage_t getKernel() = 0;
            /** @brief Gets called before the pipeline enters execution. */
            virtual void setup() { }
            /** @brief Fills the initial batch buffer with rays. */
            virtual bool execute(size_t initial_ray_buffer_index) = 0;
            /** @brief Returns the launch parameter for the kernel. */
            virtual KernelLaunchParameter getLaunchParameter() = 0;
        protected:
            virtual bool allocate(
                const ExperimentalSetup& setup,
                const RayTracingStepCUDAInfo& info) = 0;
        };

        class PipelineInitialHomogenousLightBoundingSpheres : 
            public IPipelineInitialStage
        {
        public:
            PipelineInitialHomogenousLightBoundingSpheres(Context& context);
            void setup() override;
            pipelineExecuteInitialStage_t getKernel() override;
            bool execute(size_t initial_ray_buffer_index) override;
            KernelLaunchParameter getLaunchParameter() override;
        protected:
            bool allocate(
                const ExperimentalSetup& setup,
                const RayTracingStepCUDAInfo& info) override;
        public:
            struct Rect
            {
                int x;
                int y;
                int width;
                int height;
            };
            struct HomogenousLightSource
            {
                Vec3 origin;
                Vec3 x_axis;
                Vec3 y_axis;
                double width;
                double height;
                Vec3 ray_direction;
                double ray_intensity;
            };
        private:
            Context& m_context;
            CudaPtr<Rect> m_projection_buffer;
            CudaPtr<HomogenousLightSource> m_light_source_buffer;
            CudaPtr<size_t> m_num_rays_buffer;
            CudaPtr<size_t> m_temp_num_rays_buffer;
            size_t m_batch_ctr;
            size_t m_total_batch_count;
        };
    }
}

#endif // LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#endif // WWU_LDPLAB_RTSCUDA_PIPELINE_INITIAL_HPP