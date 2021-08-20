#ifndef WWU_LDPLAB_RTSCUDA_IMPL_INITIAL_STAGE_HPP
#define WWU_LDPLAB_RTSCUDA_IMPL_INITIAL_STAGE_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <LDPLAB/RayTracingStep/CUDA/IInitialStage.hpp>
#include "ImplBoundingVolumeIntersection.hpp"

#include <atomic>

namespace ldplab
{
    namespace rtscuda
    {
        class InitialStageHomogenousLightBoundingSphereProjection :
            public IInitialStage
        {
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
            struct PerDeviceData
            {
                DeviceBuffer<BoundingSphere> bounding_spheres;
                DeviceBuffer<Rect> projection_buffer;
                DeviceBuffer<HomogenousLightSource> light_source_buffer;
                DeviceBuffer<size_t> num_rays_buffer;
                DeviceBuffer<size_t> temp_num_rays_buffer;
            };
        public:
            InitialStageHomogenousLightBoundingSphereProjection(
                double light_resolution_per_world_unit,
                std::vector<PerDeviceData>&& per_device_data);
            void stepSetup(
                const SimulationState& simulation_state,
                SharedStepData& shared_data,
                DeviceContext& device_context) override;
            bool execute(
                StreamContext& stream_context,
                size_t batch_no,
                size_t initial_batch_buffer_index) override;
        private:
            double m_light_resolution_per_world_unit;
            std::vector<PerDeviceData> m_per_device_data;
            size_t m_total_batch_count;
        };
    }
}

#endif
#endif