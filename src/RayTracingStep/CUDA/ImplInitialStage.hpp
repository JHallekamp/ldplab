#ifndef WWU_LDPLAB_RTSCUDA_IMPL_INITIAL_STAGE_HPP
#define WWU_LDPLAB_RTSCUDA_IMPL_INITIAL_STAGE_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <LDPLAB/RayTracingStep/CUDA/IInitialStage.hpp>
#include "ImplBoundingVolumeIntersection.hpp"

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
        public:
            InitialStageHomogenousLightBoundingSphereProjection(
                double light_resolution_per_world_unit,
                DeviceBuffer<BoundingSphere>&& bounding_spheres,
                DeviceBuffer<Rect>&& projection_buffer,
                DeviceBuffer<HomogenousLightSource>&& light_source_buffer,
                DeviceBuffer<size_t>&& num_rays_buffer,
                DeviceBuffer<size_t>&& temp_num_rays_buffer);
            void stepSetup(
                const SimulationState& simulation_state,
                const GlobalData& global_data) override;
            bool execute(
                const GlobalData& global_data,
                BatchData& batch_data,
                size_t initial_batch_buffer_index) override;
        private:
            double m_light_resolution_per_world_unit;
            DeviceBuffer<BoundingSphere> m_bounding_spheres;
            DeviceBuffer<Rect> m_projection_buffer;
            DeviceBuffer<HomogenousLightSource> m_light_source_buffer;
            DeviceBuffer<size_t> m_num_rays_buffer;
            DeviceBuffer<size_t> m_temp_num_rays_buffer;
            size_t m_batch_ctr;
            size_t m_total_batch_count;
        };
    }
}

#endif
#endif