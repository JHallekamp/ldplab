#ifndef WWU_LDPLAB_RTSCUDA_IMPL_SURFACE_INTERACTION_HPP
#define WWU_LDPLAB_RTSCUDA_IMPL_SURFACE_INTERACTION_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <LDPLAB/RayTracingStep/CUDA/ISurfaceInteraction.hpp>

namespace ldplab
{
    namespace rtscuda
    {
        class SurfaceInteraction :
            public ISurfaceInteraction
        {
        public:
            void execute(
                const GlobalData& global_data,
                BatchData& batch_data,
                size_t input_ray_buffer_index,
                size_t output_ray_buffer_index,
                size_t intersection_buffer_index,
                double intensity_cutoff,
                double medium_reflection_index,
                bool input_inner_particle_rays,
                bool reflection_pass,
                size_t pass_no) override;
        };
    }
}

#endif
#endif