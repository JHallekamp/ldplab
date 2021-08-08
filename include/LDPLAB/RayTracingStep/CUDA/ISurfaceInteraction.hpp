#ifndef WWU_LDPLAB_RTSCUDA_I_SURFACE_INTERACTION_HPP
#define WWU_LDPLAB_RTSCUDA_I_SURFACE_INTERACTION_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include "IPipelineStage.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        class ISurfaceInteraction : public IPipelineStage
        {
        public:
            virtual ~ISurfaceInteraction() { }
            virtual void execute(
                const GlobalData& global_data,
                BatchData& batch_data,
                size_t input_ray_buffer_index,
                size_t output_ray_buffer_index,
                size_t intersection_buffer_index,
                double intensity_cutoff,
                double medium_reflection_index,
                bool input_inner_particle_rays,
                bool reflection_pass,
                size_t pass_no) = 0;
        };
    }
}

#endif
#endif