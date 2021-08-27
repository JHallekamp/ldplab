#ifndef WWU_LDPLAB_RTSCUDA_I_SURFACE_INTERACTION_HPP
#define WWU_LDPLAB_RTSCUDA_I_SURFACE_INTERACTION_HPP

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
                StreamContext& stream_context,
                size_t ray_input_buffer_index,
                size_t ray_output_buffer_index,
                size_t intersection_buffer_index,
                size_t output_buffer_index,
                double intensity_cutoff,
                double medium_reflection_index,
                bool input_inner_particle_rays,
                bool reflection_pass,
                size_t pass_no,
                size_t num_rays) = 0;
        };
    }
}

#endif