#ifndef WWU_LDPLAB_RTSCUDA_I_BOUNDING_VOLUME_INTERSECTION_HPP
#define WWU_LDPLAB_RTSCUDA_I_BOUNDING_VOLUME_INTERSECTION_HPP

#include "IPipelineStage.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        /**
         * @brief Abstract baseclass for the bounding volume intersection
         *        pipeline stage.
         * @note Since particle bounding volumes are only processed by this
         *       stage (in general), it needs to track the bounding volumes
         *       on its own. The pipeline therefore does not provide any
         *       bounding volume representations to the stage execute method.
         */
        class IBoundingVolumeIntersection : public IPipelineStage
        {
        public:
            virtual ~IBoundingVolumeIntersection() { }
            /**
             * @brief Performs an intersection test of all world space rays
             *        with the bounding spheres.
             * @details Each ray inside the buffer that is in world space (when
             *          particle index is greater than the number of particles)
             *          needs to be checked against the bounding volumes of the
             *          particles inside the experimental setup. If a ray
             *          intersects a bounding volume, it has to be transformed
             *          into the corresponding particle space. Otherwise, if
             *          the ray intersects no particle bounding volume, it has
             *          to be set to invalid by settings its corresponding
             *          particle index to a negative value.
             * @param[in] global_data All shared pipeline data.
             * @param[in] stream_context
             * @param[in] ray_buffer_index The index of the affected ray 
             *                             buffer.
             * @param[in] num_rays The number of rays inside the ray buffer.
             */
            virtual void execute(
                StreamContext& stream_context,
                size_t ray_buffer_index,
                size_t num_rays) = 0;
        };
    }
}

#endif