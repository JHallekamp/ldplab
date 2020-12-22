#ifndef WWU_LDPLAP_RTSCPU_INNER_PARTICLE_PROPAGATION_STAGE_HPP
#define WWU_LDPLAP_RTSCPU_INNER_PARTICLE_PROPAGATION_STAGE_HPP

#include <memory>

namespace ldplab
{
    namespace rtscpu
    {
        // Prototype
        struct Context;
        struct RayBuffer;

        /**
         * @brief Inner particle propagation stage interface
         * @detail The inner particle propagation stage is responsible for
         *         calculating the path threw the particle and the resulting
         *         momentum difference.
         */
        class IInnerParticlePropagationStage
        {
        public:
            /**
             * @brief Calculating the path of the rays threw the particle.
             * @param[in] particle Index of the particle.
             * @param[in, out] rays RayBuffer holding the propagating rays. 
             */
            virtual void execute(
                const size_t particle,
                RayBuffer& rays) = 0;
        };

    }
}

#endif