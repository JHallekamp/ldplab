#ifndef WWU_LDPLAB_RTSCPU_INITIAL_STAGE_HPP
#define WWU_LDPLAB_RTSCPU_INITIAL_STAGE_HPP

#include "Context.hpp"
#include "Data.hpp"

#include <vector>

namespace ldplab
{
    namespace rtscpu
    {
        /**
         * @brief Initial stage interface.
         * @detail The initial stage is responsible for projecting particles
         *         on lightsources and create
         */
        class IInitialStage
        {
        public:
            /** 
             * @brief Called once per ray tracing stage execution to setup the
             *        initial stage with the current experimental setup.
             */
            virtual void setup() = 0;
            /**
             * @brief Fills the initial batch buffer with rays.
             * @details Projects the particle bounding volumes onto light 
             *          sources to create the rays. 
             * @param[in, out] initial_batch_buffer The buffer that is used to
             *                                      store the created rays.
             * @param[out] particle_index The index of the particle which 
             *                            bounding volume is hit by the ray.
             * @returns true, if there are still rays to be created. Otherwise
             *          false, in which case the ray tracing is finished after
             *          the current batch.
             */
            virtual bool createBatch(RayBuffer& initial_batch_buffer,
                size_t& particle_index) = 0;
        };

        /**
         * @brief Implements the initial stage interface for experimental
         *        setups using spheres as their particle bounding volume
         *        and 
         */
        class InitialStageBoundingSpheresHomogenousLight : public IInitialStage
        {
        public:
            InitialStageBoundingSpheresHomogenousLight(
                std::shared_ptr<Context> context);
            /** @brief Inherited via ldplab::rtscpu::IInitialStage. */
            void setup() override;
            /** @brief Inherited via ldplab::rtscpu::IInitialStage. */
            bool createBatch(RayBuffer& initial_batch_buffer, 
                size_t& particle_index) override;
        private:
            struct Projection
            {
                // Center relative to the light source plane origin
                Vec2 center;
                double radius;
                double depth;
                // Particle index
                size_t particle_index;
                // Overlapping projections that overlay this projection 
                // partially
                std::vector<Projection*> overlaps;
            };
        private:
            std::shared_ptr<Context> m_context;
            // Projections for each particle
            std::vector<std::vector<Projection>> m_projections_per_particle;
        };
    }
}

#endif