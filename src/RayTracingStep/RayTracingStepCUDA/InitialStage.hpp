#ifndef WWU_LDPLAB_RTSCUDA_INITIAL_STAGE_HPP
#define WWU_LDPLAB_RTSCUDA_INITIAL_STAGE_HPP

#include "Data.hpp"
#include <LDPLAB/ExperimentalSetup/Lightsource.hpp>

#include <mutex>
#include <vector>

namespace ldplab
{
    namespace rtscuda
    {
        //Prototype
        struct Context;

        /**
         * @brief Initial stage interface.
         * @detail The initial stage is responsible for projecting particles
         *         on light sources and create batches of rays.
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
             *          sources to only create rays that will hit the bounding
             *          volume.
             * @param[in, out] initial_batch_buffer The buffer that is used to
             *                                      store the created rays.
             * @returns true, if there are still rays to be created. Otherwise
             *          false, in which case the ray tracing is finished after
             *          the current batch.
             */
            virtual bool createBatch(RayBuffer& initial_batch_buffer) = 0;
        };

        /**
         * @brief Implements the initial stage interface for experimental
         *        setups using spheres as their particle bounding volume
         *        and having a homogeneous direction.
         */
        class InitialStageBoundingSpheresHomogenousLight : public IInitialStage
        {
        public:
            InitialStageBoundingSpheresHomogenousLight(
                Context& context);
            /** @brief Inherited via ldplab::rtscuda::IInitialStage. */
            void setup() override;
            /** @brief Inherited via ldplab::rtscuda::IInitialStage. */
            bool createBatch(RayBuffer& initial_batch_buffer) override;
        private:
            struct Projection
            {
                Vec2 center;
                double radius;
                size_t light_index;
                double depth;
                // Pointers to overlapping projections
                std::vector<Projection*> overlaps;
            };
        private:
            bool projLightOverlap(
                const Vec2& center,
                const double radius,
                const LightSource& light_source) const;
            bool hasToCreateRay(
                const Projection& projection,
                const LightSource& light_source) const;
            void advBatchCreationLight(size_t& li);
            void advBatchCreationParticle(size_t& pi);
            inline void transformRayFromWorldToParticleSpace(
                Ray& ray, size_t pidx) const;
        private:
            Context& m_context;
            // Projections for each particle
            std::vector<std::vector<Projection>> m_projections_per_particle;
            size_t m_batch_creation_particle_index;
            size_t m_batch_creation_light_index;
            bool m_batch_creation_particle_initialized;
            double m_rasterization_x;
            double m_rasterization_y;
            bool m_rasterization_up;
            bool m_rasterization_right;
            double m_rasterization_step_size;
            std::mutex m_mutex;
        };
    }
}

#endif