#ifndef WWU_LDPLAB_RTSCPU_INITIAL_STAGE_HPP
#define WWU_LDPLAB_RTSCPU_INITIAL_STAGE_HPP

#include "Data.hpp"
#include "..\..\ExperimentalSetup\Lightsource.hpp"

#include <vector>

namespace ldplab
{
    namespace rtscpu
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
         *        and having a homogeneous direction.
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
                Vec2 center;
                double radius;
                size_t light_index;
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
        private:
            std::shared_ptr<Context> m_context;
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
        };
    }
}

#endif