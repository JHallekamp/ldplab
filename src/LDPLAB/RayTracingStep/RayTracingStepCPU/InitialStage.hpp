#ifndef WWU_LDPLAB_RTSCPU_INITIAL_STAGE_HPP
#define WWU_LDPLAB_RTSCPU_INITIAL_STAGE_HPP

#include "Context.hpp"
#include "Data.hpp"

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
             */
            virtual bool createBatch(RayBuffer& initial_batch_buffer,
                size_t& particle_index) = 0;
        };

        /**
         * @brief Implements the initial stage interface for experimental
         *        setups using spheres as their particle bounding volume.
         */
        class InitialStageBoundingSpheres : public IInitialStage
        {
        public:
            InitialStageBoundingSpheres(
                std::shared_ptr<Context> context);
            /** @brief Inherited via ldplab::rtscpu::IInitialStage. */
            void setup() override;
            /** @brief Inherited via ldplab::rtscpu::IInitialStage. */
            bool createBatch(RayBuffer& initial_batch_buffer, 
                size_t& particle_index) override;
        private:
            std::shared_ptr<Context> m_context;

        };
    }
}

#endif