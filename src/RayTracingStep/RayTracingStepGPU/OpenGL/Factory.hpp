#ifndef WWU_LDPLAB_RTSGPU_OGL_FACTORY_HPP
#define WWU_LDPLAB_RTSGPU_OGL_FACTORY_HPP

#include "Context.hpp"
#include "RayTracingStep.hpp"
#include <LDPLAB/RayTracingStep/RayTracingStepGPUOpenGLInfo.hpp>
#include <LDPLAB/ExperimentalSetup/ExperimentalSetup.hpp>

namespace ldplab
{
    namespace rtsgpu_ogl
    {
        class Factory
        {
        public:
            /** @brief Creates the ray tracing step instance */
            static std::shared_ptr<RayTracingStep> createRTS(
                const ExperimentalSetup& setup,
                const RayTracingStepGPUOpenGLInfo& info);
        private:
            // Saves the types within the setup and info
            ILightDirection::Type m_light_direction_type;
            ILightDistribution::Type m_light_distribution_type;
            ILightPolarisation::Type m_light_polarization_type;
            IBoundingVolume::Type m_bounding_volume_type;
            IParticleGeometry::Type m_particle_geometry_type;
            IParticleMaterial::Type m_particle_material_type;
            IEikonalSolver::Type m_solver_type;
        private:
            /** @brief Factory is used by factory functions. */
            Factory() { }
            /** 
             * @brief Checks if the given experimental setup is in a valid and 
             *        supported state. 
             * @param[in] setup The given experimental setup.
             * @param[in] info The given info structure.
             * @returns true if the setup is valid.
             */
            bool validateSetup(
                const ExperimentalSetup& setup,
                const RayTracingStepGPUOpenGLInfo& info);
            /** 
             * @brief Sets the type member. 
             * @param[in] setup The given experimental setup.
             *  * @param[in] info The given info structure.
             * @note Called by validateSetup.
             */
            void setTypes(
                const ExperimentalSetup& setup,
                const RayTracingStepGPUOpenGLInfo& info);
            /**
             * @brief Checks if the types in the experimental setup are
             *        homogenous.
             * @param[in] setup The given experimental setup.
             * @returns true, if all types are valid.
             * @note Called by validateSetup.
             */
            bool validateTypeHomogeneity(const ExperimentalSetup& setup);
            /** @brief Create context. */
            bool createContext(
                const ExperimentalSetup& setup,
                const RayTracingStepGPUOpenGLInfo& info,
                std::shared_ptr<Context>& context);
            /** @brief Creates data structrue instances. */
            bool createDataInstances(
                const ExperimentalSetup& setup,
                const RayTracingStepGPUOpenGLInfo& info,
                Context& context);
            /** @brief Creates bounding spheres data instances. */
            void createBoundingSphereDataInstances(
                const ExperimentalSetup& setup,
                const RayTracingStepGPUOpenGLInfo& info,
                Context& context);
            /** @brief Creates rod particle data instances. */
            void createRodParticleDataInstances(
                const ExperimentalSetup& setup,
                const RayTracingStepGPUOpenGLInfo& info,
                Context& context);
            /** @brief Creates linear unpolarized one directional material. */
            void createParticleMaterialLinearOneDirectionalInstances(
                const ExperimentalSetup& setup,
                const RayTracingStepGPUOpenGLInfo& info,
                Context& context);
            /** @brief Creates the stages of the ray tracing pipeline. */
            bool createPipelineStages(
                const ExperimentalSetup& setup,
                const RayTracingStepGPUOpenGLInfo& info,
                Context& context,
                std::unique_ptr<IInitialStage>& initial,
                std::unique_ptr<IInnerParticlePropagationStage>& ipp,
                std::unique_ptr<IRayBoundingVolumeIntersectionTestStage>& rbvi,
                std::unique_ptr<IRayParticleInteractionStage>& rpi,
                std::unique_ptr<IRayParticleIntersectionTestStage>& rpit);
            /**
             * @brief Creates a initial stage if support for the given types
             *        exists.
             * @note Called by createPipelineStages.
             */
            bool createInitialStage(
                const ExperimentalSetup& setup,
                const RayTracingStepGPUOpenGLInfo& info,
                Context& context,
                std::unique_ptr<IInitialStage>& stage);
            /**
             * @brief Creates a inner particle propagation stage if support for
             *        the given types exists.
             * @note Called by createPipelineStages.
             */
            bool createInnerParticlePropagationStage(
                const ExperimentalSetup& setup,
                const RayTracingStepGPUOpenGLInfo& info,
                Context& context,
                std::unique_ptr<IInnerParticlePropagationStage>& stage);
            /**
             * @brief Creates a ray bounding volume intersection test stage if
             *        support for the given types exists.
             * @note Called by createPipelineStages.
             */
            bool createRayBoundingVolumeIntersectionTestStage(
                const ExperimentalSetup& setup,
                const RayTracingStepGPUOpenGLInfo& info,
                Context& context,
                std::unique_ptr<IRayBoundingVolumeIntersectionTestStage>& stage);
            /**
             * @brief Creates a ray particle interaction stage if support for
             *        the given types exists.
             * @note Called by createPipelineStages.
             */
            bool createRayParticleInteractionStage(
                const ExperimentalSetup& setup,
                const RayTracingStepGPUOpenGLInfo& info,
                Context& context,
                std::unique_ptr<IRayParticleInteractionStage>& stage);
            /**
             * @brief Creates a ray particle intersection test stage if support
             *        for the given types exists.
             * @note Called by createPipelineStages.
             */
            bool createRayParticleIntersectionTestStage(
                const ExperimentalSetup& setup,
                const RayTracingStepGPUOpenGLInfo& info,
                Context& context,
                std::unique_ptr<IRayParticleIntersectionTestStage>& stage);
        };
    }
}

#endif