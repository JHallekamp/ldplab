#ifndef WWU_LDPLAB_RTSCUDA_PIPELINE_CONFIGURATION_HPP
#define WWU_LDPLAB_RTSCUDA_PIPELINE_CONFIGURATION_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <memory>
#include <map>

#include <LDPLAB/ExperimentalSetup/ExperimentalSetup.hpp>

namespace ldplab
{
    namespace rtscuda
    {
        // Prototypes
        class IBoundingVolumeIntersectionFactory;
        class IInitialStageFactory;
        class IInnerParticlePropagationFactory;
        class IParticleIntersectionFactory;
        class ISurfaceInteractionFactory;
        class IGenericGeometryFactory;
        class IGenericMaterialFactory;

        /**
         * @brief Holds a configuration of pipeline stages to be used for a
         *        pipeline instance.
         * @details Each stage slot in the configuration can be cast with a
         *          pipeline stage factory. If a configuration is valid, which
         *          means that the experimental setup and the rest of the
         *          pipeline satisfies all the needs of a given stage, the
         *          pipeline instance is created, using the given stage
         *          factories to create the individual states.
         *          The ray tracing step factory also creates a default
         *          configuration for any given experimental setup, if it can,
         *          and replaces unoccupied slots of a given configuration with
         *          the default stages. It also tries to replace stages, that
         *          are not valid with respect to the given configuration and
         *          setup, with default stages. This behaviour can be prevented
         *          however.
         */
        struct PipelineConfiguration
        {
            std::shared_ptr<IBoundingVolumeIntersectionFactory> bounding_volume_intersection;
            std::shared_ptr<IInitialStageFactory> initial_stage;
            std::shared_ptr<IInnerParticlePropagationFactory> inner_particle_propagation;
            std::shared_ptr<IParticleIntersectionFactory> particle_intersection;
            std::shared_ptr<ISurfaceInteractionFactory> surface_interaction;
            /**
             * @brief Maps particle geometry types to generic particle
             *        geometry factories.
             * @details Every particle geometry type that occurs in the
             *          experimental setup but has no associated generic
             *          geometry factory will have its generic geometry
             *          representation created by a default generic geometry
             *          factory, if possible.
             */
            std::map<IParticleGeometry::Type, std::shared_ptr<IGenericGeometryFactory>>
                generic_geometries;
            /**
             * @brief Maps particle material types to generic particle
             *        material factories.
             * @details Every particle material type that occurs in the
             *          experimental setup but has no associated generic
             *          material factory will have its generic material
             *          representation created by a default generic material
             *          factory, if possible.
             */
            std::map<IParticleMaterial::Type, std::shared_ptr<IGenericMaterialFactory>>
                generic_materials;
        };
    }
}

#endif
#endif