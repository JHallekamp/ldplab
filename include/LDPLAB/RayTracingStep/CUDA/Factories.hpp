#ifndef WWU_LDPLAB_RTSCUDA_STAGE_FACTORIES_HPP
#define WWU_LDPLAB_RTSCUDA_STAGE_FACTORIES_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <memory>
#include <string>
#include <vector>

#include "IBoundingVolumeIntersection.hpp"
#include "IGenericGeometry.hpp"
#include "IGenericMaterial.hpp"
#include "IInitialStage.hpp"
#include "IInnerParticlePropagation.hpp"
#include "IParticleIntersection.hpp"
#include "ISurfaceInteraction.hpp"
#include "PipelineConfiguration.hpp"

#include <LDPLAB/RayTracingStep/RayTracingStepCUDAInfo.hpp>

namespace ldplab
{
    namespace rtscuda
    {
        /** @brief Abstract interface for pipeline stage factory baseclasses. */
        class IStageFactory
        {
        public:
            virtual ~IStageFactory() { }
            /**
             * @brief Returns a human readable name for the pipeline stage
             *        implementation.
             * @details This is used for logging purposes. However one could
             *          also check for specific stage implementaions using
             *          their respective name.
             *          All default implementations are required to have a
             *          static method to retrieve their implementation name.
             *          For default implementations these names are unique.
             *          Obviously user defined stage implementations could
             *          violate the above rules. Check the public userDefined
             *          method to distinguish between default and user defined
             *          stage implementations.
             */
            virtual std::string implementationName() const = 0;
            /**
             * @brief Determines whether a stage implementation is part of the
             *        LDPLAB default implementations or user defined.
             * @returns true, if the pipeline stage is user defined.
             * @warning This should only ever be overwritten by default
             *          pipeline stage implementations to ensure expected
             *          behaviour.
             */
            virtual bool userDefined() const { return true; }
            /**
             * @brief Checks whether the pipeline stage instance would be
             *        compatible with the step and pipeline configuration, as
             *        well as the given experimental setup.
             * @details The ray tracing step factory creates a pipeline
             *          configuration that is used to create the ray tracing
             *          pipeline. Therefore it uses a user defined config on
             *          the one hand and a default one on the other.
             *          The prefered option is to cast pipeline stages with
             *          the user defined ones. However, if the user does not
             *          provides a stage factory for a specific stage, the
             *          factory reverts to using the default one (if it is
             *          able to select a default stage in the first place).
             *          As soon as the configuration is created, all stages
             *          are checked for compability. Any stage, that is not
             *          compatible will be replaced by a default option, so
             *          long as there are still options available (this
             *          behaviour can actually be prevented by the user, to
             *          force the factory to abort, if the initial
             *          configuration turns out to be invalid).
             * @param[in] step_info The ray tracing step info that is used to
             *                      create the ray tracing step pipeline.
             * @param[in] configuration The current complete configuration, for
             *                          which the pipeline stage may be
             *                          created, if all factories within the
             *                          configuration agree.
             * @param[in] setup The experimental setup for which the ray
             *                  tracing step is created.
             * @param[in] interface_mapping The ray tracing step interface
             *                              mapping to relate experimental
             *                              setup and simulation state
             *                              components given by their uid to
             *                              the internal index based structure.
             * @returns true, if the pipeline stage, which is created by this
             *          factory instance is compatible to the given configs and
             *          setup. Otherwise false.
             */
            virtual bool checkCompability(
                const RayTracingStepCUDAInfo& step_info,
                const PipelineConfiguration& configuration,
                const ExperimentalSetup& setup,
                const InterfaceMapping& interface_mapping) = 0;
            /**
             * @brief Called to create the stage dependent thread-local data.
             * @details The ray tracing step pipeline is meant to be executed
             *          in parallel. Resources like ray buffers are
             *          thread-local, so each thread only manages its own.
             *          If a specific stage implementation requires to add
             *          additional stage dependent data, for example to
             *          individual rays, it can do so by managing this data
             *          on its own and refer to it using the ray buffer and
             *          individual ray index.
             *          However, if done so, it cannot manage that data as
             *          normal member data, since one pipeline stage instance
             *          is shared across all pipeline threads.
             *          The createStageDependentData method solves this, by
             *          allocating stage implementation defined data for each
             *          pipeline thread, which is then later passed to the
             *          stages execution method.
             *          The created data is always regarded as thread-local and
             *          will only be passed to methods that are executed by the
             *          thread for which it has originally been allocated.
             * @param[in] global_data All pipeline resources present.
             * @param[in] configuration The current complete configuration, for
             *                          which the pipeline stage may be
             *                          created, if all factories within the
             *                          configuration agree.
             * @param[in] setup The experimental setup for which the ray
             *                  tracing step is created.
             * @param[in] interface_mapping The ray tracing step interface
             *                              mapping to relate experimental
             *                              setup and simulation state
             *                              components given by their uid to
             *                              the internal index based structure.
             * @param[out] stage_dependent_data Pointer to the allocated
             *                                  thread-local data. Can be a
             *                                  nullptr, if no data has to be
             *                                  passed to the stage when
             *                                  executed.
             * @returns true, if the data creation has been successful or if
             *          no data has to be created in the first place.
             *          Otherwise false.
             * @note Stage dependent data creation is always called some time
             *       after the pipeline stage implementation instance has been
             *       successfully created.
             */
            virtual bool createStageDependentData(
                const GlobalData& global_data,
                const RayTracingStepCUDAInfo& step_info,
                const PipelineConfiguration& configuration,
                const ExperimentalSetup& setup,
                const InterfaceMapping& interface_mapping,
                std::shared_ptr<void>& stage_dependent_data)
            {
                return true;
            }
        };

        /** @brief Abstract baseclass for initial stage factories. */
        class IInitialStageFactory : public IStageFactory
        {
        public:
            virtual ~IInitialStageFactory() { }
            /**
             * @brief Creates the stage implementation instance.
             * @param[in] step_info The ray tracing step info that is used to
             *                      create the ray tracing step pipeline.
             * @param[in] configuration The current complete configuration, for
             *                          which the pipeline stage may be
             *                          created, if all factories within the
             *                          configuration agree.
             * @param[in] setup The experimental setup for which the ray
             *                  tracing step is created.
             * @param[in] interface_mapping The ray tracing step interface
             *                              mapping to relate experimental
             *                              setup and simulation state
             *                              components given by their uid to
             *                              the internal index based structure.
             * @returns A shared pointer to the newly created stage or nullptr
             *          if the instance creation failed.
             */
            virtual std::shared_ptr<IInitialStage> create(
                const RayTracingStepCUDAInfo& step_info,
                const PipelineConfiguration& configuration,
                const ExperimentalSetup& setup,
                const InterfaceMapping& interface_mapping) = 0;
        };

        /**
         * @brief Abstract baseclass for inner particle propagation stage
         *        factories.
         */
        class IInnerParticlePropagationFactory : public IStageFactory
        {
        public:
            virtual ~IInnerParticlePropagationFactory() { }
            /**
             * @brief Creates the stage implementation instance.
             * @param[in] step_info The ray tracing step info that is used to
             *                      create the ray tracing step pipeline.
             * @param[in] configuration The current complete configuration, for
             *                          which the pipeline stage may be
             *                          created, if all factories within the
             *                          configuration agree.
             * @param[in] setup The experimental setup for which the ray
             *                  tracing step is created.
             * @param[in] interface_mapping The ray tracing step interface
             *                              mapping to relate experimental
             *                              setup and simulation state
             *                              components given by their uid to
             *                              the internal index based structure.
             * @returns A shared pointer to the newly created stage or nullptr
             *          if the instance creation failed.
             */
            virtual std::shared_ptr<IInnerParticlePropagation> create(
                const RayTracingStepCUDAInfo& step_info,
                const PipelineConfiguration& configuration,
                const ExperimentalSetup& setup,
                const InterfaceMapping& interface_mapping) = 0;
        };

        /**
         * @brief Abstract baseclass for particle intersection stage
         *        factories.
         */
        class IParticleIntersectionFactory : public IStageFactory
        {
        public:
            virtual ~IParticleIntersectionFactory() { }
            /**
             * @brief Creates the stage implementation instance.
             * @param[in] step_info The ray tracing step info that is used to
             *                      create the ray tracing step pipeline.
             * @param[in] configuration The current complete configuration, for
             *                          which the pipeline stage may be
             *                          created, if all factories within the
             *                          configuration agree.
             * @param[in] setup The experimental setup for which the ray
             *                  tracing step is created.
             * @param[in] interface_mapping The ray tracing step interface
             *                              mapping to relate experimental
             *                              setup and simulation state
             *                              components given by their uid to
             *                              the internal index based structure.
             * @returns A shared pointer to the newly created stage or nullptr
             *          if the instance creation failed.
             */
            virtual std::shared_ptr<IParticleIntersection> create(
                const RayTracingStepCUDAInfo& step_info,
                const PipelineConfiguration& configuration,
                const ExperimentalSetup& setup,
                const InterfaceMapping& interface_mapping) = 0;
        };

        /**
         * @brief Abstract baseclass for surface interaction stage
         *        factories.
         */
        class ISurfaceInteractionFactory : public IStageFactory
        {
        public:
            virtual ~ISurfaceInteractionFactory() { }
            /**
             * @brief Creates the stage implementation instance.
             * @param[in] step_info The ray tracing step info that is used to
             *                      create the ray tracing step pipeline.
             * @param[in] configuration The current complete configuration, for
             *                          which the pipeline stage may be
             *                          created, if all factories within the
             *                          configuration agree.
             * @param[in] setup The experimental setup for which the ray
             *                  tracing step is created.
             * @param[in] interface_mapping The ray tracing step interface
             *                              mapping to relate experimental
             *                              setup and simulation state
             *                              components given by their uid to
             *                              the internal index based structure.
             * @returns A shared pointer to the newly created stage or nullptr
             *          if the instance creation failed.
             */
            virtual std::shared_ptr<ISurfaceInteraction> create(
                const RayTracingStepCUDAInfo& step_info,
                const PipelineConfiguration& configuration,
                const ExperimentalSetup& setup,
                const InterfaceMapping& interface_mapping) = 0;
        };

        /** @brief Abstract baseclass for generic geometry factories. */
        class IGenericGeometryFactory
        {
        public:
            virtual ~IGenericGeometryFactory() { }
            /**
             * @brief Returns a human readable name for the generic geometry
             *        implementstion
             * @default This is used mainly for logging purposes. However one
             *          could also check for specific generic geometry
             *          implementaions using their respective name.
             *          All default implementations are required to have a
             *          static method to retrieve their implementation name.
             *          For default implementations these names are unique.
             *          Obviously user defined stage implementations could
             *          violate the above rules. Check the userDefined method
             *          to distinguish between default and user defined generic
             *          geometry implementations.
             */
            virtual std::string implementationName() const = 0;
            /**
             * @brief Determines whether a stage implementation is part of the
             *        LDPLAB default implementations or user defined.
             * @returns true, if the pipeline stage is user defined.
             * @warning This should only ever be overwritten by default
             *          pipeline stage implementations to ensure expected
             *          behaviour.
             */
            virtual bool userDefined() const { return true; }
            /**
             * @brief Checks whether the generic geometry instance would be
             *        compatible with the particle geometry, the step and
             *        pipeline configuration, as  well as the given
             *        experimental setup.
             * @param[in] geometry_type The type of the geometry that has to be
             *                          represented by the generic geometry
             *                          implementation that is instanciated by
             *                          this factory instance.
             * @param[in] step_info The ray tracing step info that is used to
             *                      create the ray tracing step pipeline.
             * @param[in] configuration The current complete configuration, for
             *                          which the pipeline stage may be
             *                          created, if all factories within the
             *                          configuration agree.
             * @param[in] setup The experimental setup for which the ray
             *                  tracing step is created.
             * @param[in] interface_mapping The ray tracing step interface
             *                              mapping to relate experimental
             *                              setup and simulation state
             *                              components given by their uid to
             *                              the internal index based structure.
             * @returns true, if the generic geometry implementation instance,
             *          which is created by this factory instance is compatible
             *          to the given configs and setup. Otherwise false.
             */
            virtual bool checkCompability(
                IParticleGeometry::Type geometry_type,
                const RayTracingStepCUDAInfo& step_info,
                const PipelineConfiguration& configuration,
                const ExperimentalSetup& setup,
                const InterfaceMapping& interface_mapping) = 0;
            /**
            * @brief Creates the generic particle implementation instance.
            * @param[in] particle_geometry The geometry of the particle for
            *                              which the generic geometry is
            *                              created.
            * @param[in] step_info The ray tracing step info that is used to
            *                      create the ray tracing step pipeline.
            * @param[in] configuration The current complete configuration, for
            *                          which the pipeline stage may be
            *                          created, if all factories within the
            *                          configuration agree.
            * @param[in] setup The experimental setup for which the ray
            *                  tracing step is created.
            * @param[in] interface_mapping The ray tracing step interface
            *                              mapping to relate experimental
            *                              setup and simulation state
            *                              components given by their uid to
            *                              the internal index based structure.
            * @returns A shared pointer to the newly created stage or nullptr
            *          if the instance creation failed.
            */
            virtual std::shared_ptr<IGenericGeometry> create(
                const std::shared_ptr<IParticleGeometry>& particle_geometry,
                const RayTracingStepCUDAInfo& step_info,
                const PipelineConfiguration& configuration,
                const ExperimentalSetup& setup,
                const InterfaceMapping& interface_mapping) = 0;
        };

        /** @brief Abstract baseclass for generic material factories. */
        class IGenericMaterialFactory
        {
        public:
            virtual ~IGenericMaterialFactory() { }
            /**
             * @brief Returns a human readable name for the generic material
             *        implementstion
             * @default This is used mainly for logging purposes. However one
             *          could also check for specific generic material
             *          implementaions using their respective name.
             *          All default implementations are required to have a
             *          static method to retrieve their implementation name.
             *          For default implementations these names are unique.
             *          Obviously user defined stage implementations could
             *          violate the above rules. Check the userDefined method
             *          to distinguish between default and user defined generic
             *          material implementations.
             */
            virtual std::string implementationName() const = 0;
            /**
             * @brief Determines whether an implementation is part of the
             *        LDPLAB default implementations or user defined.
             * @returns true, if the generic material is user defined.
             * @warning This should only ever be overwritten by default
             *          implementations to ensure expected behaviour.
             */
            virtual bool userDefined() const { return true; }
            /**
             * @brief Checks whether the generic material instance would be
             *        compatible with the particle material, the step and
             *        pipeline configuration, as  well as the given
             *        experimental setup.
             * @param[in] material_type The type of the material that has to be
             *                          represented by the generic material
             *                          implementation that is instanciated by
             *                          this factory instance.
             * @param[in] step_info The ray tracing step info that is used to
             *                      create the ray tracing step pipeline.
             * @param[in] configuration The current complete configuration, for
             *                          which the pipeline stage may be
             *                          created, if all factories within the
             *                          configuration agree.
             * @param[in] setup The experimental setup for which the ray
             *                  tracing step is created.
             * @param[in] interface_mapping The ray tracing step interface
             *                              mapping to relate experimental
             *                              setup and simulation state
             *                              components given by their uid to
             *                              the internal index based structure.
             * @returns true, if the generic material implementation instance,
             *          which is created by this factory instance is compatible
             *          to the given configs and setup. Otherwise false.
             */
            virtual bool checkCompability(
                IParticleMaterial::Type material_type,
                const RayTracingStepCUDAInfo& step_info,
                const PipelineConfiguration& configuration,
                const ExperimentalSetup& setup,
                const InterfaceMapping& interface_mapping) = 0;
            /**
             * @brief Creates the generic material implementation instance.
             * @param[in] particle_material The material of the particle for
             *                              which the generic material is
             *                              created.
             * @param[in] step_info The ray tracing step info that is used to
             *                      create the ray tracing step pipeline.
             * @param[in] configuration The current complete configuration, for
             *                          which the pipeline stage may be
             *                          created, if all factories within the
             *                          configuration agree.
             * @param[in] setup The experimental setup for which the ray
             *                  tracing step is created.
             * @param[in] interface_mapping The ray tracing step interface
             *                              mapping to relate experimental
             *                              setup and simulation state
             *                              components given by their uid to
             *                              the internal index based structure.
             * @returns A shared pointer to the newly created stage or nullptr
             *          if the instance creation failed.
             */
            virtual std::shared_ptr<IGenericMaterial> create(
                const std::shared_ptr<IParticleMaterial>& particle_material,
                const RayTracingStepCUDAInfo& step_info,
                const PipelineConfiguration& configuration,
                const ExperimentalSetup& setup,
                const InterfaceMapping& interface_mapping) = 0;
        };
    }
}

#endif
#endif