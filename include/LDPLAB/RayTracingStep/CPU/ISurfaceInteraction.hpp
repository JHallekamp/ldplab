#ifndef WWU_LDPLAB_RTSCPU_I_SURFACE_INTERACTION_HPP
#define WWU_LDPLAB_RTSCPU_I_SURFACE_INTERACTION_HPP

#include <LDPLAB/ExperimentalSetup/ExperimentalSetup.hpp>
#include <LDPLAB/RayTracingStep/CPU/Data.hpp>
#include <LDPLAB/RayTracingStep/CPU/IPipelineStage.hpp>
#include <LDPLAB/SimulationState.hpp>

namespace ldplab
{
    namespace rtscpu
    {
        /** 
         * @brief Abstract baseclass for the interaction between rays and the
         *        particle surface.
         */
        class ISurfaceInteraction : public IPipelineStage
        {
        public:
            enum class InteractionPassType 
            {
                reflection,
                transmission
            };
        public:
            virtual ~ISurfaceInteraction() { }
            /**
             * @brief Performs the surface interaction.
             * @details Computes the interaction between active rays and the
             *          particles they intersect. At the intersection point,
             *          the ray interacts with the particle surface, which
             *          results in reflection and transmission of said ray.
             *          The surface interaction for a batch of rays is 
             *          computed in different passes. Each pass computes one
             *          interaction, which can either be a reflection or 
             *          transmission - determined by the pass_type paramater.
             * @param[in] input_ray_data Buffer containing a ray batch with 
             *                           rays that interact with the particle
             *                           surface.
             * @param[in] intersection_data Buffer containing the intersection
             *                              data needed to determine the 
             *                              location of the ray surface 
             *                              interaction.
             * @param[out] output_ray_data Buffer that is filled with the 
             *                             branched off rays.
             * @param[in, out] output_data Output buffer that is used to 
             *                             collect the force and torque data
             *                             generated during the ray surface
             *                             interaction.
             * @param[in] intensity_cutoff Ray intensity cutoff threshold.
             * @param[in] medium_reflection_index The reflection index of the
             *                                    physical medium between the 
             *                                    particles.
             * @param[in] pass_type The type of the pass, determining whether
             *                      the interaction results in a reflection or
             *                      in a transmission.
             * @param[in] pass_no The current pass number. Transmission and
             *                    reflection passes are counted independently,
             *                    so the number denotes the number of passes 
             *                    of the same type before.
             * @param[in] simulation_parameter Parameters of the simulation.
             * @param[in] stage_dependent_data A trhead-local pointer to a data
             *                                 structure created by the stage
             *                                 factory. Can be null, if the 
             *                                 stage factory did not provide 
             *                                 any data on stage creation.
             */
            virtual void execute(
                const RayBuffer& input_ray_data,
                const IntersectionBuffer& intersection_data,
                RayBuffer& output_ray_data,
                OutputBuffer& output_data,
                double intensity_cutoff,
                double medium_reflection_index,
                InteractionPassType pass_type,
                size_t pass_no,
                const SimulationParameter& simulation_parameter,
                void* stage_dependent_data) = 0;
        };
    }
}

#endif
