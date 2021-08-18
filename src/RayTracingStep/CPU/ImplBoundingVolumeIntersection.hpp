#ifndef WWU_LDPLAB_RTSCPU_IMPL_BOUNDING_VOLUME_INTERSECTION_HPP
#define WWU_LDPLAB_RTSCPU_IMPL_BOUNDING_VOLUME_INTERSECTION_HPP

#include <LDPLAB/RayTracingStep/CPU/IBoundingVolumeIntersection.hpp>

namespace ldplab
{
    namespace rtscpu
    {
        /**
         * @brief Bruteforces the bounding volume intersection test for
         *        bounding spheres.
         */
        class BoundingSphereIntersectionBruteforce :
            public IBoundingVolumeIntersection
        {
        public:
            void stepSetup(
                const ExperimentalSetup& setup,
                const SimulationState& simulation_state,
                const InterfaceMapping& interface_mapping,
                const std::vector<ParticleTransformation>& particle_transformation) override;
            virtual size_t execute(
                RayBuffer& ray_data, 
                const std::vector<ParticleTransformation>& transformation_data, 
                const SimulationParameter& simulation_parameter, 
                void* stage_dependent_data) override;
        private:
            std::vector<BoundingVolumeSphere> m_particle_bounding_spheres;
        };
    }
}

#endif