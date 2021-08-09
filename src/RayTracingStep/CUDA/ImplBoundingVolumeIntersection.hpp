#ifndef WWU_LDPLAB_RTSCUDA_IMPL_BOUNDING_VOLUME_INTERSECTION_HPP
#define WWU_LDPLAB_RTSCUDA_IMPL_BOUNDING_VOLUME_INTERSECTION_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <LDPLAB/ExperimentalSetup/BoundingVolume.hpp>
#include <LDPLAB/RayTracingStep/CUDA/IBoundingVolumeIntersection.hpp>

namespace ldplab
{
    namespace rtscuda
    {
        struct BoundingSphere
        {
            BoundingSphere();
            BoundingSphere(const BoundingVolumeSphere&);
            Vec3 center;
            double radius;
        };

        class BoundingSphereIntersectionBruteforce :
            public IBoundingVolumeIntersection
        {
        public:
            /**
             * @brief Ctor
             * @param[in] bounding_spheres Allocated buffer on host and device
             *                             to contain bounding sphere data per
             *                             particle.
             */
            BoundingSphereIntersectionBruteforce(
                DeviceBuffer<BoundingSphere>&& bounding_spheres);
            void stepSetup(
                const SimulationState& simulation_state,
                GlobalData& global_data) override;
            void execute(const GlobalData& global_data,
                BatchData& batch_data,
                size_t ray_buffer_index) override;
        private:
            DeviceBuffer<BoundingSphere> m_bounding_spheres;
        };
    }
}

#endif
#endif