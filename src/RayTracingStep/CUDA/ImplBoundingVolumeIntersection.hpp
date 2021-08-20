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
                std::vector<DeviceBuffer<BoundingSphere>>&& bounding_spheres_per_device);
            void stepSetup(
                const SimulationState& simulation_state,
                SharedStepData& shared_data,
                DeviceContext& device_context) override;
            void execute(
                StreamContext& stream_context,
                size_t ray_buffer_index) override;
        private:
            std::vector<DeviceBuffer<BoundingSphere>> m_bounding_spheres_per_device;
        };
    }
}

#endif
#endif