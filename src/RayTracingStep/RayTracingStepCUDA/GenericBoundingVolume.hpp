#ifndef WWU_LDPLAB_RTSCUDA_GENERIC_BOUNDING_VOLUME_HPP
#define WWU_LDPLAB_RTSCUDA_GENERIC_BOUNDING_VOLUME_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <LDPLAB/ExperimentalSetup/BoundingVolume.hpp>
#include <LDPLAB/Geometry.hpp>
#include <memory>

#include "CudaResource.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        /** 
         * @brief Typedefinition of bounding volume intersection kernel 
         *        function pointer.
         * @param[in] ray_origin The ray origin in particle space.
         * @param[in] ray_direction The ray direction in particle space.
         * @param[in] bounding_volume_data Pointer to the bounding volume data.
         * @param[out] dist Distance from ray_origin to the bounding volume
         *                  intersection.
         * @returns true, if the ray intersects the bounding volume.
         */
        typedef bool (*intersectRayBoundingVolumeFunction_t)(
            const Vec3& ray_origin,
            const Vec3& ray_direction,
            void* bounding_volume_data,
            double& dist);

        /** @brief Device resource holding generic bounding volume data. */
        struct GenericBoundingVolumeData
        {
            /** @brief The type of the bounding volume. */
            enum Type { TYPE_SPHERE } type;
            /** @brief Pointer to the bounding resource. */
            void* data;
            /** @brief Pointer of ray bounding volume intersection function. */
            intersectRayBoundingVolumeFunction_t intersect_ray_bounding_volume;
        };

        /** @brief Container baseclass for generic bounding volumes. */
        class GenericBoundingVolume
        {
        public:
            virtual ~GenericBoundingVolume() { }
            /** @brief Creates a generic bounding volume instance. */
            static std::shared_ptr<GenericBoundingVolume> create(
                std::shared_ptr<IBoundingVolume> bounding_volume);
            /** @brief Provides the resource data. */
            GenericBoundingVolumeData getData();
        protected:
            GenericBoundingVolume() { }
            /** @brief Allocates the resource. */
            virtual bool allocate(
                std::shared_ptr<IBoundingVolume> bounding_volume) = 0;
            /** @brief Returns the resource pointer to the resource data. */
            virtual void* getResourcePtr() = 0;
            /** @brief Returns pointer to the intersection function. */
            virtual intersectRayBoundingVolumeFunction_t getIsecFunction() = 0;
            /** @brief Returns the bounding volume type. */
            virtual GenericBoundingVolumeData::Type getBoundingVolumeType() = 0;
        };

        /** @brief Bounding volume implementation for bounding spheres. */
        class BoundingSphere : public GenericBoundingVolume
        {
        public:
            /** @brief Bounding sphere data. */
            struct Data
            {
                /** @brief Bounding sphere center in particle space. */
                Vec3 center;
                /** @brief Bounding sphere radius in particle space. */
                double radius;
            };
        protected:
            /** @brief Inherited via ldplab::rtscuda::GenericBoundingVolume */
            bool allocate(
                std::shared_ptr<IBoundingVolume> bounding_volume) override;
            /** @brief Inherited via ldplab::rtscuda::GenericBoundingVolume */
            void* getResourcePtr() override;
            /** @brief Inherited via ldplab::rtscuda::GenericBoundingVolume */
            intersectRayBoundingVolumeFunction_t getIsecFunction() override;
            /** @brief Inherited via ldplab::rtscuda::GenericBoundingVolume */
            GenericBoundingVolumeData::Type getBoundingVolumeType() override;
        private:
            /** @brief The intersection kernel. */
            static __device__ bool intersectRayKernel(
                const ldplab::Vec3& ray_origin,
                const ldplab::Vec3& ray_direction,
                void* bounding_volume_data,
                double& dist);
            /** @brief Actual function pointer. */
            static __device__ intersectRayBoundingVolumeFunction_t
                intersect_ray_kernel_ptr;
        private:
            CudaPtr<Data> m_data;
        };
    }
}

#endif // LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#endif // WWU_LDPLAB_RTSCUDA_GENERIC_BOUNDING_VOLUME_HPP