#ifndef WWU_LDPLAB_RTSCUDA_GENERIC_PARTICLE_GEOMETRY_HPP
#define WWU_LDPLAB_RTSCUDA_GENERIC_PARTICLE_GEOMETRY_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <LDPLAB/ExperimentalSetup/ParticleGeometry.hpp>
#include <LDPLAB/Geometry.hpp>
#include <memory>

#include "CudaResource.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        /**
         * @brief Typedefinition of particle geometry intersection kernel
         *        function pointer.
         * @param[in] ray_origin The ray origin in particle space.
         * @param[in] ray_direction The ray direction in particle space.
         * @param[in] particle_geometry_data Pointer to the particle geometry
         *                                   data.
         * @param[out] intersection_point If ray intersects the geometry,
         *                                then intersection_point will
         *                                contain the point.
         * @param[out] intersection_normal If ray intersects the geometry,
         *                                 then intersection_normal will
         *                                 contain the geometry surface
         *                                 normal at the point of
         *                                 intersection.
         * @param[out] dist The distance to the intersection point. This is
         *                  meaningless if the method returns false.
         * @param[out] intersects_outside Tells the caller weather the 
         *                                intersection occured in- or
         *                                outside of the particle. This is
         *                                meaningless if the method returns
         *                                false.
         * @returns true, if the ray intersects the bounding volume.
         */
        typedef bool (*intersectRayParticleGeometryFunction_t)(
            const Vec3& ray_origin,
            const Vec3& ray_direction,
            void* particle_geometry_data,
            Vec3& intersection_point,
            Vec3& intersection_normal,
            double& dist,
            bool& intersects_outside);

        /** 
         * @brief Device side wrapper methods around a generic ray particle
         *        intersection function.
         */
        class GenericParticlFunctionWrapper
        {
        public:
            GenericParticlFunctionWrapper() = delete;
            /**
             * @brief Typedefinition of particle geometry intersection kernel
             *        function pointer.
             * @param[in] function Function pointer to the implemented generic
             *                     intersection function.
             * @param[in] ray_origin The ray origin in particle space.
             * @param[in] ray_direction The ray direction in particle space.
             * @param[in] particle_geometry_data Pointer to the particle
             *                                   geometry data.
             * @param[out] intersection_point If ray intersects the geometry,
             *                                then intersection_point will
             *                                contain the point.
             * @param[out] intersection_normal If ray intersects the geometry,
             *                                 then intersection_normal will
             *                                 contain the geometry surface
             *                                 normal at the point of
             *                                 intersection.
             * @returns true, if the ray intersects the bounding volume.
             */
            static __device__ bool intersectRay(
                intersectRayParticleGeometryFunction_t function,
                const Vec3& ray_origin,
                const Vec3& ray_direction,
                void* particle_geometry_data,
                Vec3& intersection_point,
                Vec3& intersection_normal);
            /**
             * @brief Typedefinition of particle geometry intersection kernel
             *        function pointer.
             * @param[in] function Function pointer to the implemented generic
             *                     intersection function.
             * @param[in] ray_origin The ray origin in particle space.
             * @param[in] ray_direction The ray direction in particle space.
             * @param[in] particle_geometry_data Pointer to the particle 
             *                                   geometry data.
             * @param[out] intersection_point If ray intersects the geometry,
             *                                then intersection_point will
             *                                contain the point.
             * @param[out] intersection_normal If ray intersects the geometry,
             *                                 then intersection_normal will
             *                                 contain the geometry surface
             *                                 normal at the point of
             *                                 intersection.
             * @param[out] intersects_outside Tells the caller weather the
             *                                intersection occured in- or
             *                                outside of the particle. This is
             *                                meaningless if the method returns
             *                                false.
             * @returns true, if the ray intersects the bounding volume.
             */
            static __device__ bool intersectRay(
                intersectRayParticleGeometryFunction_t function,
                const Vec3& ray_origin,
                const Vec3& ray_direction,
                void* particle_geometry_data,
                Vec3& intersection_point,
                Vec3& intersection_normal,
                bool& intersects_outside);
            /**
             * @brief Typedefinition of particle geometry intersection kernel
             *        function pointer.
             * @param[in] function Function pointer to the implemented generic
             *                     intersection function.
             * @param[in] ray_origin The ray origin in particle space.
             * @param[in] ray_direction The ray direction in particle space.
             * @param[in] particle_geometry_data Pointer to the particle
             *                                   geometry data.
             * @param[out] intersection_normal If the line segment intersects
             *                                 the geometry, then
             *                                 intersection_normal will contain
             *                                 the geometry surface normal at
             *                                 the point of intersection.
             * @param[out] end_point_inside Tells the caller that segment end
             *                              point is inside or outside of the
             *                              particle. This is needed by the
             *                              inner particle propagation to 
             *                              identify rays that leave the
             *                              particle due to the gradient in 
             *                              the first step.
             * @returns true, if the line segment does intersect the geometry
             *          and is not fully inside or outside of it.
             * @note All vectors, be it in- or output (including those in the
             *       given ray) are assumed to be given in the same coordinate
             *       system as the underlying geometry. The caller has to make
             *       sure that this assumption is never violated.
             */
            static __device__ bool intersectSegment(
                intersectRayParticleGeometryFunction_t function,
                const Vec3& segment_origin,
                const Vec3& segment_end,
                void* particle_geometry_data,
                Vec3& intersection_point,
                Vec3& intersection_normal,
                bool& end_point_inside);
        };

        /** @brief Device resource holding generic particle geometry data. */
        struct GenericParticleGeometryData
        {
            /** @brief The type of the particle geometry. */
            enum Type { TYPE_ROD, TYPE_SPHERE } type;
            /** @brief Pointer to the particle geometry data. */
            void* data;
            /** @brief Pointer of ray particle intersection function. */
            intersectRayParticleGeometryFunction_t intersect_ray_particle;
        };

        /** @brief Container baseclass for generic particle geometries. */
        class GenericParticleGeometry
        {
        public:
            virtual ~GenericParticleGeometry() { }
            /** @brief Creates a generic particle geometry instance. */
            static std::shared_ptr<GenericParticleGeometry> create(
                std::shared_ptr<IParticleGeometry> particle_geometry);
            /** @brief Provides the resource data. */
            GenericParticleGeometryData getData();
        protected:
            /** @brief Allocates the resource. */
            virtual bool allocate(
                std::shared_ptr<IParticleGeometry> particle_geometry) = 0;
            /** @brief Returns the resource pointer to the resource data. */
            virtual void* getResourcePtr() = 0;
            /** @brief Returns pointer to the intersection function. */
            virtual intersectRayParticleGeometryFunction_t getIsecFunction() = 0;
            /** @brief Returns the particle geometry type. */
            virtual GenericParticleGeometryData::Type getGeometryType() = 0;
        };

        /** @brief Geometry implementation for rod particles. */
        class RodParticle : public GenericParticleGeometry
        {
        public:
            /** @brief Rod particle data. */
            struct Data
            {
                double cylinder_radius;
                double cylinder_length;
                double sphere_radius;
                Vec3 origin_cap;
                Vec3 origin_indentation;
            };
        protected:
            /** @brief Inherited via ldplab::rtscuda::GenericParticleGeometry. */
            bool allocate(
                std::shared_ptr<IParticleGeometry> particle_geometry) override;
            /** @brief Inherited via ldplab::rtscuda::GenericParticleGeometry. */
            void* getResourcePtr() override;
            /** @brief Inherited via ldplab::rtscuda::GenericParticleGeometry. */
            intersectRayParticleGeometryFunction_t getIsecFunction() override;
            /** @brief Inherited via ldplab::rtscuda::GenericParticleGeometry. */
            GenericParticleGeometryData::Type getGeometryType() override;
        private:
            /** @brief Intersection kernel. */
            static __device__ bool intersectRayKernel(
                const Vec3& ray_origin,
                const Vec3& ray_direction,
                void* particle_geometry_data,
                Vec3& intersection_point,
                Vec3& intersection_normal,
                double& dist,
                bool& intersects_outside);
            /** @brief Actual function pointer. */
            static __device__ intersectRayParticleGeometryFunction_t
                intersect_ray_kernel_ptr;
        private:
            CudaPtr<Data> m_data;
        };

        /** @brief Geometry implementation for sphere particles. */
        class SphereParticle : public GenericParticleGeometry
        {
        public:
            /** @brief Rod particle data. */
            struct Data
            {
                double radius;
            };
        protected:
            /** @brief Inherited via ldplab::rtscuda::GenericParticleGeometry. */
            bool allocate(
                std::shared_ptr<IParticleGeometry> particle_geometry) override;
            /** @brief Inherited via ldplab::rtscuda::GenericParticleGeometry. */
            void* getResourcePtr() override;
            /** @brief Inherited via ldplab::rtscuda::GenericParticleGeometry. */
            intersectRayParticleGeometryFunction_t getIsecFunction() override;
            /** @brief Inherited via ldplab::rtscuda::GenericParticleGeometry. */
            GenericParticleGeometryData::Type getGeometryType() override;
        private:
            /** @brief Intersection kernel. */
            static __device__ bool intersectRayKernel(
                const Vec3& ray_origin,
                const Vec3& ray_direction,
                void* particle_geometry_data,
                Vec3& intersection_point,
                Vec3& intersection_normal,
                double& dist,
                bool& intersects_outside);
            /** @brief Actual function pointer. */
            static __device__ intersectRayParticleGeometryFunction_t
                intersect_ray_kernel_ptr;
        private:
            CudaPtr<Data> m_data;
        };
    }
}

#endif // LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#endif // WWU_LDPLAB_RTSCUDA_GENERIC_PARTICLE_GEOMETRY_HPP