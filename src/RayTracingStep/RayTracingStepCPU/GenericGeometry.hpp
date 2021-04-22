#ifndef WWU_LDPLAB_RTSCPU_GENERIC_GEOMETRY_HPP
#define WWU_LDPLAB_RTSCPU_GENERIC_GEOMETRY_HPP

#include <LDPLAB/ExperimentalSetup/ParticleGeometry.hpp>
#include <LDPLAB/Geometry.hpp>

#include <memory>

namespace ldplab
{
    namespace rtscpu
    {
        /**
         * @brief Interface containing abstract intersection queries. 
         * @details Enables the pipeline to test for intersection with the
         *          underlying geometry without knowing the actual data.
         */
        class IGenericGeometry
        {
        public:
            /** @brief Virtual destructor. */
            virtual ~IGenericGeometry() { }
            /**
             * @brief Tests an intersection between a ray and the geometry and
             *        computes the intersection point and the surface normal at
             *        that point.
             * @param[in] ray The ray for which the intersection test is
             *                computed.
             * @param[out] intersection_point If ray intersects the geometry,
             *                                then intersection_point will
             *                                contain the point.
             * @param[out] intersection_normal If ray intersects the geometry,
             *                                 then intersection_normal will
             *                                 contain the geometry surface
             *                                 normal at the point of
             *                                 intersection.
             * @returns true, if ray does intersect the particle.
             * @note All vectors, be it in- or output (including those in the
             *       given ray) are assumed to be given in the same coordinate
             *       system as the underlying geometry. The caller has to make
             *       sure that this assumption is never violated.
             */
            virtual bool intersectRay(
                const Ray& ray,
                Vec3& intersection_point,
                Vec3& intersection_normal) = 0;
            /**
             * @brief Tests an intersection between a line segment and the
             *        geometry hull. Computes the intersection point and 
             *        surface normal at such an intersection.
             * @param[in] segment_origin The origin point of the line segment.
             * @param[in] segment_end The end point of the line segment.
             * @param[out] intersection_point If the line segment intersects
             *                                the geometry, then
             *                                intersection_point will contain
             *                                the point at which the given
             *                                intersection occurs.
             * @param[out] intersection_normal If the line segment intersects
             *                                 the geometry, then
             *                                 intersection_normal will contain
             *                                 the geometry surface normal at
             *                                 the point of intersection.
             * @returns true, if the line segment does intersect the geometry
             *          and is not fully inside or outside of it.
             * @note All vectors, be it in- or output (including those in the
             *       given ray) are assumed to be given in the same coordinate
             *       system as the underlying geometry. The caller has to make
             *       sure that this assumption is never violated.
             */
            virtual bool intersectSegment(
                const Vec3& segment_origin,
                const Vec3& segment_end,
                Vec3& intersection_point,
                Vec3& intersection_normal) = 0;
        };

        /** @brief Geometry implementation for rod particles. */
        class RodGeometry : public IGenericGeometry
        {
        public:
            /** @brief Constructs the rod particle geometry. */
            RodGeometry(const RodParticleGeometry* geometry);
            /** @brief Inherited via IGenericGeometry */
            bool intersectRay(
                const Ray& ray,
                Vec3& intersection_point,
                Vec3& intersection_normal) override;
            /** @brief Inherited via IGenericGeometry */
            bool intersectSegment(
                const Vec3& segment_origin,
                const Vec3& segment_end,
                Vec3& intersection_point,
                Vec3& intersection_normal) override;
        private:
            /** 
             * @brief Checks against the infinite cylinder.
             * @param[in] ray The ray to be tested.
             * @param[out] dist_min The minimum intersection distance. Can be
             *                      negative, if the ray origin lies within the
             *                      cylinder. If the ray origin lies within the
             *                      cylinder and the ray direction is parallel
             *                      to the cylinder, dist_min equals negative
             *                      infinity.
             * @param[out] dist_max The maximum intersection distance. If the
             *                      origin lies within the cylinder and the
             *                      ray direction is parallel to the cylinder,
             *                      dist_max equals infinity.
             * @returns true, if the ray intersects the infinite cylinder.
             */
            bool overlapCylinder(
                const Ray& ray,
                double& dist_min,
                double& dist_max);
            /** @brief Test inside the cylinder. */
            bool intersectInsideCylinder(
                const Ray& ray,
                Vec3& intersection_point,
                Vec3& intersection_normal,
                double& isec_dist);
            /** @brief Test outside the cylinder. */
            bool intersectOutsideCylinder(
                const Ray& ray,
                double min_dist,
                Vec3& intersection_point,
                Vec3& intersection_normal,
                double& isec_dist);
            /** @brief Intersection with xy plane, if kappa is to small. */
            bool intersectTopBottomPlane(
                const Ray& ray,
                double z,
                Vec3& intersection_point,
                Vec3& intersection_normal,
                double& isec_dist);
        private:
            double m_cylinder_radius;
            double m_cylinder_length;
            double m_sphere_radius;
            Vec3 m_origin_cap;
            Vec3 m_origin_indentation;
        };

        /** @brief Geometry implementation for spherical particles. */
        class SphericalGeometry : public IGenericGeometry
        {
        public:
            /** @brief Constructs the sphere particle geometry. */
            SphericalGeometry(const SphericalParticleGeometry* geometry);
            /** @brief Inherited via IGenericGeometry */
            bool intersectRay(
                const Ray& ray,
                Vec3& intersection_point,
                Vec3& intersection_normal) override;
            /** @brief Inherited via IGenericGeometry */
            bool intersectSegment(
                const Vec3& segment_origin,
                const Vec3& segment_end,
                Vec3& intersection_point,
                Vec3& intersection_normal) override;
        private:
            double m_radius;
        };
    }
}

#endif