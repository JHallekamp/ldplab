#ifndef WWU_LDPLAB_RTSCPU_IMPL_GENERIC_GEOMETRY_HPP
#define WWU_LDPLAB_RTSCPU_IMPL_GENERIC_GEOMETRY_HPP

#include <LDPLAB/ExperimentalSetup/ParticleGeometry.hpp>
#include <LDPLAB/RayTracingStep/CPU/IGenericGeometry.hpp>

namespace ldplab
{
    namespace rtscpu
    {
        class RodGeometry : public IGenericGeometry
        {
        public:
            /** @brief Constructs the rod particle geometry. */
            RodGeometry(const RodParticleGeometry* geometry);
        protected:
            virtual bool intersectRay(
                const Ray& ray, 
                Vec3& intersection_point, 
                Vec3& intersection_normal, 
                double& dist, 
                bool& intersects_outside) const override;
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
                double max_dist,
                Vec3& intersection_point,
                Vec3& intersection_normal,
                double& isec_dist,
                bool& intersects_outside);
            /** @brief Test outside the cylinder. */
            bool intersectOutsideCylinder(
                const Ray& ray,
                double min_dist,
                Vec3& intersection_point,
                Vec3& intersection_normal,
                double& isec_dist);
            /** @brief Intersection with rod particle cap. */
            bool intersectCap(
                const Ray& ray,
                bool inside_cylinder,
                Vec3& intersection_point,
                Vec3& intersection_normal,
                double& isec_dist,
                bool& intersects_outside);
            /** @brief Intersection with rod particle indent. */
            bool intersectIndent(
                const Ray& ray,
                bool inside_cylinder,
                Vec3& intersection_point,
                Vec3& intersection_normal,
                double& isec_dist,
                bool& intersects_outside);
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
        protected:
            /** @brief Inherited via IGenericGeometry */
            bool intersectRay(
                const Ray& ray,
                Vec3& intersection_point,
                Vec3& intersection_normal,
                double& dist,
                bool& intersects_outside) const override;
        private:
            double m_radius;
        };
    }
}

#endif