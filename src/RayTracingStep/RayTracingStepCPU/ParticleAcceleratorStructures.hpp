#ifndef WWU_LDPLAB_RTSCPU_PARTICLE_ACCELERATOR_STRUCTURES_HPP
#define WWU_LDPLAB_RTSCPU_PARTICLE_ACCELERATOR_STRUCTURES_HPP

#include <memory>
#include <vector>

#include <LDPLAB/Geometry.hpp>
#include <LDPLAB/ExperimentalSetup/ParticleGeometry.hpp>

namespace ldplab
{
    namespace rtscpu
    {
        /** 
         * @brief Abstract baseclass for accelerator structures containing 
         *        particle data.
         * @details Particle accelerator structures are used to compute
         *          intersections with a specific particles.
         */
        class IParticleAcceleratorStructure
        {
        public:
            /**
             * @brief Tests an intersection between a ray and the particle and
             *        computes the intersection point and the surface normal at
             *        that point.
             * @param[in] ray The ray for which the intersection test is 
             *                computed.
             * @param[out] intersection_point If ray intersects the particle,
             *                                then intersection_point will 
             *                                contain the point in particle
             *                                space at which the given 
             *                                intersection occurs.
             * @param[out] intersection_normal If ray intersects the particle,
             *                                 then intersection_normal will
             *                                 contain the particle surface
             *                                 normal at the point of
             *                                 intersection.
             * @returns true, if ray does intersect the particle.
             */
            virtual bool intersects(
                const Ray& ray,
                Vec3& intersection_point,
                Vec3& intersection_normal) = 0;
            /**
             * @brief Tests an intersection between a line segment and the
             *        particle and computes the intersection point and surface
             *        normal at that point.
             * @param[in] segment_origin The origin point of the line segment.
             * @param[in] segment_end The end point of the line segment.
             * @param[out] intersection_point If the line segment intersects 
             *                                the particle, then 
             *                                intersection_point will contain
             *                                the point in particle space at 
             *                                which the given intersection 
             *                                occurs.
             * @param[out] intersection_normal If the line segment intersects 
             *                                 the particle, then 
             *                                 intersection_normal will contain
             *                                 the particle surface normal at 
             *                                 the point of intersection.
             * @returns true, if the line segment does intersect the particle.
             */
            virtual bool intersectsLineSegment(
                const Vec3& segment_origin,
                const Vec3& segment_end,
                Vec3& intersection_point,
                Vec3& intersection_normal) = 0;
        };

        class ParticleMeshTriangleList : public IParticleAcceleratorStructure
        {
        public:
            ParticleMeshTriangleList(const std::vector<Triangle>& mesh);
            /** @brief Inherited via IParticleAcceleratorStructure */
            bool intersects(
                const Ray& ray,
                Vec3& intersection_point,
                Vec3& intersection_normal) override;
            /** @brief Inherited via IParticleAcceleratorStructure */
            bool intersectsLineSegment(
                const Vec3& segment_origin,
                const Vec3& segment_end,
                Vec3& intersection_point,
                Vec3& intersection_normal) override;
        private:
            std::vector<Triangle> m_mesh;
        };
    }
}

#endif