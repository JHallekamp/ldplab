#ifndef WWU_LDPLAB_RTSCPU_PARTICLE_ACCELERATOR_STRUCTURES_HPP
#define WWU_LDPLAB_RTSCPU_PARTICLE_ACCELERATOR_STRUCTURES_HPP

#include <limits>
#include <memory>
#include <vector>

#include <LDPLAB/Geometry.hpp>
#include <LDPLAB/ExperimentalSetup/ParticleGeometry.hpp>

#include "../../Utils/Array.hpp"

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

        /**
         * @brief Stores the particle mesh triangles as a simple list.
         * @details An intersection lookup consists of traversing the whole
         *          list and checking each triangle individually.
         */
        class ParticleMeshList : public IParticleAcceleratorStructure
        {
        public:
            ParticleMeshList(const std::vector<Triangle>& mesh);
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

        /**
         * @brief Stores the particle mesh triangles in an octree structure.
         */
        class ParticleMeshOctree : public IParticleAcceleratorStructure
        {
        public:
            ParticleMeshOctree(
                const std::vector<Triangle>& mesh,
                size_t octree_depth);
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
            /** @brief Saves the data of an octree node. */
            struct OctreeNode
            {
                OctreeNode();
                /** @brief The bounding box describing the nodes borders */
                AABB aabb;
                /** @brief Number of divisions. */
                size_t num_children;
                /**
                 * @brief Child indices, also triangle array indices on leaf
                 *        nodes;
                 */
                size_t children[8];
            };
        private:
            /** @brief Constructs the octree. */
            void construct(
                const std::vector<Triangle>& mesh,
                size_t octree_depth);
            /** @brief Intersection test recursive. */
            bool intersectRecursive(
                const OctreeNode& node,
                const size_t depth,
                const Ray& ray,
                Vec3& intersection_point,
                Vec3& intersection_normal);
            bool intersectBase(
                const utils::Array<Triangle>& triangles,
                const Ray& ray,
                Vec3& intersection_point,
                Vec3& intersection_normal);
            bool intersectSegmentRecursive(
                const OctreeNode& node,
                const size_t depth,
                const Vec3& segment_origin,
                const Vec3& segment_end,
                Vec3& intersection_point,
                Vec3& intersection_normal);
            bool intersectSegmentBase(
                const utils::Array<Triangle>& triangles,
                const Vec3& segment_origin,
                const Vec3& segment_end,
                Vec3& intersection_point,
                Vec3& intersection_normal);
        private:
            size_t m_octree_depth;
            std::vector<OctreeNode> m_nodes;
            std::vector<utils::Array<Triangle>> m_triangle_arrays;
        };
    }
}

#endif