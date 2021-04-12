#ifndef WWU_LDPLAB_RTSCPU_PARTICLE_ACCELERATOR_STRUCTURES_HPP
#define WWU_LDPLAB_RTSCPU_PARTICLE_ACCELERATOR_STRUCTURES_HPP

#include <limits>
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
            /** @brief Constructs the octree. */
            void construct(
                const std::vector<Triangle>& mesh,
                size_t octree_depth);
        private:
            /** @brief Saves the data of an octree node. */
            struct OctreeNode
            {
                /** @brief The bounding box describing the nodes borders */
                AABB aabb;
                /** 
                 * @brief Saves the index to the vector contianing the children
                 *        of the node.
                 * @details Depending on the depth of the node, which has to be
                 *          tracked by the querying algorithm, this index can
                 *          either be in the vector of OctreeNode vectors or in
                 *          the vector of Triangle vectors. If the node is 
                 *          empty, the index is set to invalid.
                 */
                size_t children_vector_index;
            };
        private:
            AABB m_octree_aabb;
            size_t m_root_node_vector_index;
            std::vector<std::vector<OctreeNode>> m_node_vectors;
            std::vector<std::vector<Triangle>> m_triangle_vectors;
        };
    }
}

#endif