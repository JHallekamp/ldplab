#ifndef WWU_LDPLAB_RTSCUDA_PARTICLE_ACCELERATOR_STRUCTURES_HPP
#define WWU_LDPLAB_RTSCUDA_PARTICLE_ACCELERATOR_STRUCTURES_HPP

#include <limits>
#include <memory>
#include <vector>

#include <LDPLAB/Geometry.hpp>
#include <LDPLAB/ExperimentalSetup/ParticleGeometry.hpp>
#include <LDPLAB/RayTracingStep/AcceleratorStructureParameter.hpp>

#include "GenericGeometry.hpp"
#include "../../Utils/Array.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        /** 
         * @brief Interface for accelerator structures.
         * @tparam InData Type of the data that is given to the accelerator
         *                structure to construct it.
         */
        template <typename InData>
        class IAcceleratorStructre : public IGenericGeometry
        {
        public:
            virtual ~IAcceleratorStructre() { }
            /**
             * @brief Constructs the accelerator structure.
             * @param[in] data The data used to construct the accelerator 
             *                 structure.
             * @param[in] parameter The accelerator structure parameters.
             * @returns true, if the construction was successful.
             */
            virtual bool construct(
                const InData& data, 
                const IAcceleratorStructureParameter* parameter) = 0;
        };

        /** 
         * @brief Accelerator structure interface for structures managing 
         *        triangle meshes.
         */
        class ITriangleMeshAcceleratorStructure :
            public IAcceleratorStructre<std::vector<Triangle>>
        {
        public:
            virtual ~ITriangleMeshAcceleratorStructure() { }
            /** Inherited via IAcceleratorStructre */
            bool construct(
                const std::vector<Triangle>& data,
                const IAcceleratorStructureParameter* parameter) override
            { return constructInternal(data, parameter); }
        protected:
            virtual bool constructInternal(
                const std::vector<Triangle>& mesh,
                const IAcceleratorStructureParameter* parameter) = 0;
        };

        /**
         * @brief Stores the particle mesh triangles as a simple list.
         * @details An intersection lookup consists of traversing the whole
         *          list and checking each triangle individually.
         */
        class TriangleMeshGeometryList : 
            public ITriangleMeshAcceleratorStructure
        {
        protected:
            /** @brief Inherited via IGenericGeometry */
            bool intersectRay(
                const Ray& ray,
                Vec3& intersection_point,
                Vec3& intersection_normal,
                double& dist,
                bool& intersect_outside) override;
            /** @brief Inherited via ITriangleMeshAcceleratorStructure */
            bool constructInternal(
                const std::vector<Triangle>& mesh,
                const IAcceleratorStructureParameter* parameter) override;
        private:
            std::vector<Triangle> m_mesh;
        };

        /**
         * @brief Stores the particle mesh triangles in an octree structure.
         */
        class TriangleMeshGeometryOctree : 
            public ITriangleMeshAcceleratorStructure
        {
        protected:
            /** @brief Inherited via IGenericGeometry */
            bool intersectRay(
                const Ray& ray,
                Vec3& intersection_point,
                Vec3& intersection_normal,
                double& dist,
                bool& intersect_outside) override;
            /** @brief Inherited via ITriangleMeshAcceleratorStructure */
            bool constructInternal(
                const std::vector<Triangle>& mesh,
                const IAcceleratorStructureParameter* parameter) override;
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
            /** @brief Helper methods. */
            inline size_t pow8(
                size_t exp) const noexcept;
            inline size_t mapIndexP2C(
                size_t parent_idx,
                size_t child_no) const noexcept;
            inline size_t mapIndexC2P(
                size_t child_idx) const noexcept;
            inline size_t mapIndexGetChildNo(
                size_t child_idx) const noexcept;
            /** @brief Constructs the octree. */
            AABB constructOctreeAABB(
                const std::vector<Triangle>& mesh);
            void constructConstructionLayers(
                const AABB& octree_aabb,
                std::vector<std::vector<OctreeNode>>& layers);
            bool constructSortTrianglesRecursive(
                const Triangle& triangle,
                size_t current_layer,
                size_t current_node,
                std::vector<std::vector<OctreeNode>>& layers,
                std::vector<std::vector<Triangle>>& triangle_storage);
            size_t constructMakePersistentRecursive(
                const size_t current_layer,
                const size_t current_node,
                const std::vector<std::vector<OctreeNode>>& layers,
                const std::vector<std::vector<Triangle>>& triangle_storage);
            /** @brief Intersection test recursive. */
            bool intersectRecursive(
                const OctreeNode& node,
                const size_t depth,
                const Ray& ray,
                Vec3& intersection_point,
                Vec3& intersection_normal,
                bool& intersect_outside);
            bool intersectBase(
                const utils::Array<Triangle>& triangles,
                const Ray& ray,
                Vec3& intersection_point,
                Vec3& intersection_normal,
                bool& intersect_outside);
        private:
            size_t m_octree_depth;
            std::vector<OctreeNode> m_nodes;
            std::vector<utils::Array<Triangle>> m_triangle_arrays;
        };
    }
}

#endif