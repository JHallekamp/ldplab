#ifndef WWU_LDPLAB_RTSCPU_ACCELERATOR_STRUCTURE_HPP
#define WWU_LDPLAB_RTSCPU_ACCELERATOR_STRUCTURE_HPP

#include <memory>
#include <vector>

#include <LDPLAB/Geometry.hpp>
#include <LDPLAB/ExperimentalSetup/ParticleGeometry.hpp>

namespace ldplab
{
    namespace rtscpu
    {
        struct Context;

        class IAcceleratorStructure
        {
        public:
            /**
             * @brief  
             */
            virtual bool Intersection(
                const Ray ray,
                Vec3& intersection_point,
                Vec3& intersection_normal) = 0;
        };

        class TriangleList : 
            public IAcceleratorStructure
        {
        public:
            TriangleList(std::vector<Triangle> mesh);
            bool Intersection(
                const Ray ray,
                Vec3& intersection_point,
                Vec3& intersection_normal) override;
        private:
            /**
             * @brief Calcutes the intersection of a ray with the triangle.
             * @details Using the Möller-Trumbore intersection algorithm.
             * @returns Returns the distance between the ray origin and
             *          the intersecion point. For no intersection -1 will be
             *          returned.
             */
            double RayTriangleIntersection(const Ray& ray, 
                const Triangle& triangle);
        private:
            std::vector<Triangle> m_mesh;
        };
    }
}

#endif