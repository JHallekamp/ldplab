#ifndef WWU_LDPLAB_RTSCUDA_IMPL_GENERIC_GEOMETRY_HPP
#define WWU_LDPLAB_RTSCUDA_IMPL_GENERIC_GEOMETRY_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <LDPLAB/Geometry.hpp>
#include <LDPLAB/RayTracingStep/CUDA/Data.hpp>
#include <LDPLAB/RayTracingStep/CUDA/IGenericGeometry.hpp>

namespace ldplab
{
    namespace rtscuda
    {
        struct GeometrySphereData
        {
            double radius;
        };

        class GenericGeometrySphere :
            public IGenericGeometry
        {
        public:
            GenericGeometrySphere(DeviceBuffer<GeometrySphereData>&& geometry_data);
            void* getDeviceData() override;
            intersectRay getDeviceIntersectRayFunction() override;
            intersectSegment getDeviceIntersectSegmentFunction() override;
        private:
            DeviceBuffer<GeometrySphereData> m_geometry_data;
        };

        struct GeometryRodData
        {
            double cylinder_radius;
            double cylinder_length;
            double sphere_radius;
            Vec3 origin_cap;
            Vec3 origin_indentation;
        };

        class GenericGeometryRod :
            public IGenericGeometry
        {
        public:
            GenericGeometryRod(DeviceBuffer<GeometryRodData>&& geometry_data);
            void* getDeviceData() override;
            intersectRay getDeviceIntersectRayFunction() override;
            intersectSegment getDeviceIntersectSegmentFunction() override;
        private:
            DeviceBuffer<GeometryRodData> m_geometry_data;
        };
    }
}

#endif
#endif