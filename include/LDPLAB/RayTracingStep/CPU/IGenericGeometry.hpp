#ifndef WWU_LDPLAB_RTSCPU_I_GENERIC_GEOMETRY_HPP
#define WWU_LDPLAB_RTSCPU_I_GENERIC_GEOMETRY_HPP

#include <LDPLAB/Constants.hpp>
#include <LDPLAB/Geometry.hpp>

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
                Vec3& intersection_normal) const
            {
                double td;
                bool to;
                return intersectRay(
                    ray, 
                    intersection_point, 
                    intersection_normal, 
                    td, 
                    to);
            }
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
             * @param[out] intersects_outside Tells the caller weather the
             *                                intersection occured in- or
             *                                outside of the particle. This is
             *                                meaningless if the method returns
             *                                false.
             * @returns true, if ray does intersect the particle.
             * @note All vectors, be it in- or output (including those in the
             *       given ray) are assumed to be given in the same coordinate
             *       system as the underlying geometry. The caller has to make
             *       sure that this assumption is never violated.
             */
            virtual bool intersectRay(
                const Ray& ray,
                Vec3& intersection_point,
                Vec3& intersection_normal,
                bool& intersects_outside) const
            {
                double t;
                return intersectRay(
                    ray,
                    intersection_point,
                    intersection_normal,
                    t,
                    intersects_outside);
            }
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
            virtual bool intersectSegment(
                const Vec3& segment_origin,
                const Vec3& segment_end,
                Vec3& intersection_point,
                Vec3& intersection_normal,
                bool& end_point_inside) const
            {
                const Vec3 seg = segment_end - segment_origin;
                const double seg_length = glm::length(seg);
                const Ray ray{ segment_origin, seg / seg_length, -1 };
                double dist = 0;
                if (!intersectRay(
                    ray,
                    intersection_point, 
                    intersection_normal, 
                    dist, 
                    end_point_inside))
                {
                    end_point_inside = false;
                    return false;
                }
                else
                {
                    if (dist <= seg_length + constant::intersection_tests::epsilon)
                        return true;
                    end_point_inside = !end_point_inside;
                    return false;
                }
            }
        protected:
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
             * @param[out] dist The distance to the intersection point. This is
             *                  meaningless if the method returns false.
             * @param[out] intersects_outside Tells the caller weather the
             *                                intersection occured in- or
             *                                outside of the particle. This is
             *                                meaningless if the method returns
             *                                false.
             * @returns true, if ray does intersect the particle.
             * @note All vectors, be it in- or output (including those in the
             *       given ray) are assumed to be given in the same coordinate
             *       system as the underlying geometry. The caller has to make
             *       sure that this assumption is never violated.
             */
            virtual bool intersectRay(
                const Ray& ray,
                Vec3& intersection_point,
                Vec3& intersection_normal,
                double& dist,
                bool& intersects_outside) const = 0;
        };
    }
}

#endif
