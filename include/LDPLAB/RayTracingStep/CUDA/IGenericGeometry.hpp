#ifndef WWU_LDPLAB_RTSCUDA_I_GENERIC_GEOMETRY_HPP
#define WWU_LDPLAB_RTSCUDA_I_GENERIC_GEOMETRY_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

namespace ldplab
{
    namespace rtscuda
    {
        class IGenericGeometry
        {
        public:
            /**
             * @brief Function pointer type for a function that tests an 
             *        intersection between a ray and the generic geometry and
             *        computes the intersection point and the surface normal at
             *        that point.
             * @param[in] ray_origin Origin of the tested ray.
             * @param[in] ray_direction Direction of the tested ray.
             * @param[in] particle_geometry Implementation specific particle
             *                              geometry data.
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
            typedef bool (*intersectRay)(
                const Vec3& ray_origin,
                const Vec3& ray_direction,
                const void* particle_geometry,
                Vec3& intersection_point,
                Vec3& intersection_normal,
                bool& intersects_outside);
            /**
             * @brief Function pointer type for a function that tests an 
             *        intersection between a line segment and the generic
             *        geometry hull. Computes the intersection point and
             *        surface normal at such an intersection.
             * @param[in] segment_origin The origin point of the line segment.
             * @param[in] segment_end The end point of the line segment.
             * @param[in] particle_geometry Implementation specific particle
             *                              geometry data.
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
            typedef bool (*intersectSegment)(
                const Vec3& segment_origin,
                const Vec3& segment_end,
                const void* geometry_data,
                Vec3& intersection_point,
                Vec3& intersection_normal,
                bool& end_point_inside);
        };
    }
}

#endif
#endif