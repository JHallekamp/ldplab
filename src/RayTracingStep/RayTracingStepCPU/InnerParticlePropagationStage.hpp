#ifndef WWU_LDPLAP_RTSCPU_INNER_PARTICLE_PROPAGATION_STAGE_HPP
#define WWU_LDPLAP_RTSCPU_INNER_PARTICLE_PROPAGATION_STAGE_HPP

#include <memory>
#include <vector>

#include "Data.hpp"
#include <LDPLAB/RayTracingStep/EikonalSolver.hpp>
#include <LDPLAB/Geometry.hpp>

namespace ldplab
{
    // Prototype
    struct Particle;
    struct ParticleMaterialLinearOneDirectional;
    struct SphericalParticleGeometry;
    namespace rtscpu
    {
        // Prototype
        struct Context;
        struct RayBuffer;
        struct IntersectionBuffer;
        struct OutputBuffer;
        struct RodParticle;

        /**
         * @brief Inner particle propagation stage interface
         * @detail The inner particle propagation stage is responsible for
         *         calculating the path threw the particle and the resulting
         *         momentum difference.
         */
        class IInnerParticlePropagationStage
        {
        public:
            /**
             * @brief Calculating the path of the rays threw the particle.
             * @param[in, out] rays RayBuffer holding the propagating rays.
             * @param[out] intersection IntersectionBuffer holding information 
             *             about the intersection points.
             * @param[in, out] output Buffer holding the resulting force and 
             *                        torque change of each particle.
             */
            virtual void execute(
                RayBuffer& rays,
                IntersectionBuffer& intersection,
                OutputBuffer& output) = 0;
        };

        class IInnerParticlePropagationParticle
        {
        protected:
            /**
             * @brief Check if the position is outside of the particle.
             * @param[in] particle Index of the particle.
             * @param[in] r Position to check.
             * @retuns true if the position is outside the particle, false if
             *         the position is inside.
             */
            virtual bool isOutsideParticle(const size_t particle, const Vec3& r) = 0;
            /**
             * @brief Calculating the intersection of the ray and the particle.
             * @param[in] particle Index of the particle.
             * @param[in] ray_origin_in Last ray position inside the particle.
             * @param[in] ray_origin_out First ray position outside the particle.
             * @param[out] inter_point Resulting intersection point with
             *                         the particle surface.
             * @param[out] inter_normal Resulting normal of the particle
             *                          surface at the intersection
             *                          point. The normal is pointing
             *                          inside the particle.
             */
            virtual void intersection(
                const size_t particle,
                const Vec3& ray_origin_in,
                const Vec3& ray_origin_out,
                Vec3& inter_point,
                Vec3& inter_normal) = 0;
        };

        /**
        * @brief Class implementing the inner particle propagation for
        *        linear index of refraction gradient in one direction.
        * @detail The light propagation is calculated by solving the Eikonal
        *         equation with the Runge�Kutta method.
        */
        class EikonalSolverRK4 : public IInnerParticlePropagationStage, IInnerParticlePropagationParticle
        {
        protected:
            /**
             * @brief Constructing inner particle propagation stage and setting
             *        up the parameter for the Runge-Kutta method.
             * @param context Pointer to context data for the ray tracing step.
             * @param parameters Structure containing all parameter for the
             *                   Runge-Kutta method.
             */
            EikonalSolverRK4(
                Context& context,
                RK4 parameters);
        public:
            /**
             * @brief Inherited via ldplab::rtscpu::IInnerParticlePropagationStage.
             * @details Calculating the path of the rays threw the particle.
             * @param[in, out] rays RayBuffer holding the propagating rays.
             * @param[out] intersection IntersectionBuffer holding information
             *             about the intersection points.
             * @param[in, out] output Buffer holding the resulting force and torque
             *                        change of each particle.
             */
            void execute(
                RayBuffer& rays,
                IntersectionBuffer& intersection,
                OutputBuffer& output) override;
        protected:
            /**
             * @brief Structure keeping all variables of the differential
             *        equation.
             */
            struct Arg
            {
                /**
                 * @brief Vector pointing in the direction of light. Its norm
                 *        is the index of reflection at position r.
                 */
                Vec3 w;
                /**
                 * @brief Vector pointing to the light rays origin.
                 */
                Vec3 r;
                inline Arg operator*(const double& d) const
                {
                    return Arg{ w * d, r * d };
                }
                inline void operator*=(const double& d)
                {
                    w *= d;
                    r *= d;
                }
                inline void operator+=(const Arg& rhs)
                {
                    w += rhs.w;
                    r += rhs.r;
                }
            };
        private:
            /**
             * @brief Calculating the ray propagation through the particle.
             * @detail The ray propagation is integrated until the ray
             *         intersects with the particle surface.
             * @param[in] particle Index of the particle.
             * @param[in,out] ray The ray which is propagating threw the
             *                    particle. The ray will be updated to the
             *                    closest point at the particle surface in
             *                    terms of the integration step size.
             * @param[out] inter_point Resulting intersection point with
             *                         the particle surface.
             * @param[out] inter_normal Resulting normal of the particle
             *                          surface at the intersection
             *                          point. The normal is pointing
             *                          inside the particle.
             * @param[in, out] output Buffer holding the resulting force and
             *                        torque change of each particle.
             */
            void rayPropagation(
                const size_t particle,
                Ray& ray,
                Vec3& inter_point,
                Vec3& inter_normal,
                OutputBuffer& output);
            /**
             * @brief Check if the position is outside of the particle.
             * @param[in] particle Index of the particle.
             * @param[in] r Position to check.
             * @retuns true if the position is outside the particle, false if
             *         the position is inside.
             */
            //virtual bool isOutsideParticle(const size_t particle, const Vec3& r) = 0;
            /**
             * @brief Integration step of the Runge-Kutta method.
             * @param[in] particle Pointer to the particle material containing
             *            the index of reflection distribution.
             * @param[in] x Current integration variable.
             * @param[in] h Integration step size.
             * @param[out] x_new Resulting integration variable.
             */
            void rk4(
                const ParticleMaterialLinearOneDirectional* particle,
                const Arg& x,
                const double h,
                Arg& x_new) const;
            /**
             * @brief The eikonal equation is a partial differential
             *        equation used for wave propagation.
             * @param[in] particle Pointer to the particle material containing
             *                     the index of reflection distribution.
             * @param[in] x Input variable of the equation.
             * @returns the time derivative of the input variable.
             */
            inline Arg eikonal(
                const ParticleMaterialLinearOneDirectional* particle,
                const Arg& x) const;
            /**
             * @brief Calculating the intersection of the ray and the particle.
             * @warning It is necessary that the ray origin is inside the
             *          particle.
             * @param[in] particle Index of the particle.
             * @param[in] ray Specifies the ray.
             * @param[out] inter_point Resulting intersection point with
             *                         the particle surface.
             * @param[out] inter_normal Resulting normal of the particle
             *                          surface at the intersection
             *                          point. The normal is pointing
             *                          inside the particle.
             */
            //virtual void intersection(
            //    const size_t particle,
            //    const Vec3& origin_in,
            //    const Vec3& origin_out,
            //    Vec3& inter_point,
            //    Vec3& inter_normal) = 0;
        private:
            const RK4 m_parameters;
        protected:
            Context& m_context;
        };

        /**
        * @brief Class implementing the inner particle propagation for
        *        linear index of refraction gradient in one direction.
        * @detail The light propagation is calculated by solving the Eikonal
        *         equation with the Runge�Kutta�Fehlberg(45) method.
        */
        class EikonalSolverRK45 : public IInnerParticlePropagationStage, 
            IInnerParticlePropagationParticle
        {
        protected:
            /**
             * @brief Constructing inner particle propagation stage and setting
             *        up the parameter for the Runge-Kutta-Fehlberg method.
             * @param context Pointer to context data for the ray tracing step.
             * @param parameters Structure containing all parameter for the
             *                   Runge-Kutta-Fehlberg method.
             */
            EikonalSolverRK45(
                Context& context,
                RK45 parameters);
        public:
            /**
             * @brief Inherited via ldplab::rtscpu::IInnerParticlePropagationStage.
             * @details Calculating the path of the rays threw the particle.
             * @param[in, out] rays RayBuffer holding the propagating rays.
             * @param[out] intersection IntersectionBuffer holding information
             *             about the intersection points.
             * @param[in, out] output Buffer holding the resulting force and torque
             *                        change of each particle.
             */
            void execute(
                RayBuffer& rays,
                IntersectionBuffer& intersection,
                OutputBuffer& output) override;
        protected:
            /**
             * @brief Structure keeping all variables of the differential
             *        equation.
             */
            struct Arg
            {
                /**
                 * @brief Vector pointing in the direction of light. Its norm
                 *        is the index of reflection at position r.
                 */
                Vec3 w;
                /**
                 * @brief Vector pointing to the light rays origin.
                 */
                Vec3 r;
                inline Arg operator*(const double& d) const
                {
                    return Arg{ w * d, r * d };
                }
                inline void operator*=(const double& d)
                {
                    w *= d;
                    r *= d;
                }
                inline void operator+=(const Arg& rhs)
                {
                    w += rhs.w;
                    r += rhs.r;
                }
                /**
                 * @brief Calculates the maximum value of all variables.
                 * @returns the maximum of r and w evaluated in all directions.
                 */
                double absoluteMax();
            };
        private:
            /**
             * @brief Calculating the ray propagation through the particle.
             * @detail The ray propagation is integrated until the ray
             *         intersects with the particle surface.
             * @param[in] particle Index of the particle.
             * @param[in,out] ray The ray which is propagating threw the
             *                    particle. The ray will be updated to the
             *                    closest point at the particle surface in
             *                    terms of the integration step size.
             * @param[out] inter_point Resulting intersection point with
             *                         the particle surface.
             * @param[out] inter_normal Resulting normal of the particle
             *                          surface at the intersection
             *                          point. The normal is pointing
             *                          inside the particle.
             * @param[in, out] output Buffer holding the resulting force and
             *                        torque change of each particle.
             */
            void rayPropagation(
                const size_t particle,
                Ray& ray,
                Vec3& inter_point,
                Vec3& inter_normal,
                OutputBuffer& output);
            /**
             * @brief Integration step of the Runge-Kutta-Fehlberg method.
             * @param[in] particle Pointer to the particle material containing
             *            the index of reflection distribution.
             * @param[in] x Current integration variable.
             * @param[in] h Integration step size.
             * @param[out] x_new Resulting integration variable.
             * @returns The error of the integration.
             */
            double rk45(
                const ParticleMaterialLinearOneDirectional* particle,
                const Arg& x,
                const double h,
                Arg& x_new) const;
            /**
             * @brief The eikonal equation is a partial differential
             *        equation used for wave propagation.
             * @param[in] particle Pointer to the particle material containing
             *                     the index of reflection distribution.
             * @param[in] x Input variable of the equation.
             * @returns the time derivative of the input variable.
             */
            inline Arg eikonal(
                const ParticleMaterialLinearOneDirectional* particle,
                const Arg& x) const;
        private:
            const double alpha[6]{ 0.0, 1.0 / 4.0, 3.0 / 8.0, 12.0 / 13.0, 1.0, 1.0 / 2.0 };
            const double beta[36]{
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ,
                1.0 / 4.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                3.0 / 32.0, 9.0 / 32.0, 0.0, 0.0, 0.0, 0.0,
                1932.0 / 2197.0, (-7200.0) / 2197.0, 7296.0 / 2197.0, 0.0, 0.0, 0.0,
                439.0 / 216.0, -8.0, 3680.0 / 513.0, (-845.0) / 4104.0, 0.0, 0.0,
                (-8.0) / 27.0, 2.0, (-3544.0) / 2565.0, 1859.0 / 4104.0, (-11.0) / 40.0, 0.0 };
            const double c[6]{ 25.0 / 216.0, 0.0, 1408.0 / 2565.0, 2197.0 / 4104.0, (-1.0) / 5.0, 0.0 };
            const double c_star[6]{ 16.0 / 135.0, 0.0, 6656.0 / 12825.0, 28561.0 / 56430.0, (-9.0) / 50.0, 2.0 / 55.0 };
            const double cerr[6]{ -0.00277778,  0.0 ,  0.02994152,  0.02919989, -0.02 , -0.03636364 };
        private:
            const RK45 m_parameters;
        protected:
            Context& m_context;
        };

        class IPPRodParticle : public IInnerParticlePropagationParticle
        {
        protected:
            IPPRodParticle(Context& context);
            /**
             * @brief Check if the position is outside of the particle.
             * @param[in] particle Index of the particle.
             * @param[in] r Position to check.
             * @retuns true if the position is outside the particle, false if
             *         the position is inside.
             */
            bool isOutsideParticle(const size_t particle, const Vec3& r) override;
            /**
             * @brief Calculating the intersection of the ray and the particle.
             * @warning It is necessary that the ray origin is inside the
             *          particle.
             * @param[in] particle Index of the particle.
             * @param[in] ray_origin_in Last ray position inside the particle.
             * @param[in] ray_origin_out First ray position outside the particle.
             * @param[out] inter_point Resulting intersection point with
             *                         the particle surface.
             * @param[out] inter_normal Resulting normal of the particle
             *                          surface at the intersection
             *                          point. The normal is pointing
             *                          inside the particle.
             */
            void intersection(
                const size_t particle,
                const Vec3& origin_in,
                const Vec3& origin_out,
                Vec3& inter_point,
                Vec3& inter_normal) override;
        private:
            /**
             *@brief Calculate the intersection point of a rayand the cylinder.
             * @param[in] particle Specifies the particle geometry.
             * @param[in] ray Specifies the ray.
             * @param[out] distance_min Resulting distance between the origin
             * of the rayand the intersection point.
             * @param[out] distance_max Resulting distance between the origin
             * of the rayand the intersection point.
             * @retuns true if the cylinderand the ray are intersecting, else
             *false will be returned.
             */
            bool cylinderIntersection(
                const RodParticle & particle,
                const Ray & ray,
                double& distance_min,
                double& distance_max);
            /**
             * @brief Calculates whether a ray intersects the particle at the
             *        cap or indentation under the assumption that the ray is
             *        within the infinite cylinder.
             * @param[in] particle Specifies the particle geometry.
             * @param[in] ray Specifies the ray.
             * @param[out] inter_point Resulting intersection point with
             *                         the particle surface.
             * @param[out] inter_normal Resulting normal of the particle
             *                          surface at the intersection
             *                          point. The normal is pointing
             *                          outside of the particle.
             * @returns true if the ray intersects the particle at the bottom
             *          or top.
             */
            bool bottomTopIntersection(
                const RodParticle & particle,
                const Ray & ray,
                Vec3 & inter_point,
                Vec3 & inter_normal);
            /**
             * @brief Calculate the intersection point of a ray and the sphere.
             * @param[in] origin Specifies the origin of the sphere.
             * @param[in] radius Specifies the radius of the sphere.
             * @param[in] ray Specifies the ray.
             * @param[out] distance_min Resulting distance between the origin
             *                      of the ray and the intersection point.
             * @param[out] distance_max Resulting distance between the origin
             *                      of the ray and the intersection point.
             * @retuns true if the sphere and the ray are intersecting, else
             *         false will be returned.
             */
            bool sphereIntersection(
                const Vec3 & origin,
                const double& raduis,
                const Ray & ray,
                double& distance_min,
                double& distance_max);

            /**
             * @brief Calculate the intersection point of a ray and the
             *        spherical cap of the rod particle.
             * @note The intersection point is always the point with the
             *       minimum distance from the ray origin.
             * @warning Only the correct hight of the intersection point is
             *          checked. It is assumed that the intersection point is
             *          inside the infinite cylinder.
             * @param[in] geometry Specifies the particle geometry.
             * @param[in] ray Specifies the ray.
             * @param[out] intersection_point Resulting intersection point with
             *                                the cap.
             * @param[out] intersection_normal Resulting normal of the particle
             *                                 surface at the intersection
             *                                 point. The normal is pointing
             *                                 outside of the particle.
             * @retuns true if the sphere and the ray are intersecting, else
             *         false will be returned.
             */
            bool capIntersection(
                const RodParticle & geometry,
                const Ray & ray,
                Vec3 & inter_point,
                Vec3 & inter_normal);
            /**
             * @brief Calculate the intersection point of a ray and the
             *        spherical indentation of the rod particle.
             * @note The intersection point is always the point with the
             *       maximum distance from the ray origin (including negative
             *       distances).
             * @warning No further checking is done if the intersection point
             *          is inside the cylinder.
             * @param[in] geometry Specifies the particle geometry.
             * @param[in] ray Specifies the ray.
             * @param[out] inter_point Resulting intersection point with
             *                                the indentation.
             * @param[out] inter_normal Resulting normal of the particle
             *                                 surface at the intersection
             *                                 point. The normal is pointing
             *                                 outside of the particle.
             * @retuns true if the sphere and the ray are intersecting, else
             *         false will be returned.
             */
            bool indentationIntersection(
                const RodParticle & geometry,
                const Ray & ray,
                Vec3 & inter_point,
                Vec3 & inter_normal);
        private:
            std::vector<RodParticle>& m_rod_particles;
        };

        class IPPSphereParticle : public IInnerParticlePropagationParticle
        {
        protected:
            IPPSphereParticle(Context& context);
            /**
             * @brief Check if the position is outside of the particle.
             * @param[in] particle Index of the particle.
             * @param[in] r Position to check.
             * @retuns true if the position is outside the particle, false if
             *         the position is inside.
             */
            bool isOutsideParticle(const size_t particle, const Vec3& r) override;
            /**
             * @brief Calculating the intersection of the ray and the particle.
             * @warning It is necessary that the ray origin is inside the
             *          particle.
             * @param[in] particle Index of the particle.
             * @param[in] ray_origin_in Last ray position inside the particle.
             * @param[in] ray_origin_out First ray position outside the particle.
             * @param[out] inter_point Resulting intersection point with
             *                         the particle surface.
             * @param[out] inter_normal Resulting normal of the particle
             *                          surface at the intersection
             *                          point. The normal is pointing
             *                          inside the particle.
             */
            void intersection(
                const size_t particle,
                const Vec3& origin_in,
                const Vec3& origin_out,
                Vec3& inter_point,
                Vec3& inter_normal) override;
        private:
            Context& m_context;
        };

        /**
         * @brief Class implementing the inner particle propagation for 
         *        linear index of refraction gradient in one direction.
         * @detail The light propagation is calculated by solving the Eikonal 
         *         equation with the Runge�Kutta�Fehlberg(45) method.
         */
        class RK45RodParticlePropagation
            : public EikonalSolverRK45, IPPRodParticle
        {
        public:
            /**
             * @brief Constructing inner particle propagation stage and setting 
             *        up the parameter for the Runge-Kutta-Fehlberg method.
             * @param context Pointer to context data for the ray tracing step.
             * @param parameters Structure containing all parameter for the 
             *                   Runge-Kutta-Fehlberg method.
             */
            RK45RodParticlePropagation(
                Context& context,
                RK45 parameters);
        private:
            /**
             * @brief Check if the position is outside of the particle.
             * @param[in] particle Index of the particle.
             * @param[in] r Position to check.
             * @retuns true if the position is outside the particle, false if 
             *         the position is inside.
             */
            bool isOutsideParticle(const size_t particle, const Vec3& r) override;
            /**
             * @brief Calculating the intersection of the ray and the particle.
             * @warning It is necessary that the ray origin is inside the 
             *          particle.
             * @param[in] particle Index of the particle.
             * @param[in] ray_origin_in Last ray position inside the particle.
             * @param[in] ray_origin_out First ray position outside the particle.
             * @param[out] inter_point Resulting intersection point with
             *                         the particle surface.
             * @param[out] inter_normal Resulting normal of the particle
             *                          surface at the intersection
             *                          point. The normal is pointing
             *                          inside the particle.
             */
            void intersection(
                const size_t particle,
                const Vec3& ray_in,
                const Vec3& ray_out,
                Vec3& inter_point,
                Vec3& inter_normal) override;
        };

        /**
        * @brief Class implementing the inner particle propagation for
        *        linear index of refraction gradient in one direction.
        * @detail The light propagation is calculated by solving the Eikonal
        *         equation with the Runge�Kutta method.
        */
        class RK4RodParticlePropagation
            : public EikonalSolverRK4, IPPRodParticle
        {
        public:
            /**
             * @brief Constructing inner particle propagation stage and setting
             *        up the parameter for the Runge-Kutta method.
             * @param context Pointer to context data for the ray tracing step.
             * @param parameters Structure containing all parameter for the
             *                   Runge-Kutta method.
             */
            RK4RodParticlePropagation(
                Context& context,
                RK4 parameters);
        private:
            /**
             * @brief Check if the position is outside of the particle.
             * @param[in] particle Index of the particle.
             * @param[in] r Position to check.
             * @retuns true if the position is outside the particle, false if
             *         the position is inside.
             */
            bool isOutsideParticle(const size_t particle, const Vec3& r) override;
            /**
             * @brief Calculating the intersection of the ray and the particle.
             * @warning It is necessary that the ray origin is inside the
             *          particle.
             * @param[in] particle Index of the particle.
             * @param[in] ray_origin_in Last ray position inside the particle.
             * @param[in] ray_origin_out First ray position outside the particle.
             * @param[out] inter_point Resulting intersection point with
             *                         the particle surface.
             * @param[out] inter_normal Resulting normal of the particle
             *                          surface at the intersection
             *                          point. The normal is pointing
             *                          inside the particle.
             */
            void intersection(
                const size_t particle,
                const Vec3& ray_in,
                const Vec3& ray_out,
                Vec3& inter_point,
                Vec3& inter_normal) override;
        };

        /**
         * @brief Class implementing the eikonal solver for spherical particle.
         * @detail The light propagation is calculated by solving the Eikonal
         *         equation with the Runge�Kutta�Fehlberg(45) method.
         */
        class RK45SphericalParticlePropagation :
            public EikonalSolverRK45, IPPSphereParticle
        {
        public:
            /**
             * @brief Constructing inner particle propagation stage and setting
             *        up the parameter for the Runge-Kutta-Fehlberg method.
             * @param context Pointer to context data for the ray tracing step.
             * @param parameters Structure containing all parameter for the
             *                   Runge-Kutta-Fehlberg method.
             */
            RK45SphericalParticlePropagation(
                Context& context,
                RK45 parameters);
        private:
            /**
             * @brief Check if the position is outside of the particle.
             * @param[in] particle Index of the particle.
             * @param[in] r Position to check.
             * @retuns true if the position is outside the particle, false if
             *         the position is inside.
             */
            bool isOutsideParticle(
                const size_t particle,
                const Vec3& r) override;
            /**
             * @brief Calculating the intersection of the ray and the particle.
             * @warning It is necessary that the ray origin is inside the
             *          particle.
             * @param[in] particle Index of the particle.
             * @param[in] ray_origin_in Last ray position inside the particle.
             * @param[in] ray_origin_out First ray position outside the particle.
             * @param[out] inter_point Resulting intersection point with
             *                         the particle surface.
             * @param[out] inter_normal Resulting normal of the particle
             *                          surface at the intersection
             *                          point. The normal is pointing
             *                          inside the particle.
             */
            void intersection(
                const size_t particle,
                const Vec3& ray_in,
                const Vec3& ray_out,
                Vec3& inter_point,
                Vec3& inter_normal) override;
        };
       
        /**
         * @brief Class implementing the eikonal solver for spherical particle.
         * @detail The light propagation is calculated by solving the Eikonal
         *         equation with the Runge�Kutta method.
         */
        class RK4SphericalParticlePropagation :
            public EikonalSolverRK4, IPPSphereParticle
        {
        public:
            /**
             * @brief Constructing inner particle propagation stage and setting
             *        up the parameter for the Runge-Kutta method.
             * @param context Pointer to context data for the ray tracing step.
             * @param parameters Structure containing all parameter for the
             *                   Runge-Kutta method.
             */
            RK4SphericalParticlePropagation(
                Context& context,
                RK4 parameters);
        private:
            /**
             * @brief Check if the position is outside of the particle.
             * @param[in] particle Index of the particle.
             * @param[in] r Position to check.
             * @retuns true if the position is outside the particle, false if
             *         the position is inside.
             */
            bool isOutsideParticle(
                const size_t particle,
                const Vec3& r);
            /**
             * @brief Calculating the intersection of the ray and the particle.
             * @warning It is necessary that the ray origin is inside the
             *          particle.
             * @param[in] particle Index of the particle.
             * @param[in] ray_origin_in Last ray position inside the particle.
             * @param[in] ray_origin_out First ray position outside the particle.
             * @param[out] inter_point Resulting intersection point with
             *                         the particle surface.
             * @param[out] inter_normal Resulting normal of the particle
             *                          surface at the intersection
             *                          point. The normal is pointing
             *                          inside the particle.
             */
            void intersection(
                const size_t particle,
                const Vec3& ray_in,
                const Vec3& ray_out,
                Vec3& inter_point,
                Vec3& inter_normal);
        };
    }
}

#endif