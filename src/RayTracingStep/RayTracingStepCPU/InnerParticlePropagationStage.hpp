#ifndef WWU_LDPLAP_RTSCPU_INNER_PARTICLE_PROPAGATION_STAGE_HPP
#define WWU_LDPLAP_RTSCPU_INNER_PARTICLE_PROPAGATION_STAGE_HPP

#include <memory>
#include <vector>

#include "Data.hpp"
#include <LDPLAB/RayTracingStep/EikonalSolverParameter.hpp>
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

        /**
        * @brief Class implementing the inner particle propagation for
        *        linear index of refraction gradient in one direction.
        * @detail The light propagation is calculated by solving the Eikonal
        *         equation with the Runge�Kutta method.
        */
        class EikonalSolverRK4LinearIndexGradient : 
            public IInnerParticlePropagationStage
        {
        public:
            /**
             * @brief Constructing inner particle propagation stage and setting
             *        up the parameter for the Runge-Kutta method.
             * @param context Pointer to context data for the ray tracing step.
             * @param parameters Structure containing all parameter for the
             *                   Runge-Kutta method.
             */
            EikonalSolverRK4LinearIndexGradient(
                Context& context,
                RK4Parameter parameters);
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
                inline Arg operator*(const real_t& d) const
                {
                    return Arg{ w * d, r * d };
                }
                inline void operator*=(const real_t& d)
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
                const real_t h,
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
            const RK4Parameter m_parameters;
        protected:
            Context& m_context;
        };

        /**
        * @brief Class implementing the inner particle propagation for
        *        linear index of refraction gradient in one direction.
        * @detail The light propagation is calculated by solving the Eikonal
        *         equation with the Runge�Kutta�Fehlberg(45) method.
        */
        class EikonalSolverRK45LinearIndexGradient : 
            public IInnerParticlePropagationStage
        {
        public:
            /**
             * @brief Constructing inner particle propagation stage and setting
             *        up the parameter for the Runge-Kutta-Fehlberg method.
             * @param context Pointer to context data for the ray tracing step.
             * @param parameters Structure containing all parameter for the
             *                   Runge-Kutta-Fehlberg method.
             */
            EikonalSolverRK45LinearIndexGradient(
                Context& context,
                RK45Parameter parameters);
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
                inline Arg operator*(const real_t& d) const
                {
                    return Arg{ w * d, r * d };
                }
                inline void operator*=(const real_t& d)
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
                real_t absoluteMax();
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
            real_t rk45(
                const ParticleMaterialLinearOneDirectional* particle,
                const Arg& x,
                const real_t h,
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
            const real_t alpha[6]{ 0.0, 1.0 / 4.0, 3.0 / 8.0, 12.0 / 13.0, 1.0, 1.0 / 2.0 };
            const real_t beta[36]{
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ,
                1.0 / 4.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                3.0 / 32.0, 9.0 / 32.0, 0.0, 0.0, 0.0, 0.0,
                1932.0 / 2197.0, (-7200.0) / 2197.0, 7296.0 / 2197.0, 0.0, 0.0, 0.0,
                439.0 / 216.0, -8.0, 3680.0 / 513.0, (-845.0) / 4104.0, 0.0, 0.0,
                (-8.0) / 27.0, 2.0, (-3544.0) / 2565.0, 1859.0 / 4104.0, (-11.0) / 40.0, 0.0 };
            const real_t c[6]{ 25.0 / 216.0, 0.0, 1408.0 / 2565.0, 2197.0 / 4104.0, (-1.0) / 5.0, 0.0 };
            const real_t c_star[6]{ 16.0 / 135.0, 0.0, 6656.0 / 12825.0, 28561.0 / 56430.0, (-9.0) / 50.0, 2.0 / 55.0 };
            const real_t cerr[6]{ -0.00277778,  0.0 ,  0.02994152,  0.02919989, -0.02 , -0.03636364 };
        private:
            const RK45Parameter m_parameters;
        protected:
            Context& m_context;
        };
    }
}

#endif