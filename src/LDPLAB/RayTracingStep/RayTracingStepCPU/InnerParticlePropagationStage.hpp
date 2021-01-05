#ifndef WWU_LDPLAP_RTSCPU_INNER_PARTICLE_PROPAGATION_STAGE_HPP
#define WWU_LDPLAP_RTSCPU_INNER_PARTICLE_PROPAGATION_STAGE_HPP

#include <memory>
#include <vector>

#include "Data.hpp"
#include "../../Geometry.hpp"
#include "../../ExperimentalSetup/EikonalSolver.hpp"

namespace ldplab
{
    // Prototype
    struct Particle;
    struct ParticleMaterialLinearOneDirectional;

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
         *         equation with the Runge–Kutta–Fehlberg(45) method.
         */
        class LinearIndexGradientRodParticlePropagation
            : public IInnerParticlePropagationStage
        {
        public:
            /**
             * @brief Constructing inner particle propagation stage and setting 
             *        up the parameter for the Runge-Kutta-Fehlberg method.
             * @param context Pointer to context data for the ray tracing step.
             * @param parameters Structure containing all parameter for the 
             *                   Runge-Kutta-Fehlberg method.
             */
            LinearIndexGradientRodParticlePropagation(
                std::shared_ptr<Context> context,
                RK45 parameters);
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
        private:
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
             * @param[in] geometry Specifies the particle geometry.
             * @param[in] r Position to check.
             * @retuns true if the position is outside the particle, false if 
             *         the position is inside.
             */
            bool isOutsideParticle(const RodParticle& geometry, const Vec3& r);
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
            /**
             * @brief Calculating the intersection of the ray and the particle.
             * @warning It is necessary that the ray origin is inside the 
             *          particle.
             * @param[in] geometry Specifies the particle geometry.
             * @param[in] ray Specifies the ray.
             * @param[out] inter_point Resulting intersection point with
             *                         the particle surface.
             * @param[out] inter_normal Resulting normal of the particle
             *                          surface at the intersection
             *                          point. The normal is pointing
             *                          inside the particle.
             */
            void intersection(
                const RodParticle& geometry,
                const Arg& ray,
                Vec3& inter_point,
                Vec3& inter_normal);
            /**
             * @brief Calculate the intersection point of a ray and the cylinder.
             * @warning It is assumed that the rays origin is inside the 
             *          particle.
             * @param[in] geometry Specifies the particle geometry.
             * @param[in] ray Specifies the ray.
             * @param[out] inter_point Resulting intersection point with
             *                         the cap.
             * @param[out] inter_normal Resulting normal of the particle
             *                          surface at the intersection
             *                          point. The normal is pointing
             *                          inside the particle.
             * @retuns true if the cylinder and the ray are intersecting, else
             *         false will be returned.
             */
            bool cylinderIntersection(
                const RodParticle& geometry,
                const Ray& ray,
                Vec3& inter_point,
                Vec3& inter_normal);
            /**
             * @brief Calculate the intersection point of a ray and the
             *        spherical cap of the rod particle.
             * @warning It is assumed that the rays origin is inside the 
             *          particle.
             * @param[in] geometry Specifies the particle geometry.
             * @param[in] ray Specifies the ray.
             * @param[out] inter_point Resulting intersection point with
             *                         the cap.
             * @param[out] inter_normal Resulting normal of the particle
             *                          surface at the intersection
             *                          point. The normal is pointing
             *                          inside the particle.
             * @retuns true if the cap and the ray are intersecting, else
             *         false will be returned.
             */
            bool capIntersection(
                const RodParticle& geometry,
                const Ray& ray,
                Vec3& inter_point,
                Vec3& inter_normal);
            /**
             * @brief Calculate the intersection point of a ray and the
             *        spherical indentation of the rod particle.
             * @warning It is assumed that the rays origin is inside the 
             *          particle.
             * @param[in] geometry Specifies the particle geometry.
             * @param[in] ray Specifies the ray.
             * @param[out] intersection_point Resulting intersection point with
             *                                the indentation.
             * @param[out] intersection_normal Resulting normal of the particle
             *                                 surface at the intersection
             *                                 point. The normal is pointing
             *                                 inside the particle.
             * @retuns true if the indentation and the ray are intersecting, 
             *         else false will be returned.
             */
            bool indentationIntersection(
                const RodParticle& geometry,
                const Ray& ray,
                Vec3& inter_point,
                Vec3& inter_normal);
        private:
            const double alpha[6]{ 0.0, 1.0/4.0, 3.0/8.0, 12.0/13.0, 1.0, 1.0/2.0 };
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
            std::shared_ptr<Context> m_context;
            std::vector<RodParticle>& m_rod_particles;
        };
    }
}

#endif