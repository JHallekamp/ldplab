#ifndef WWU_LDPLAP_RTSCPU_INNER_PARTICLE_PROPAGATION_STAGE_HPP
#define WWU_LDPLAP_RTSCPU_INNER_PARTICLE_PROPAGATION_STAGE_HPP

#include <cmath>
#include <memory>
#include "../../Geometry.hpp"


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
        struct RodeParticle;

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
             */
            virtual void execute(
                RayBuffer& rays,
                IntersectionBuffer& intersection) = 0;
        };

        /**
         * @brief Class implementing the inner particle propagation for 
         *        linear index of refraction gradient in one direction.
         * @detail The Light propagation is calculated by solving the Eikonal 
         *         equation with the Runge–Kutta–Fehlberg method.
         */
        class LinearIndexGradientRodeParticlePropagation
            : public IInnerParticlePropagationStage
        {
        public:
            LinearIndexGradientRodeParticlePropagation(
                std::shared_ptr<Context> context,
                const double initial_step_size,
                const double epsilon,
                const double safety_factor);
            /**
             * @brief Inherited via ldplab::rtscpu::IInnerParticlePropagationStage.  
             * @details Calculating the path of the rays threw the particle.
             * @param[in, out] rays RayBuffer holding the propagating rays.
             */
            void execute(
                RayBuffer& rays,
                IntersectionBuffer& intersection) override;
        private:
            struct Arg
            {
                Vec3 w;
                Vec3 r;
                Arg operator*(const double& d) 
                {
                    Arg arg { this->w * d, this->r * d };
                    return arg; 
                }
                Arg operator+(const Arg& a)
                {
                    Arg arg{ this->w + a.w, this->r + a.r };
                    return arg;
                }
                double absoluteMax()
                {
                    double max = std::abs(w.x);
                    return max;
                }
            };
            void rayPropagation(
                const size_t particle, 
                Ray& ray, 
                Vec3& inter_point,
                Vec3& inter_normal);
            bool isOutsideParticle(const RodeParticle& geometry, const Vec3& r);
            double rk45(
                const ParticleMaterialLinearOneDirectional* particle, 
                const Arg& x,
                const double h,
                Arg& x_new);
            Arg eikonal(
                const ParticleMaterialLinearOneDirectional* particle, 
                Arg& x);

            void intersection(
                const RodeParticle& geometry,
                Arg& ray,
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
                const RodeParticle& geometry,
                const Ray& ray,
                Vec3& inter_point,
                Vec3& inter_normal);
            /**
             * @brief Calculate the intersection point of a ray and the
             *        spherical cap of the rode particle.
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
                const RodeParticle& geometry,
                const Ray& ray,
                Vec3& inter_point,
                Vec3& inter_normal);
            /**
             * @brief Calculate the intersection point of a ray and the
             *        spherical indentation of the rode particle.
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
                const RodeParticle& geometry,
                const Ray& ray,
                Vec3& inter_point,
                Vec3& inter_normal);

        private:
            const double initial_step_size;
            const double epsilon;
            const double safety_factor;
            const double alpha[6] { 0.0, 1.0/4.0, 3.0/8.0, 12.0/13.0, 1.0, 1.0/2.0 };
            const double beta[36]{
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ,
                1.0 / 4.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                3.0 / 32.0, 9.0 / 32.0, 0.0, 0.0, 0.0, 0.0,
                1932.0 / 2197.0, (-7200.0) / 2197.0, 7296.0 / 2197.0, 0.0, 0.0, 0.0,
                439.0 / 216.0, -8.0, 3680.0 / 513.0, (-845.0) / 4104.0, 0.0, 0.0,
                (-8.0) / 27.0, 2.0, (-3544.0) / 2565.0, 1859.0 / 4104.0, (-11.0) / 40.0, 0.0 };
            const double c[6] { 25.0 / 216.0, 0.0, 1408.0 / 2565.0, 2197.0 / 4104.0, (-1.0) / 5.0, 0.0 };
            const double c_star[6] { 16.0 / 135.0, 0.0, 6656.0 / 12825.0, 28561.0 / 56430.0, (-9.0) / 50.0, 2.0 / 55.0 };
            const double cerr[6] { -0.00277778,  0.0 ,  0.02994152,  0.02919989, -0.02 , -0.03636364 };
        private:
            std::shared_ptr<Context> m_context;
        };
    }
}

#endif