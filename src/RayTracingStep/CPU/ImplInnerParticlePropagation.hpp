#ifndef WWU_LDPLAB_RTSCPU_IMPL_INNER_PARTICLE_PROPAGATION_HPP
#define WWU_LDPLAB_RTSCPU_IMPL_INNER_PARTICLE_PROPAGATION_HPP

#include <LDPLAB/RayTracingStep/EikonalSolverParameter.hpp>
#include <LDPLAB/RayTracingStep/CPU/IInnerParticlePropagation.hpp>

namespace ldplab
{
    namespace rtscpu
    {
        /**
       * @brief Class implementing the inner particle propagation for
       *        linear index of refraction gradient in one direction.
       * @detail The light propagation is calculated by solving the Eikonal
       *         equation with the Runge�Kutta method.
       */
        class EikonalSolverRK4LinearIndexGradient :
            public IInnerParticlePropagation
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
                RK4Parameter parameters);
            virtual void stepSetup(
                const ExperimentalSetup& setup, 
                const SimulationState& simulation_state, 
                const InterfaceMapping& interface_mapping) override { }
            virtual void execute(
                RayBuffer& ray_data, 
                IntersectionBuffer& intersection_data, 
                OutputBuffer& output_data, 
                const std::vector<std::shared_ptr<IGenericGeometry>>& geometry_data, 
                const std::vector<std::shared_ptr<IParticleMaterial>>& material_data,
                const std::vector<Vec3>& center_of_mass,
                const SimulationParameter& simulation_parameter, 
                void* stage_dependent_data) override;
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
                size_t particle_index,
                Ray& ray,
                Vec3& inter_point,
                Vec3& inter_normal,
                const std::shared_ptr<IGenericGeometry>& particle_geometry,
                const std::shared_ptr<IParticleMaterial>& particle_material,
                const Vec3& particle_center_of_mass,
                OutputBuffer& output) const;
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
                const std::shared_ptr<IParticleMaterial>& particle_material,
                const Arg& x,
                const double h,
                Arg& x_new) const;
        private:
            const RK4Parameter m_parameters;
        };
    }
}

#endif