#ifndef WWU_LDPLAB_EIKONAL_SOLVER_HPP
#define WWU_LDPLAB_EIKONAL_SOLVER_HPP

namespace ldplab
{
    struct IEikonalSolverParameter
    {
        enum class Type { rk45, rk4 };
        /**
         * @brief The destructor is virtual since classes inherit from
         *        IEikonalSolver.
         */
        virtual ~IEikonalSolverParameter() { }
        /** @brief Returns the type of the instance. */
        virtual Type type() const = 0;
        /** @brief Returns the type of the instance as string. */
        const char* typeString() const
        {
            switch (type())
            {
            case Type::rk45: return "rk45";
            case Type::rk4: return "rk4";
            default: return "unknown_type";
            }
        }
    };

    struct RK45Parameter : public IEikonalSolverParameter
    {
        RK45Parameter(real_t initial_step_size, real_t epsilon, real_t safety_factor)
            :
            initial_step_size{ initial_step_size },
            epsilon{ epsilon },
            safety_factor{ safety_factor }
        {}
        /** @brief Initial step size for each integration method. */
        real_t initial_step_size;
        /** @brief epsilon Maximum error tolerance of the integration steps. */
        real_t epsilon;
        /** @brief Factor for new step size calculation in RK45 Method. */
        real_t safety_factor;

        Type type() const override { return IEikonalSolverParameter::Type::rk45; }
    };

    struct RK4Parameter : public IEikonalSolverParameter
    {
        RK4Parameter(real_t step_size)
            :
            step_size{ step_size }
        {}
        /** @brief Initial step size for each integration method. */
        real_t step_size;

        Type type() const override { return IEikonalSolverParameter::Type::rk4; }
    };
}

#endif