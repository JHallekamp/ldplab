#ifndef WWU_LDPLAB_EIKONAL_SOLVER_HPP
#define WWU_LDPLAB_EIKONAL_SOLVER_HPP

namespace ldplab
{
    struct IEikonalSolver
    {
        enum class Type { rk45 };
        /**
         * @brief The destructor is virtual since classes inherit from
         *        IEikonalSolver.
         */
        virtual ~IEikonalSolver() { }
        /** @brief Returns the type of the instance. */
        virtual Type type() const = 0;
        /** @brief Returns the type of the instance as string. */
        const char* typeString() const
        {
            switch (type())
            {
            case Type::rk45: return "rk45";
            default: return "unknown_type";
            }
        }
    };

    struct RK45 : public IEikonalSolver
    {
        RK45(double initial_step_size, double epsilon, double safety_factor)
            :
            initial_step_size{ initial_step_size },
            epsilon{ epsilon },
            safety_factor{ safety_factor }
        {}
        /** @brief Initial step size for each integration method. */
        double initial_step_size;
        /** @brief epsilon Maximum error tolerance of the integration steps. */
        double epsilon;
        /** @brief Factor for new step size calculation in RK45 Method. */
        double safety_factor;

        Type type() const override { return IEikonalSolver::Type::rk45; }
    };
}

#endif