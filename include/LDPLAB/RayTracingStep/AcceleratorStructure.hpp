#ifndef WWU_LDPLAB_ACCELERATOR_STUCTURE_HPP
#define WWU_LDPLAB_ACCELERATOR_STUCTURE_HPP

namespace ldplab
{
    /**
     * @brief Interface for accelerator structure for intersection test in the 
     *        RayTracingStep.
     */
    struct IAcceleratorStructure
    {
        enum class Type { brut_force };
        /**
         * @brief The destructor is virtual since classes inherit from
         *        IEikonalSolver.
         */
        virtual ~IAcceleratorStructure() { }
        /** @brief Returns the type of the instance. */
        virtual Type type() const = 0;
        /** @brief Returns the type of the instance as string. */
        const char* typeString() const
        {
            switch (type())
            {
            case Type::brut_force: return "brut_force";
            default: return "unknown_type";
            }
        }
    };
    
}
#endif