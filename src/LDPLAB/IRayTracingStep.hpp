#ifndef WWU_LDPLAB_IRAY_TRACING_STEP_HPP
#define WWU_LDPLAB_IRAY_TRACING_STEP_HPP

namespace ldplab
{
    /**
     * @brief Generic interface class which provides the simulation a 
     *        calculation of the light induced force on the particles via 
     *        dynamic ray tracing. 
     */
    class IRayTracingStep
    {
    public:
        /**
         * @brief The destructor is virtual since class inhered from 
         *        IRayTracingStep
         */
        virtual ~IRayTracingStep() { };
        /**
         * @brief Begin ray tracing simulation.
         */
        virtual void start() = 0;
    };
}

#endif 
