#ifndef WWU_LDPLAB_ACCELERATOR_STUCTURE_HPP
#define WWU_LDPLAB_ACCELERATOR_STUCTURE_HPP

#include <string>

namespace ldplab
{
    /**
     * @brief Paramter for RayTracingStep accelerator structures.
     */
    struct IAcceleratorStructureParameter
    {
        enum class Type { brute_force, octree };
        /**
         * @brief The destructor is virtual since classes inherit from
         *        IAcceleratorStructureParameter.
         */
        virtual ~IAcceleratorStructureParameter() { }
        /** @brief Returns the type of the instance. */
        virtual Type type() const = 0;
        /** @brief Returns the type of the instance as string. */
        const std::string typeString() const
        {
            switch (type())
            {
            case Type::brute_force: return "brute_force";
            case Type::octree: return "octree";
            default: return "unknown_type";
            }
        }
    };

    /** @brief Brute force approaches do not take any special parameter. */
    struct AcceleratorStructureBruteForceParameter : 
        public IAcceleratorStructureParameter
    {
        /** @brief Returns Type::brute_force */
        Type type() const override { return Type::brute_force; }
    };

    /** @brief Store mesh in an octree to accerlerate intersection tests. */
    struct AcceleratorStructureOctreeParameter :
        public IAcceleratorStructureParameter
    {
        AcceleratorStructureOctreeParameter(size_t depth)
            :
            octree_depth { depth }
        { }
        /** @brief Returns Type::octree */
        Type type() const override { return Type::octree; }
        /** @brief The depth used for the mesh octrees. */
        size_t octree_depth;
    };
}
#endif