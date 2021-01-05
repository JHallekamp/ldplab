#ifndef WWU_LDPLAB_ILIGHT_DIRECTION_HPP
#define WWU_LDPLAB_ILIGHT_DIRECTION_HPP

#include "../Geometry.hpp"

namespace ldplab
{
	struct ILightDirection
	{
		enum class Type { homogenous };
		/**
		 * @brief The destructor is virtual since classes inherit from
		 *        ILightDirection.
		 */
		virtual ~ILightDirection() { }
		/** @brief Returns the type of the instance. */
		virtual Type type() const = 0;
        /** @brief Returns the type of the instance as string. */
        const char* typeString() const
        {
            switch (type())
            {
            case Type::homogenous: return "homogenous";
            default: return "unknown_type";
            }
        }
	};

    /** @brief Light is emited in the direction of the plane. */
    struct LightDirectionHomogenous : public ILightDirection
    {
        Type type() const override { return Type::homogenous; }
    };
}

#endif
