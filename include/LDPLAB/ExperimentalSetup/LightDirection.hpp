#ifndef WWU_LDPLAB_ILIGHT_DIRECTION_HPP
#define WWU_LDPLAB_ILIGHT_DIRECTION_HPP

#include "../Geometry.hpp"
#include <string>

namespace ldplab
{
	struct ILightDirection
	{
		enum class Type { homogeneous };
		/**
		 * @brief The destructor is virtual since classes inherit from
		 *        ILightDirection.
		 */
		virtual ~ILightDirection() { }
		/** @brief Returns the type of the instance. */
		virtual Type type() const = 0;
        /** @brief Returns the type of the instance as string. */
        const std::string typeString() const
        {
            switch (type())
            {
            case Type::homogeneous: return "homogeneous";
            default: return "unknown_type";
            }
        }
	};

    /** @brief Light is emitted in the direction of the plane. */
    struct LightDirectionHomogeneous : public ILightDirection
    {
        Type type() const override { return Type::homogeneous; }
    };
}

#endif
