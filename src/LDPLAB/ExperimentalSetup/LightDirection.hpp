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
	};

    struct LightDirectionHomogenous : public ILightDirection
    {
        Type type() const override { return Type::homogenous; }
        /**
         * @brief The direction in which rays are emitted by the light source.
         */
        Vec3 direction;
    };
}

#endif
