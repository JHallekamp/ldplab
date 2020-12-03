#ifndef WWU_LDPLAB_ILIGHT_DIRECTION_HPP
#define WWU_LDPLAB_ILIGHT_DIRECTION_HPP

namespace ldplab
{
    class ILightDirection
    {
	public:
		enum class Type { homogenous };
	public:
		/**
		 * @brief The destructor is virtual since classes inherit from
		 *        ILightDirection.
		 */
		virtual ~ILightDirection() { }
		/** @brief Returns the type of the instance. */
		virtual Type type() const = 0;
    }
}

#endif
