#ifndef WWU_LDPLAB_ILIGHT_DIRECTION_HPP
#define WWU_LDPLAB_ILIGHT_DIRECTION_HPP

namespace ldplab
{
    class ILightDirection
    {
	private:
		enum class Type { homogenous } type;
	public:
		/**
		 * @brief The destructor is virtual since classes inherit from
		 *        ILightDirection.
		 */
		virtual ~ILightDirection() { }
		Type getType() { return type; }
    }
}

#endif
