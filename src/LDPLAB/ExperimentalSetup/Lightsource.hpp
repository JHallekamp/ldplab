#ifndef WWU_LDPLAB_LIGHTSOUCRE_HPP
#define WWU_LDPLAB_LIGHTSOUCRE_HPP

#include "LightDirection.hpp"
#include "LightDistribution.hpp"
#include "LightPolarization.hpp"
#include "../Geometry.hpp"
#include "../Utils/UID.hpp"

#include <memory>

namespace ldplab
{
    /**
     * @brief Class modeling a light source. 
     */
    struct LightSource
    {
        /**
         * @brief Unique light source id.
         */
        UID<LightSource> uid;
        /**
         * @brief Direction of the individual rays. 
         */
        std::shared_ptr<ILightDirection> direction;
        /**
         * @brief Distribution of the light intensity.
         */
        std::shared_ptr<ILightDistribution> intensity_distribution;
        /**
         * @brief Orientation if the light source plane.
         * @note Has to be a normalized vector
         */
        Vec3 orientation;
        /**
         * @brief The corner of the light source rectangle that is used as the
         *        origin of the light planes coordinate system.
         */
        Vec3 origin_corner;
        /**
         * @brief The direction of the x-axis of the light plane given in world
         *        coordinates.
         * @note Has to be orthogonal to orientation
         * @note Has to be orthogonal to vertical_direction
         * @note Has to be a normalized vector
         */
        Vec3 horizontal_direction;
        /**
         * @brief The direction of the y-axis of the light plane given in world
         *        coordinates.
         * @note Has to be orthogonal to orientation
         * @note Has to be orthogonal to horizontal_direction
         * @note Has to be a normalized vector
         */
        Vec3 vertical_direction;
        /**
         * @brief Extent of the light source rectangle in the direction of the
         *        x-axis in world units.
         * @note The x-axis is defined by horizontal_direction
         */
        double horizontal_size;
        /**
         * @brief Extent of the light source rectangle in the direction of the
         *        y-axis in world units.
         * @note The y-axis is defined by vertical_direction
         */
        double vertical_size;
        /**
         * @brief Polarization state of the Light source.
         */
        std::shared_ptr<ILightPolarisation> polarisation;
        
    };
}

#endif