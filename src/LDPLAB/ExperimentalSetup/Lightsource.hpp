#ifndef WWU_LDPLAB_LIGHTSOUCRE_HPP
#define WWU_LDPLAB_LIGHTSOUCRE_HPP

#include "LightDirection.hpp"
#include "LightDistribution.hpp"
#include "LightPolarization.hpp"
#include "../Geometry.hpp"

#include <memory>

namespace ldplab
{
    /**
     * @brief Class modeling a light source. 
     */
    struct LightSource
    {
        /**
         * @brief Direction of the individual rays. 
         */
        std::shared_ptr<ILightDirection> direction;
        /**
         * @brief Distribution of the light intensity.
         */
        std::shared_ptr<ILightDistribution> intensity_distribution;
        /**
         * @brief Position of the light source in the world coordinate system.
         */
        Vec3 position;
        /**
         * @brief Orientation if the light source plane. 
         */
        Vec3 orientation;
        /**
         * @brief Boundary of the light source in the plane.
         */
        BoundingBox plane_boundry;
        /**
         * @brief Polarization state of the Light source.
         */
        std::shared_ptr<ILightPolarisation> polarisation;
        
    };
}

#endif