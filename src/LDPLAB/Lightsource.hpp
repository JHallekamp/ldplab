#ifndef WWU_LDPLAB_LIGHTSOUCRE_HPP
#define WWU_LDPLAB_LIGHTSOUCRE_HPP

#include "ILightDirection.hpp"
#include "ILightDistribution.hpp"
#include "ILightPolarization.hpp"
#include "Geometry.hpp"

namespace ldplab
{
    /**
     * @brief Class modeling a light source. 
     */
    class LightSource
    {
        /**
         * @brief Direction of the individual rays. 
         */
        ILightDirection direction;
        /**
         * @brief Distribution of the light intensity.
         */
        ILightDistribution intensity_distribution;
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
        ILightPolarisation polarisation;
        
    };
}

#endif