#ifndef WWU_LDPLAB_EXPERIMENTAL_SETUP_HPP
#define WWU_LDPLAB_EXPERIMENTAL_SETUP_HPP

#include "Geometry.hpp"
#include "Particle.hpp"
#include "Lightsource.hpp"
#include <vector>

namespace ldplab
{
    /**
     * @brief Structure collects all information about the experimental setup. 
     */
    struct ExperimentalSetup
    {
        /**
         * @brief This bounding box is spanning the space in which the 
         *        simulation is contained;
         */
        BoundingBox world;
        /**
         * @brief All particles 
         */
        std::vector<Particle>  particles;
        /**
         * @brief Light sources
         */
        std::vector<LightSource> light_sources;
        /**
         * @brief Index of reflection of the medium.
         */
        double refractive_index;
    };
}

#endif
