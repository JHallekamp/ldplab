#ifndef WWU_LDPLAB_EXPERIMENTAL_SETUP_HPP
#define WWU_LDPLAB_EXPERIMENTAL_SETUP_HPP

#include "Lightsource.hpp"
#include "Particle.hpp"
#include "../Geometry.hpp"

#include <vector>

namespace ldplab
{
    /**
     * @brief Structure collects all information about the experimental setup. 
     */
    struct ExperimentalSetup
    {
        /** @brief All particles */
        std::vector<Particle>  particles;
        /** @brief Light sources */
        std::vector<LightSource> light_sources;
        /** @brief Index of refraction of the medium. */
        double medium_refraction_index;
        /** @brief Unique ID of the experimental setup. */
        UID<ExperimentalSetup> uid;
    };
}

#endif
