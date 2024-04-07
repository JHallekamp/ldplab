<div>
<p>
A software framework for massively parallel simulations of arbitrary light-driven particle configurations.
</p>

[Overview](#overview) -
[Contribution](#contribution)- 
[Features](#features) -
[Documentation](#documentation) -
[Installation](#installation) -
[Usage](#usage) -
[License](#license)


</div>

## Overview

LDPLAB is a software framework designed to calculate the force and torque that light induces on micro particles with an internal index of refraction gradient. The framework provides abstracted interfaces to support optimized solutions for individual particle properties and representations. The user has full control over the simulation-specific details and can replace each simulation step with their own implementations.

LDPLAB is written in C++14 and provided as a platform-independent library.
It has the following dependencies:

- [**GLM**](https://github.com/g-truc/glm)
- [**TinyObjLoader**](https://github.com/tinyobjloader/tinyobjloader)

The project is not actively maintained.

## Contribution

LDPLAB was as part of two master theses developed within the [SFB1459](https://www.uni-muenster.de/SFB1459/research/researchareab.html) at the University of Münster.

The core framework is the work of Alexander Gerwing and Jonas Hallekamp.
The CUDA implementation has been provided by Alexander Gerwing as part of his master thesis.

## Features

- C++14 interfaces
- Parallelization with std::threads 
- GPU Acceleration with CUDA (Nvidia)
- GPU Acceleration with OpenGL (**Deprecated**)
- Support for ray tracing-based calculation of force and torque
- Support for spherical particles with linear gradient refractive index 
- Support for polarized light (CPU only)
- Support for homogeneous, collimated, polarized, and non-polarized light sources.

## Documentation

### Interface

LDPLAB consists of modules called simulation steps, each of which models a physical process at a given time step. The user holds complete control over the execution of the simulation’s steps. Each stage defines an interface describing its in- and output data. To reduce the complexity and prevent an unnecessarily verbose framework, all step interfaces can access common data stored by the simulation state and the experimental setup. The simulation state data structure contains mutable data with regard to multiple executions of simulation steps. The experimental setup models the unchanged physical environment for the entire simulation.

### Model

To simulate light-driven particles, we used an abstracted design derived from experimental setups described by the research goals of the [SFB1459]. The model interface structures support a variety of physical models and increase reusability through a modular design of the implemented components.

- ExperimentalSetup
    - Particles
    - LightSources
- SimulationState
- Particle
    - IParticleGeometry
    - IParticleMaterial
    - IBoundingVolume
- LightSource
    - ILightDirection
    - ILightDistribution
    - ILightPolarisation

### Ray-Tracing Step

The simulation calculates force and torque through ray-tracing. The resulting momentum transfer is summed up with each change of light direction. We abstracted the process in the Ray-Tracing Pipeline, consisting out of five stages. Each performing the individual ray propagation and interaction with the particle based on the underlying Model.  

To increase performance for arbitrarily shaped particles (e.g., mesh-based particles), each particle is encapsulated in a bounding volume. *However, it is important to note that overlapping bounding volumes aren't accounted for in the current implementation.*

Due to the internal reflection, the RayTracingStep would continue forever. Therefore, it is necessary to include a computational limitation as for $N$ rays the RayTracingStep requires $\mathcal{O}(N*BD)$ bits of memory with $DB$ being the maximum branching depth. Fortunately, in most cases, reflected rays lose intensity quickly hence it is possible to discard rays earlier when they reach the intensity cutoff and do not contribute anymore significantly to the force and torque.

## Installation

The library uses CMake (3.7 or higher) as its build system (3.18 or higher for GPU acceleration). It requires a compiler that supports C++14.
The recommended way to integrate LDPLAB is to clone the project and use **add_subdirectory**. 

The example and tests are only available if BUILD_LDPLAB_TEST_PROJECTS is ON.

## Usage

The following code demonstrates a simulation of the force and torque of a simple spherical particle with a linear index of refraction gradient.

Using LDPLAB takes three steps:

1. Define Setup
2. Compose Simulation Step
3. Execute Simulation

### 1. Define Setup

All LDPLAB objects and interfaces are defined in 
*ldplab.hpp*.

```cpp
#include <ldplab.hpp>

using namespace ldplab;
```

To create a simple experimental setup we create one particle with the built-in helper function PropertyGenerator::getSphericalParticleByRadius. To see a resulting force and torque, we tilt the particle slightly at a $45^\circ$ angle to the light source.

```cpp
ExperimentalSetup setup;

// Create particles
auto particle = PropertyGenerator::getSphereParticleByRadius(
    1,             // Radius 
    1.52,          // Average Particle index of refraction gradient
    0.2,           // Max difference of the index of refraction
    1,             // gradient direction +1 or -1;
    Vec3(0, 0, 0), // Particle position
    Vec3(0, M_PI/4.0, 0));  // Particle orientation in Euler angles
particle.rotation_order = RotationOrder::zyz; // Using Euler angles
setup.particles.emplace_back(std::move(particle));
```

The light source is homogeneous and collimated. We discard polarization effects so we also use an unpolarized light source.

```cpp
// Create light sources
LightSource ls;
ls.direction = std::make_shared<LightDirectionHomogeneous>();
ls.intensity_distribution = std::make_shared<LightDistributionHomogeneous>(1.0);
ls.polarization = std::make_shared<LightPolarisationUnpolarized>();
ls.orientation = Vec3(0, 0, 1.0);
ls.origin_corner = Vec3(-1,-1,-2);
ls.horizontal_direction = Vec3(1.0, 0, 0);
ls.horizontal_size = 2.0;
ls.vertical_direction = Vec3(0, 1.0, 0);
ls.vertical_size = 2.0;
setup.light_sources.emplace_back(std::move(ls));
```
We subject the particle to water which is defined by the *medium_refraction_index* of the experimental setup.

```cpp
// Set Medium Index of refraction
setup.medium_refraction_index = 1.33;
```
### 2. Compose Simulation Step

The *RayTracingStepCPUInfo* structure defines numerous system parameters. LDPLAB has a built-in thread pool for multithreading. The number of threads is provided with *number_parallel_pipelines*.

```cpp
RayTracingStepCPUInfo info;
info.number_parallel_pipelines = 8; // Number of threads
info.number_rays_per_buffer = 512;
info.maximum_branching_depth = 8; // Maximum number of ray splits
info.intensity_cutoff = 0.0005 * 1.0 / 50000.0; // CT * intensity / ray_density
info.emit_warning_on_maximum_branching_depth_discardment = false;
info.return_force_in_particle_coordinate_system = true;
```

To create a Ray Tracing Pipeline, we use *RayTracingStepFactory*. For that, we need to define the individual stages as following.

```cpp
using cpu_factory_init = rtscpu::default_factories::InitialStageHomogenousPolarizedLightBoundingSphereProjectionFactory;
using cpu_factory_intersection = rtscpu::default_factories::ParticleIntersectionPolarizedLightFactory;
using cpu_factory_interaction = rtscpu::default_factories::SurfaceInteractionPolarizedLightFactory;
using cpu_factory_propagation = rtscpu::default_factories::InnerParticlePropagationRK4PolarizationFactory;

PipelineConfiguration pipeline_config;
pipeline_config.initial_stage = 
    std::make_shared<factory_init>(std::sqrt(50000/M_PI)); // rays_per_unit_space = sqrt(ray_density/R^2 / PI) for normalization
pipeline_config.particle_intersection = std::make_shared<cpu_factory_intersection>();
pipeline_config.surface_interaction = std::make_shared<cpu_factory_interaction>();
pipeline_config.inner_particle_propagation = std::make_shared<cpu_factory_propagation>(RK4Parameter(0.001));
```

As we need the particle configuration for the *SimulationState* as well we copy it before moving it to the Factory.

```cpp
ExperimentalSetup setup_copy = setup;
std::shared_ptr<IRayTracingStep> rts =
    RayTracingStepFactory::createRayTracingStepCPU(
        rtscpu_info,
        std::move(setup_copy),
        pipeline_config,
        false);

if (rts == nullptr)
    return -1; // The factory could not build ray tracing pipeline
```

In case the individual Ray Tracing Stages aren't compatible, the factory returns a null pointer. To know which stages conflicted we can create an error log by subscribing the console to the internal logging system.

```cpp
// Prepare logging
LogCallbackStdout clog{};
clog.setLogLevel(LOG_LEVEL_ERROR);
clog.subscribe();
```

### 3. Execute Simulation

Finally, we are ready to perform the actual simulation. For that, we simply provide a *RayTracingStepOutput* structure and the current *SimulationState* to the created RayTracingStep and the force and torque are computed. Each particle is provided with an individual UID which we can use to access the results.

```cpp
// Create simulation state
SimulationState state{ setup };

// execute RayTracingStep
RayTracingStepOutput output;
rts->execute(state, output);

auto force = output.force_per_particle.at(particle.uid);
auto torque = output.torque_per_particle.at(particle.uid);
std::cout << "F = { "<< force.x <<", "<< force.y <<", "<< force.z <<"}" << std::endl;
std::cout << "T = { "<< torque.x <<", "<< torque.y <<", "<< torque.z <<"}" << std::endl;
```

## License

Copyright (c) 2022 Alexander Gerwing & Jonas Hallekamp

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

### Third-party licenses:

- GLM: MIT license
- TinyObjLoader: MIT license