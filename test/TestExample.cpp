#define _USE_MATH_DEFINES
#include <ldplab.hpp>
#include <iostream>
#include <math.h>

#include <LDPLAB/RayTracingStep/CPU/PipelineConfiguration.hpp>
#include <LDPLAB/RayTracingStep/CPU/DefaultInitialStageFactories.hpp>
#include <LDPLAB/RayTracingStep/CPU/DefaultInnerParticlePropagationFactories.hpp>
#include <LDPLAB/RayTracingStep/CPU/DefaultParticleIntersectionFactories.hpp>
#include <LDPLAB/RayTracingStep/CPU/DefaultSurfaceInteractionFactories.hpp>

using namespace ldplab;

int main()
{
    // Prepare logging
    LogCallbackStdout clog{};
    clog.setLogLevel(LOG_LEVEL_ERROR);
    clog.subscribe();

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

    // Set Medium Index of refraction
    setup.medium_refraction_index = 1.33;

    // Compose Simulation Step
    RayTracingStepCPUInfo info;
    info.number_parallel_pipelines = 16; // Number of threads
    info.number_rays_per_buffer = 512;
    info.maximum_branching_depth = 8; // Maxium number of ray splits
    info.intensity_cutoff = 0.0005 * 1.0 / 50000.0; // CT * intensity / ray_density
    info.emit_warning_on_maximum_branching_depth_discardment = false;
    info.return_force_in_particle_coordinate_system = true;

    
    using cpu_factory_init = rtscpu::default_factories::InitialStageHomogenousLightBoundingSphereProjectionFactory;
    using cpu_factory_intersection = rtscpu::default_factories::ParticleIntersectionFactory;
    using cpu_factory_interaction = rtscpu::default_factories::SurfaceInteractionFactory;
    using cpu_factory_propagation = rtscpu::default_factories::InnerParticlePropagationRK4Factory;

    rtscpu::PipelineConfiguration pipeline_config;
    pipeline_config.initial_stage = std::make_shared<cpu_factory_init>(
        std::sqrt(50000/M_PI)); // sqrt(ray_density/R^2 / PI)
    pipeline_config.particle_intersection = std::make_shared<cpu_factory_intersection>();
    pipeline_config.surface_interaction = std::make_shared<cpu_factory_interaction>();
    pipeline_config.inner_particle_propagation = std::make_shared<cpu_factory_propagation>(RK4Parameter(0.001));



    ExperimentalSetup setup_copy = setup;
    std::shared_ptr<IRayTracingStep> rts =
        RayTracingStepFactory::createRayTracingStepCPU(
            info,
            std::move(setup_copy),
            pipeline_config,
            false);

    if (rts == nullptr)
        return -1; // Factories could not build ray tracing pipeline

    // Create simulation state
    SimulationState state{ setup };

    // execute ray tracing step
    RayTracingStepOutput output;
    rts->execute(state, output);

    auto force = output.force_per_particle.at(setup.particles[0].uid);
    auto torque = output.torque_per_particle.at(setup.particles[0].uid);
    std::cout << "F = { "<< force.x <<", "<< force.y <<", "<< force.z <<"}" << std::endl;
    std::cout << "T = { "<< torque.x <<", "<< torque.y <<", "<< torque.z <<"}" << std::endl;
    return 0;
}