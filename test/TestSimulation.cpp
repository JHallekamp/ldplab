#include <ldplab.hpp>
#include <cmath>


// Particle geometry properties (rod particle)
const double ROD_PARTICLE_L = 2;
const double ROD_PARTICLE_KAPPA = 0.5;
const double ROD_PARTICLE_VOLUME_SPHERE_RADIUS = 10.0;
const double ROD_PARTICLE_CYLINDER_RADIUS = std::pow(2.0/3.0/ROD_PARTICLE_L,1.0/3.0) * ROD_PARTICLE_VOLUME_SPHERE_RADIUS;
const double ROD_PARTICLE_CYLINDER_HEIGHT = 2 * ROD_PARTICLE_L * ROD_PARTICLE_CYLINDER_RADIUS;

// Particle material properties
const double PARTICLE_MATERIAL_INDEX_OF_REFRACTION = 1.46;
const double PARTICLE_MATERIAL_GRADIENT = 0.2 / ROD_PARTICLE_CYLINDER_HEIGHT / 2;
const ldplab::Vec3 PARTICLE_MATERIAL_ORIGIN = ldplab::Vec3(0, 0, ROD_PARTICLE_CYLINDER_HEIGHT/2);
const ldplab::Vec3 PARTICLE_MATERIAL_DIRECTION = ldplab::Vec3(0, 0, 1);

// Particle bounding volume properties
const ldplab::Vec3 PARTICLE_BOUNDING_SPHERE_CENTER = ldplab::Vec3(0, 0, 
    (ROD_PARTICLE_CYLINDER_HEIGHT + ROD_PARTICLE_KAPPA * ROD_PARTICLE_CYLINDER_RADIUS) / 2);
const double PARTICLE_BOUNDING_SPHERE_RADIUS = 
    (ROD_PARTICLE_CYLINDER_HEIGHT + ROD_PARTICLE_KAPPA * ROD_PARTICLE_CYLINDER_RADIUS) / 2;

// Light geometry
const double LIGHT_GEOMETRY_PLANE_EXTENT = 4.0 * ROD_PARTICLE_CYLINDER_HEIGHT;
const ldplab::Vec3 LIGHT_GEOMETRY_ORIGIN_CORNER = 
    ldplab::Vec3(-LIGHT_GEOMETRY_PLANE_EXTENT, -LIGHT_GEOMETRY_PLANE_EXTENT, -LIGHT_GEOMETRY_PLANE_EXTENT);

// Light intensity properties
const double LIGHT_INTENSITY = 1.0;

// Simulation properties
const size_t NUM_RTS_THREADS = 8;
const size_t NUM_RTS_RAYS_PER_BUFFER = 512;
const double NUM_RTS_RAYS_PER_WORLD_SPACE_UNIT = 1000.0;
const size_t MAX_RTS_BRANCHING_DEPTH = 8;
const double RTS_INTENSITY_CUTOFF = 0.05;
const double RTS_SOLVER_EPSILON = 0.00001;
const double RTS_SOLVER_INITIAL_STEP_SIZE = 0.005;
const double RTS_SOLVER_SAFETY_FACTOR = 0.84;
const size_t NUM_SIM_ROTATION_STEPS = 32;

constexpr double const_pi() 
    { return 3.14159265358979323846264338327950288419716939937510; }

int main()
{
    // Prepare logging
    ldplab::LogCallbackFileStream flog{ "test_simulation.log" };
    ldplab::LogCallbackStdout clog{};
    flog.setLogLevel(ldplab::LOG_LEVEL_TRACE);
    clog.setLogLevel(ldplab::LOG_LEVEL_DEBUG);
    flog.subscribe();
    clog.subscribe();
   
    // Create particle
    ldplab::Particle rod_particle;
    rod_particle.position = ldplab::Vec3(0, 0, 0);
    rod_particle.orientation = ldplab::Vec3(0, 0, 0);
    rod_particle.material =
        std::make_shared<ldplab::ParticleMaterialLinearOneDirectional>(
            PARTICLE_MATERIAL_INDEX_OF_REFRACTION,
            PARTICLE_MATERIAL_GRADIENT,
            PARTICLE_MATERIAL_ORIGIN,
            PARTICLE_MATERIAL_DIRECTION);
    rod_particle.geometry =
        std::make_shared<ldplab::RodeParticleGeometry>(
            ROD_PARTICLE_CYLINDER_RADIUS,
            ROD_PARTICLE_CYLINDER_HEIGHT,
            ROD_PARTICLE_VOLUME_SPHERE_RADIUS,
            ROD_PARTICLE_KAPPA);
    rod_particle.bounding_volume =
        std::make_shared<ldplab::BoundingVolumeSphere>(
            PARTICLE_BOUNDING_SPHERE_CENTER,
            PARTICLE_BOUNDING_SPHERE_RADIUS);

    // Create light source
    ldplab::LightSource light_source;
    light_source.orientation = ldplab::Vec3(0, -1.0, 0);
    light_source.horizontal_direction = ldplab::Vec3(1.0, 0, 0);
    light_source.vertical_direction = ldplab::Vec3(0, 0, 1.0);
    light_source.horizontal_size = LIGHT_GEOMETRY_PLANE_EXTENT;
    light_source.vertical_size = LIGHT_GEOMETRY_PLANE_EXTENT;
    light_source.origin_corner = LIGHT_GEOMETRY_ORIGIN_CORNER;
    light_source.polarisation = 
        std::make_shared<ldplab::LightPolarisationUnpolarized>();
    light_source.direction =
        std::make_shared<ldplab::LightDirectionHomogenous>();
    light_source.intensity_distribution =
        std::make_shared<ldplab::LightDistributionHomogenous>(
            LIGHT_INTENSITY);

    // Create experimental setup
    std::shared_ptr<ldplab::ExperimentalSetup> experimental_setup =
        std::make_shared<ldplab::ExperimentalSetup>();
    experimental_setup->particles.emplace_back(std::move(rod_particle));
    experimental_setup->light_sources.emplace_back(std::move(light_source));

    // Thread pool
    std::shared_ptr<ldplab::ThreadPool> thread_pool =
        std::make_shared<ldplab::ThreadPool>(NUM_RTS_THREADS);

    // Create ray tracing step
    ldplab::RayTracingStepCPUInfo rtscpu_info;
    rtscpu_info.setup = experimental_setup;
    rtscpu_info.thread_pool = thread_pool;
    rtscpu_info.number_parallel_pipelines = NUM_RTS_THREADS;
    rtscpu_info.number_rays_per_buffer = NUM_RTS_RAYS_PER_BUFFER;
    rtscpu_info.number_rays_per_unit = NUM_RTS_RAYS_PER_WORLD_SPACE_UNIT;
    rtscpu_info.maximum_depth = MAX_RTS_BRANCHING_DEPTH;
    rtscpu_info.intensity_cutoff = RTS_INTENSITY_CUTOFF;
    rtscpu_info.epsilon = RTS_SOLVER_EPSILON;
    rtscpu_info.initial_step_size = RTS_SOLVER_INITIAL_STEP_SIZE;
    rtscpu_info.safety_factor = RTS_SOLVER_SAFETY_FACTOR;
    std::shared_ptr<ldplab::IRayTracingStep> ray_tracing_step =
        ldplab::RayTracingStepFactory::createRayTracingStepCPU(rtscpu_info);

    if (ray_tracing_step == nullptr)
        return -1;

    // Create simulation
    ldplab::SimulationState state;
    state.particles.emplace_back();
    state.particles.back().position = 
        experimental_setup->particles[0].position;
    state.particles.back().orientation =
        experimental_setup->particles[0].orientation;
    constexpr double lim = 2 * const_pi();
    constexpr double step_size = lim /
        static_cast<double>(NUM_SIM_ROTATION_STEPS);

    ldplab::RayTracingStepOutput output;
    for (double rotation_z = 0.0; rotation_z <= lim; rotation_z += step_size)
    {
        state.particles.back().orientation.z = rotation_z;
        ray_tracing_step->execute(state, output);
        // TODO: use or stroe output
    }

    thread_pool->terminate();
    return 0;
}