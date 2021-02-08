#include <ldplab.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>

// Particle geometry properties (rod particle)
const bool ROD_PARTICLE = true;
const double ROD_SURFACE_AREA = 1;
const double ROD_PARTICLE_L = 2;
const double ROD_PARTICLE_KAPPA = 0.0;

// Particle material properties
const double PARTICLE_MATERIAL_INDEX_OF_REFRACTION = 1.46;
const double PARTICLE_MATERIAL_NU = 0 * 0.2;

// Light geometry
const double LIGHT_GEOMETRY_PLANE_EXTENT = 10000;
const ldplab::Vec3 LIGHT_GEOMETRY_ORIGIN_CORNER =
ldplab::Vec3(
    -LIGHT_GEOMETRY_PLANE_EXTENT / 2.0,
    50,
    -LIGHT_GEOMETRY_PLANE_EXTENT / 2.0);

// Light intensity properties
const double LIGHT_INTENSITY = 1;

// Simulation properties
const size_t NUM_RTS_THREADS = 8;
const size_t NUM_RTS_RAYS_PER_BUFFER = 8192;
const double NUM_RTS_RAYS_PER_WORLD_SPACE_SQUARE_UNIT = 1500000;
const size_t MAX_RTS_BRANCHING_DEPTH = 8;
const double RTS_INTENSITY_CUTOFF =  0.01 * LIGHT_INTENSITY /
    NUM_RTS_RAYS_PER_WORLD_SPACE_SQUARE_UNIT;
const double RTS_SOLVER_EPSILON = 0.0000001;
const double RTS_SOLVER_INITIAL_STEP_SIZE = 2.0;
const double RTS_SOLVER_SAFETY_FACTOR = 0.84;
const size_t NUM_SIM_ROTATION_STEPS = 512;

constexpr double const_pi()
{
    return 3.14159265358979323846264338327950288419716939937510;
}

void plotProgress(double progress)
{
    const double progress_epsilon = 0.00001;
    const size_t steps = 40;
    const size_t iprogress =
        static_cast<size_t>((progress + progress_epsilon) * 100.0);
    const size_t threshold = steps * iprogress; // Includes factor of 100

    if (threshold == steps)
        bool a = true;

    std::string str_progress_bar(steps, ' ');
    for (size_t i = 1; i <= steps; ++i)
    {
        if (i * 100 <= threshold)
            str_progress_bar[i - 1] = (char)219; // Block character
    }

    std::cout << "Progress: [" << str_progress_bar << "] " <<
        iprogress << "%" << std::endl;
}

int main()
{
    // Prepare logging
    ldplab::LogCallbackFileStream flog{ "test_simulation_cpu.log" };
    ldplab::LogCallbackStdout clog{};
    flog.setLogLevel(ldplab::LOG_LEVEL_TRACE);
    clog.setLogLevel(ldplab::LOG_LEVEL_DEBUG);
    flog.subscribe();
    clog.subscribe();

    // Create particle
    ldplab::Particle particle;
    if (ROD_PARTICLE)
    {
        // Rod Particle
        particle = ldplab::getRodParticle(
            ROD_SURFACE_AREA,
            ROD_PARTICLE_L,
            ROD_PARTICLE_KAPPA,
            PARTICLE_MATERIAL_INDEX_OF_REFRACTION,
            PARTICLE_MATERIAL_NU,
            ldplab::Vec3(0.0,0.0,0.0),
            ldplab::Vec3(0.0,0.0,0.0));
    }

    // Create light source
    ldplab::LightSource light_source;
    light_source.orientation = ldplab::Vec3(0, -1.0, 0);
    light_source.horizontal_direction = glm::normalize(ldplab::Vec3(1.0, 0, 0));
    light_source.vertical_direction = glm::normalize(ldplab::Vec3(0, 0, 1.0));
    light_source.horizontal_size = LIGHT_GEOMETRY_PLANE_EXTENT;
    light_source.vertical_size = LIGHT_GEOMETRY_PLANE_EXTENT;
    light_source.origin_corner = LIGHT_GEOMETRY_ORIGIN_CORNER;
    light_source.polarisation =
        std::make_shared<ldplab::LightPolarisationUnpolarized>();
    light_source.direction =
        std::make_shared<ldplab::LightDirectionHomogeneous>();
    light_source.intensity_distribution =
        std::make_shared<ldplab::LightDistributionHomogeneous>(
            LIGHT_INTENSITY);

    // Create experimental setup
    ldplab::ExperimentalSetup experimental_setup;
    experimental_setup.particles.emplace_back(std::move(particle));
    experimental_setup.light_sources.emplace_back(std::move(light_source));
    experimental_setup.medium_reflection_index = 1.33;

    // Thread pool
    std::shared_ptr<ldplab::ThreadPool> thread_pool =
        std::make_shared<ldplab::ThreadPool>(NUM_RTS_THREADS);

    // Create ray tracing step
    ldplab::RayTracingStepCPUInfo rtscpu_info;
    rtscpu_info.thread_pool = thread_pool;
    rtscpu_info.number_parallel_pipelines = NUM_RTS_THREADS;
    rtscpu_info.number_rays_per_buffer = NUM_RTS_RAYS_PER_BUFFER;
    rtscpu_info.light_source_ray_density_per_unit_area =
        NUM_RTS_RAYS_PER_WORLD_SPACE_SQUARE_UNIT;
    rtscpu_info.maximum_branching_depth = MAX_RTS_BRANCHING_DEPTH;
    rtscpu_info.intensity_cutoff = RTS_INTENSITY_CUTOFF;
    rtscpu_info.solver_parameters = std::make_shared<ldplab::RK45>(
        RTS_SOLVER_INITIAL_STEP_SIZE, RTS_SOLVER_EPSILON, RTS_SOLVER_SAFETY_FACTOR);
    rtscpu_info.emit_warning_on_maximum_branching_depth_discardment = false;
    std::shared_ptr<ldplab::IRayTracingStep> ray_tracing_step =
        ldplab::RayTracingStepFactory::createRayTracingStepCPU(
            experimental_setup, rtscpu_info);

    if (ray_tracing_step == nullptr)
        return -1;

    // Create simulation
    ldplab::SimulationState state{ experimental_setup };
    constexpr double offset = 0;
    constexpr double lim = 2* const_pi();
    constexpr double step_size = (lim - offset) /
        static_cast<double>(NUM_SIM_ROTATION_STEPS - 1);
    constexpr double half_step_size = step_size / 2.0;
    constexpr double angle_shift = const_pi() / 2.0;
    // Output file
    ldplab::RayTracingStepOutput output;
    std::stringstream ss;
    ss << "D:\\Datein\\Studium\\Master\\Masterarbeit\\Code\\SimData\\force\\";
    if (ROD_PARTICLE)
        ss << "force_rod_g" << static_cast<int>(PARTICLE_MATERIAL_NU * 100.0) <<
        "_k" << static_cast<int>(ROD_PARTICLE_KAPPA * 100.0) <<
        "_l" << static_cast<int>(ROD_PARTICLE_L * 10.0); //<<
            //"_bd" << MAX_RTS_BRANCHING_DEPTH; // <<
            //"_u" << NUM_RTS_RAYS_PER_WORLD_SPACE_SQUARE_UNIT <<
            //"_rk" << std::log10(RTS_SOLVER_EPSILON);
    std::ofstream output_file(ss.str());
    
    ldplab::UID<ldplab::Particle> puid{ experimental_setup.particles[0].uid };
    state.particle_instances[puid].orientation.z = const_pi();
    state.particle_instances[puid].rotation_order = ldplab::RotationOrder::zxy;
    
    for (double rotation_x = offset + angle_shift;
        rotation_x < lim + angle_shift + half_step_size;
        rotation_x += step_size)
    {
        state.particle_instances[puid].orientation.x = rotation_x;
        ray_tracing_step->execute(state, output);
        output_file << rotation_x - const_pi() / 2 <<
            "\t" << output.force_per_particle[puid].x <<
            "\t" << output.force_per_particle[puid].y <<
            "\t" << output.force_per_particle[puid].z <<
            std::endl;
        plotProgress((rotation_x - offset - angle_shift) / (lim - offset));
    }

    thread_pool->terminate();
    return 0;
}