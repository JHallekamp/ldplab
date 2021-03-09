#include <ldplab.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>

// Particle geometry properties (rod particle)
const double ROD_PARTICLE_L = 2;
const double ROD_PARTICLE_KAPPA = 0;
const double ROD_PARTICLE_VOLUME_SPHERE_RADIUS = 10.0;
const double ROD_PARTICLE_CYLINDER_RADIUS = 
    std::pow(2.0/3.0/ROD_PARTICLE_L,1.0/3.0) * ROD_PARTICLE_VOLUME_SPHERE_RADIUS;
const double ROD_PARTICLE_CYLINDER_HEIGHT = 
    2 * ROD_PARTICLE_L * ROD_PARTICLE_CYLINDER_RADIUS;

// Particle material properties
const double PARTICLE_MATERIAL_INDEX_OF_REFRACTION = 1.46;
const double PARTICLE_MATERIAL_GRADIENT = 0.2 / ROD_PARTICLE_CYLINDER_HEIGHT / 2;
const ldplab::Vec3 PARTICLE_MATERIAL_ORIGIN = 
    ldplab::Vec3(0, 0, ROD_PARTICLE_CYLINDER_HEIGHT/2);
const ldplab::Vec3 PARTICLE_MATERIAL_DIRECTION = ldplab::Vec3(0, 0, 1);

// Particle bounding volume properties
const ldplab::Vec3 PARTICLE_BOUNDING_SPHERE_CENTER = ldplab::Vec3(0, 0, 
    (ROD_PARTICLE_CYLINDER_HEIGHT + ROD_PARTICLE_KAPPA * ROD_PARTICLE_CYLINDER_RADIUS) / 2);
const double PARTICLE_BOUNDING_SPHERE_RADIUS = 
   std::sqrt(
       std::pow((ROD_PARTICLE_CYLINDER_HEIGHT + ROD_PARTICLE_KAPPA * ROD_PARTICLE_CYLINDER_RADIUS) / 2,2.0) + 
       ROD_PARTICLE_CYLINDER_RADIUS * ROD_PARTICLE_CYLINDER_RADIUS);

// Light geometry
const double LIGHT_GEOMETRY_PLANE_EXTENT = 4.0 * PARTICLE_BOUNDING_SPHERE_RADIUS;
const ldplab::Vec3 LIGHT_GEOMETRY_ORIGIN_CORNER = 
    ldplab::Vec3(
        -LIGHT_GEOMETRY_PLANE_EXTENT / 2.0, 
        (ROD_PARTICLE_CYLINDER_HEIGHT + ROD_PARTICLE_KAPPA * ROD_PARTICLE_CYLINDER_RADIUS) * 2.0,
        -LIGHT_GEOMETRY_PLANE_EXTENT / 2.0);

// Light intensity properties
const double LIGHT_INTENSITY =  0.1 / 2.99792458;

// Reflexion index of the medium
const double MEDIUM_REFLEXION_INDEX = 1.33;

// Simulation properties
const size_t NUM_RTS_THREADS = 1;
const size_t NUM_RTS_RAYS_PER_BUFFER = 1024 * 64;
const double NUM_RTS_RAYS_PER_WORLD_SPACE_SQUARE_UNIT = 256.0;
const size_t MAX_RTS_BRANCHING_DEPTH = 0;
const double RTS_INTENSITY_CUTOFF = 0.01 * LIGHT_INTENSITY  / 
    NUM_RTS_RAYS_PER_WORLD_SPACE_SQUARE_UNIT;
const double RTS_SOLVER_EPSILON = 0.0000001;
const double RTS_SOLVER_INITIAL_STEP_SIZE = 0.5;
const double RTS_SOLVER_SAFETY_FACTOR = 0.84;
const size_t NUM_SIM_ROTATION_STEPS = 48;
const std::string BASE_SHADER_DIRECTORY = "shader/";

constexpr double const_pi() 
    { return 3.14159265358979323846264338327950288419716939937510; }

// Prototypes
void plotProgress(double progress);
void createExperimentalSetup(
    ldplab::ExperimentalSetup& experimental_setup,
    double kappa,
    double gradient);
void runSimulation(
    const ldplab::ExperimentalSetup& experimental_setup,
    double kappa,
    double gradient,
    size_t branching_depth);

int main()
{
    // Prepare logging
    ldplab::LogCallbackFileStream flog{ "test_simulation_gpu.log" };
    ldplab::LogCallbackStdout clog{};
    flog.setLogLevel(ldplab::LOG_LEVEL_TRACE);
    clog.setLogLevel(ldplab::LOG_LEVEL_DEBUG);
    flog.subscribe();
    clog.subscribe();

    // Run simulations
    std::vector<double> vec_gradient = { 0.0, PARTICLE_MATERIAL_GRADIENT };
    std::vector<double> vec_kappa = { 0.0, 0.3 };
    std::vector<size_t> vec_branching_depth = { 0, 8 };
    for (size_t i = 0; i < vec_kappa.size(); ++i)
    {
        for (size_t j = 0; j < vec_gradient.size(); ++j)
        {
            for (size_t k = 0; k < vec_branching_depth.size(); ++k)
            {
                // Create experimental setup
                ldplab::ExperimentalSetup experimental_setup;
                createExperimentalSetup(
                    experimental_setup,
                    vec_kappa[i],
                    vec_gradient[j]);
                // Run simulation
                runSimulation(
                    experimental_setup,
                    vec_kappa[i],
                    vec_gradient[j],
                    vec_branching_depth[k]);
            }
        }
    }

    return 0;
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

void createExperimentalSetup(
    ldplab::ExperimentalSetup& experimental_setup,
    double kappa,
    double gradient)
{
    // Create particle
    ldplab::Particle rod_particle;
    rod_particle.position = ldplab::Vec3(0, 0, 0);
    rod_particle.orientation = ldplab::Vec3(0, 0, 0);
    rod_particle.material =
        std::make_shared<ldplab::ParticleMaterialLinearOneDirectional>(
            PARTICLE_MATERIAL_INDEX_OF_REFRACTION,
            gradient,
            PARTICLE_MATERIAL_ORIGIN,
            PARTICLE_MATERIAL_DIRECTION);
    rod_particle.geometry =
        std::make_shared<ldplab::RodParticleGeometry>(
            ROD_PARTICLE_CYLINDER_RADIUS,
            ROD_PARTICLE_CYLINDER_HEIGHT,
            ROD_PARTICLE_VOLUME_SPHERE_RADIUS,
            kappa);
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
        std::make_shared<ldplab::LightDirectionHomogeneous>();
    light_source.intensity_distribution =
        std::make_shared<ldplab::LightDistributionHomogeneous>(
            LIGHT_INTENSITY);

    // Set experimental setup
    experimental_setup.particles.emplace_back(std::move(rod_particle));
    experimental_setup.light_sources.emplace_back(std::move(light_source));
    experimental_setup.medium_reflection_index = MEDIUM_REFLEXION_INDEX;
}

void runSimulation(
    const ldplab::ExperimentalSetup& experimental_setup, 
    double kappa,
    double gradient,
    size_t branching_depth)
{
    // Start timing
    ldplab::Profiling::reset();
    std::chrono::steady_clock::time_point start =
        std::chrono::steady_clock::now();

    // Create ray tracing step
    ldplab::RayTracingStepGPUOpenGLInfo rtsgpu_info;
    rtsgpu_info.number_parallel_pipelines = NUM_RTS_THREADS;
    rtsgpu_info.number_rays_per_buffer = NUM_RTS_RAYS_PER_BUFFER;
    rtsgpu_info.light_source_ray_density_per_unit_area =
        NUM_RTS_RAYS_PER_WORLD_SPACE_SQUARE_UNIT;
    rtsgpu_info.maximum_branching_depth = branching_depth;
    rtsgpu_info.intensity_cutoff = RTS_INTENSITY_CUTOFF;
    rtsgpu_info.solver_parameters = std::make_shared<ldplab::RK45>(
        RTS_SOLVER_INITIAL_STEP_SIZE, RTS_SOLVER_EPSILON, RTS_SOLVER_SAFETY_FACTOR);
    rtsgpu_info.shader_base_directory_path = BASE_SHADER_DIRECTORY;
    rtsgpu_info.emit_warning_on_maximum_branching_depth_discardment = false;
    std::shared_ptr<ldplab::IRayTracingStep> ray_tracing_step =
        ldplab::RayTracingStepFactory::createRayTracingStepGPUOpenGL(
            experimental_setup, rtsgpu_info);

    if (ray_tracing_step == nullptr)
        return;

    // Create simulation
    ldplab::SimulationState state{ experimental_setup };
    constexpr double offset = 0;
    constexpr double lim = const_pi(); //2 * const_pi();
    constexpr double step_size = (lim - offset) /
        static_cast<double>(NUM_SIM_ROTATION_STEPS - 1);
    constexpr double half_step_size = step_size / 2.0;
    constexpr double angle_shift = const_pi() / 2.0;
    ldplab::RayTracingStepOutput output;

    std::stringstream identificator;
    identificator << "gpu_force_g" << static_cast<int>(gradient * 10000.0) <<
        "_k" << static_cast<int>(kappa * 100.0) <<
        "_l" << static_cast<int>(ROD_PARTICLE_L * 10.0) <<
        "_bd" << branching_depth <<
        "_u" << NUM_RTS_RAYS_PER_WORLD_SPACE_SQUARE_UNIT <<
        "_rs" << NUM_SIM_ROTATION_STEPS;
    std::ofstream output_file("force_" + identificator.str());

    // Run simulation
    ldplab::UID<ldplab::Particle> puid{ experimental_setup.particles[0].uid };
    for (double rotation_x = offset + angle_shift;
        rotation_x < lim + angle_shift + half_step_size;
        rotation_x += step_size)
    {
        state.particle_instances[puid].orientation.x = rotation_x;
        ray_tracing_step->execute(state, output);
        output_file << rotation_x - const_pi() / 2 << "\t" <<
            glm::length(output.force_per_particle[puid]) << std::endl;
        plotProgress((rotation_x - offset - angle_shift) / (lim - offset));
    }

    // Stop timing
    std::chrono::steady_clock::time_point end =
        std::chrono::steady_clock::now();
    const double elapsed_time = std::chrono::duration<double>(
        end - start).count();
    std::ofstream elapsed_time_file("gpu_simulation_time_" + identificator.str());
    elapsed_time_file << elapsed_time << "s" << std::endl;

    // Profiling
    ldplab::Profiling::printReports("gpu_profiling_report_" + identificator.str());
}
