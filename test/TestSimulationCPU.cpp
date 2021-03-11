#include <ldplab.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>

constexpr double const_pi()
{
    return 3.14159265358979323846264338327950288419716939937510;
}

// Folder path
const std::string DIR_PATH = 
    "D:\\Workspace\\Studium\\Master\\Masterarbeit\\Code\\Data\\LDPLAB\\Volume\\";

const bool SPHERICAL_PARTICLE = true;
const double PARTICLE_VOLUME = 1;
// Particle geometry properties (rod particle)
const double ROD_PARTICLE_L = 2;
const double ROD_PARTICLE_KAPPA = 1.0;

// Particle material properties
const double PARTICLE_MATERIAL_INDEX_OF_REFRACTION = 1.51;
const double PARTICLE_MATERIAL_NU = 0.15;

// Light intensity properties
const double LIGHT_INTENSITY = 1;

// Simulation properties
#ifdef _DEBUG
    const size_t NUM_RTS_THREADS = 1;
#else
    const size_t NUM_RTS_THREADS = 8;
#endif
const size_t NUM_RTS_RAYS_PER_BUFFER = 8192;
const double NUM_RTS_RAYS_PER_WORLD_SPACE_SQUARE_UNIT = 500000;
const size_t MAX_RTS_BRANCHING_DEPTH = 8;
const double RTS_INTENSITY_CUTOFF =  0.01 * LIGHT_INTENSITY /
    NUM_RTS_RAYS_PER_WORLD_SPACE_SQUARE_UNIT;

// RK4
const double RTS_SOLVER_STEP_SIZE = 0.005;
// RK45
const double RTS_SOLVER_EPSILON = 0.0000001;
const double RTS_SOLVER_INITIAL_STEP_SIZE = 2.0;
const double RTS_SOLVER_SAFETY_FACTOR = 0.84;
const size_t NUM_SIM_ROTATION_STEPS = 314;

std::ofstream getFileStream(ldplab::Particle& particle, std::string path, std::string type)
{
    std::stringstream ss;
    std::ofstream file;
    if (particle.geometry->type() == ldplab::IParticleGeometry::Type::rod_particle)
    {
        ldplab::RodParticleGeometry* geomerty =
            (ldplab::RodParticleGeometry*)particle.geometry.get();
        ss << path << type <<
            "_nu" << PARTICLE_MATERIAL_NU <<
            "_l" << geomerty->l <<
            "_k" << geomerty->kappa << ".txt";
        file = std::ofstream{ ss.str() };
        file << "# Parameter" << std::endl;
        file << "# V = " << PARTICLE_VOLUME << std::endl;
        file << "# l = " << geomerty->l << std::endl;
        file << "# kappa = " << geomerty->kappa << std::endl;
        file << "# R = " << geomerty->cylinder_radius << std::endl;
        file << "# L = " << geomerty->cylinder_length << std::endl;
    }
    else 
    {
        ldplab::SphericalParticleGeometry* geomerty =
            (ldplab::SphericalParticleGeometry*)particle.geometry.get();
        ss << path << type <<
            "_nu" << PARTICLE_MATERIAL_NU <<
            "_V" << PARTICLE_VOLUME << ".txt";
        file = std::ofstream{ ss.str() };
        file << "# Parameter" << std::endl;
        file << "# V = " << PARTICLE_VOLUME << std::endl;
        file << "# R = " << geomerty->radius << std::endl;
    }
    file << "# nu = " << PARTICLE_MATERIAL_NU << std::endl;
    file << "# I = " << LIGHT_INTENSITY << std::endl;
    if (type[0] == 'f')
        file << "# theta\tF_x\tF_y\tF_z" << std::endl;
    else if (type[0] == 't')
        file << "# theta\tT_x\tT_y\tT_z" << std::endl;

    return file;
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
    double particle_world_extent;
    double rts_step_size;
    if (SPHERICAL_PARTICLE)
    {
        particle = ldplab::getSphereParticle(
            PARTICLE_VOLUME,
            PARTICLE_MATERIAL_INDEX_OF_REFRACTION,
            PARTICLE_MATERIAL_NU,
            ldplab::Vec3(0.0, 0.0, 0.0),
            ldplab::Vec3(0.0, 0.0, 0.0));
        ldplab::ParticleMaterialLinearOneDirectional* material =
            (ldplab::ParticleMaterialLinearOneDirectional*)particle.material.get();
        ldplab::BoundingVolumeSphere* bs =
            (ldplab::BoundingVolumeSphere*)particle.bounding_volume.get();
        particle_world_extent = bs->center.z + bs->radius;
        if (std::abs(material->gradient) > 1e-5)
            rts_step_size = RTS_SOLVER_STEP_SIZE / std::abs(material->gradient);
        else
            rts_step_size = 0.1;
    }
    else
    {
        // Rod Particle
        particle = ldplab::getRodParticleConstVolume(
            PARTICLE_VOLUME,
            ROD_PARTICLE_L,
            ROD_PARTICLE_KAPPA,
            PARTICLE_MATERIAL_INDEX_OF_REFRACTION,
            PARTICLE_MATERIAL_NU,
            ldplab::Vec3(0.0,0.0,0.0),
            ldplab::Vec3(0.0,0.0,0.0));
        ldplab::ParticleMaterialLinearOneDirectional* material =
            (ldplab::ParticleMaterialLinearOneDirectional*)particle.material.get();
        ldplab::BoundingVolumeSphere* bs = 
            (ldplab::BoundingVolumeSphere*)particle.bounding_volume.get();
        particle_world_extent = bs->center.z + bs->radius;
        if (std::abs(material->gradient) > 1e-5)
            rts_step_size = RTS_SOLVER_STEP_SIZE / std::abs(material->gradient);
        else
            rts_step_size = 0.1;
    }


    // Create light source
    const double LIGHT_GEOMETRY_PLANE_EXTENT = 10 * particle_world_extent;
    const ldplab::Vec3 LIGHT_GEOMETRY_ORIGIN_CORNER =
        ldplab::Vec3(
            -LIGHT_GEOMETRY_PLANE_EXTENT / 2.0,
            -LIGHT_GEOMETRY_PLANE_EXTENT / 2.0,
            2 * particle_world_extent);
    ldplab::LightSource light_source;
    light_source.orientation = ldplab::Vec3(0, 0, -1.0);
    light_source.horizontal_direction = glm::normalize(ldplab::Vec3(1.0, 0, 0));
    light_source.vertical_direction = glm::normalize(ldplab::Vec3(0, 1.0, 0));
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
    ldplab::BoundingVolumeSphere* bs =
        (ldplab::BoundingVolumeSphere*)experimental_setup.particles[0].bounding_volume.get();
    rtscpu_info.light_source_ray_density_per_unit_area =
        NUM_RTS_RAYS_PER_WORLD_SPACE_SQUARE_UNIT / (bs->radius * bs->radius * const_pi());
    rtscpu_info.maximum_branching_depth = MAX_RTS_BRANCHING_DEPTH;
    rtscpu_info.intensity_cutoff = RTS_INTENSITY_CUTOFF;
    rtscpu_info.solver_parameters = std::make_shared<ldplab::RK4>(
        rts_step_size);
    rtscpu_info.emit_warning_on_maximum_branching_depth_discardment = false;
    std::shared_ptr<ldplab::IRayTracingStep> ray_tracing_step =
        ldplab::RayTracingStepFactory::createRayTracingStepCPU(
            experimental_setup, rtscpu_info);

    if (ray_tracing_step == nullptr)
        return -1;

    // Output file
    ldplab::RayTracingStepOutput output;
    std::ofstream output_force = getFileStream(experimental_setup.particles[0], DIR_PATH, "force");
    std::ofstream output_torque = getFileStream(experimental_setup.particles[0], DIR_PATH, "torque");

    // Create simulation
    ldplab::SimulationState state{ experimental_setup };
    constexpr double offset = 0.0;
    constexpr double lim = const_pi();
    constexpr double step_size = (lim - offset) /
        static_cast<double>(NUM_SIM_ROTATION_STEPS - 1);
    constexpr double half_step_size = step_size / 2.0;

    ldplab::UID<ldplab::Particle> puid{ experimental_setup.particles[0].uid };
    
    for (double rotation_x = offset;
        rotation_x < lim + half_step_size;
        rotation_x += step_size)
    {
        state.particle_instances[puid].orientation.x = rotation_x;
        ray_tracing_step->execute(state, output);
        output_force << rotation_x <<
            "\t" << output.force_per_particle[puid].x <<
            "\t" << output.force_per_particle[puid].y <<
            "\t" << output.force_per_particle[puid].z <<
            std::endl;
        output_torque << rotation_x <<
            "\t" << output.torque_per_particle[puid].x <<
            "\t" << output.torque_per_particle[puid].y <<
            "\t" << output.torque_per_particle[puid].z <<
            std::endl;
        plotProgress((rotation_x - offset) / (lim - offset));
    }

    thread_pool->terminate();
    return 0;
}