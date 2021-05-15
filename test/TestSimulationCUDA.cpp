#include <ldplab.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>

constexpr double const_pi()
{
    return 3.14159265358979323846264338327950288419716939937510;
}

// Used particle geometry type
enum class GeometryType 
{
    rod,
    sphere,
    triangle_mesh
};
const GeometryType GEOMETRY_TYPE = GeometryType::rod;

// Folder path
const std::string OUTPUT_DIRECTORY = 
    "results\\cuda_";
const std::string OBJ_PATH = "sphere.obj";

// Particle Geometry
const double PARTICLE_VOLUME = 4.0 * const_pi() / 3.0;
// Particle geometry properties (rod particle)
const double ROD_PARTICLE_L = 2;
const double ROD_PARTICLE_KAPPA = 1.0;

// Particle material properties
const double PARTICLE_MATERIAL_INDEX_OF_REFRACTION = 1.51;
const double PARTICLE_MATERIAL_NU = 0.2;

// Light intensity properties
const double LIGHT_INTENSITY = 1;

// Reflexion index of the medium
const double MEDIUM_REFLEXION_INDEX = 1.33;

// Simulation properties
#ifdef _DEBUG
    const size_t NUM_RTS_THREADS = 1;
#else
    const size_t NUM_RTS_THREADS = 8;
#endif
const size_t NUM_RTS_RAYS_PER_BUFFER = 8192;
const double NUM_RTS_RAYS_PER_WORLD_SPACE_SQUARE_UNIT = 20000; //500000;
const size_t MAX_RTS_BRANCHING_DEPTH = 8;
const double RTS_INTENSITY_CUTOFF =  0.01 * LIGHT_INTENSITY /
    NUM_RTS_RAYS_PER_WORLD_SPACE_SQUARE_UNIT;
const size_t OCTREE_DEPTH = 5;

// RK4
const double RTS_SOLVER_STEP_SIZE = 0.05; //0.005;
// RK45
const double RTS_SOLVER_EPSILON = 0.0000001;
const double RTS_SOLVER_INITIAL_STEP_SIZE = 2.0;
const double RTS_SOLVER_SAFETY_FACTOR = 0.84;
const size_t NUM_SIM_ROTATION_STEPS = 314;

// Prototypes
std::ofstream getFileStream(const ldplab::Particle& particle,
    double nu,
    std::string path,
    std::string type,
    size_t branching_depth);
void plotProgress(double progress);
void createExperimentalSetup(ldplab::ExperimentalSetup& experimental_setup,
    double kappa,
    double gradient);
void runSimulation(const ldplab::ExperimentalSetup& experimental_setup,
    double kappa,
    double nu,
    size_t branching_depth);

int main()
{
    // Prepare logging
    ldplab::LogCallbackFileStream flog{ "logs/test_simulation_cuda.log" };
    ldplab::LogCallbackStdout clog{};
    flog.setLogLevel(ldplab::LOG_LEVEL_TRACE);
    clog.setLogLevel(ldplab::LOG_LEVEL_DEBUG);
    flog.subscribe();
    clog.subscribe();

    // Run simulations
    std::vector<double> vec_kappa = { 0.0 };
    if (GEOMETRY_TYPE == GeometryType::rod)
        vec_kappa.push_back(0.3);

    std::vector<double> vec_nu = { 0.15 };
    std::vector<size_t> vec_branching_depth = { 0, MAX_RTS_BRANCHING_DEPTH };
    for (size_t i = 0; i < vec_kappa.size(); ++i)
    {
        for (size_t j = 0; j < vec_nu.size(); ++j)
        {
            for (size_t k = 0; k < vec_branching_depth.size(); ++k)
            {
                // Create experimental setup
                ldplab::ExperimentalSetup experimental_setup;
                createExperimentalSetup(
                    experimental_setup,
                    vec_kappa[i],
                    vec_nu[j]);
                // Run simulation
                runSimulation(
                    experimental_setup,
                    vec_kappa[i],
                    vec_nu[j],
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

    constexpr char block_char = (char)219;
    constexpr char progress_char = '#'; //block_char;
    constexpr char no_progress_char = '_';

    std::string str_progress_bar(steps, no_progress_char);
    for (size_t i = 1; i <= steps; ++i)
    {
        if (i * 100 <= threshold)
            str_progress_bar[i - 1] = progress_char;
        else
            break;
    }

    std::cout << "Progress: [" << str_progress_bar << "] " <<
        iprogress << "%" << std::endl;
}

std::ofstream getFileStream(
    const ldplab::Particle& particle,
    double nu,
    std::string path,
    std::string type,
    size_t branching_depth)
{
    std::stringstream ss;
    std::ofstream file;
    if (GEOMETRY_TYPE == GeometryType::rod)
    {
        ldplab::RodParticleGeometry* geomerty =
            (ldplab::RodParticleGeometry*)particle.geometry.get();        
        ss << path << "rod_geometry_" << type <<
            "_nu" << nu <<
            "_l" << geomerty->l <<
            "_k" << geomerty->kappa << 
            "_bd" << branching_depth << ".txt";
        file = std::ofstream{ ss.str() };
        file << "# Parameter" << std::endl;
        file << "# V = " << PARTICLE_VOLUME << std::endl;
        file << "# l = " << geomerty->l << std::endl;
        file << "# kappa = " << geomerty->kappa << std::endl;
        file << "# R = " << geomerty->cylinder_radius << std::endl;
        file << "# L = " << geomerty->cylinder_length << std::endl;
    }
    else if (GEOMETRY_TYPE == GeometryType::sphere)
    {
        ldplab::SphericalParticleGeometry* geomerty =
            (ldplab::SphericalParticleGeometry*)particle.geometry.get();
        ss << path << "sphere_geometry_" << type <<
            "_nu" << nu <<
            "_V" << PARTICLE_VOLUME << 
            "_bd" << branching_depth << ".txt";
        file = std::ofstream{ ss.str() };
        file << "# Parameter" << std::endl;
        file << "# V = " << PARTICLE_VOLUME << std::endl;
        file << "# R = " << geomerty->radius << std::endl;
    }
    else if (GEOMETRY_TYPE == GeometryType::triangle_mesh)
    {
        ss << path << "mesh_geometry_" << type <<
            "_nu" << nu <<
            "_bd" << branching_depth << ".txt";
        file = std::ofstream{ ss.str() };
        file << "# Parameter" << std::endl;
        file << "# mesh = " << OBJ_PATH << std::endl;
    }
    file << "# nu = " << nu << std::endl;
    file << "# I = " << LIGHT_INTENSITY << std::endl;
    if (type[0] == 'f')
        file << "# theta\tF_x\tF_y\tF_z" << std::endl;
    else if (type[0] == 't')
        file << "# theta\tT_x\tT_y\tT_z" << std::endl;
    return file;
}

void createExperimentalSetup(
    ldplab::ExperimentalSetup& experimental_setup,
    double kappa,
    double nu)
{
    //nu = PARTICLE_MATERIAL_NU;
    // Create particle
    ldplab::Particle particle;
    double particle_world_extent;
    if (GEOMETRY_TYPE == GeometryType::sphere)
    {
        particle = ldplab::PropertyGenerator::getSphereParticleByVolume(
            PARTICLE_VOLUME,
            PARTICLE_MATERIAL_INDEX_OF_REFRACTION,
            nu,
            1.0,
            ldplab::Vec3(0.0, 0.0, 0.0),
            ldplab::Vec3(0.0, 0.0, 0.0));
    }
    else if (GEOMETRY_TYPE == GeometryType::rod)
    {
        // Rod Particle
        particle = ldplab::PropertyGenerator::getRodParticleConstVolume(
            PARTICLE_VOLUME,
            ROD_PARTICLE_L,
            kappa,
            PARTICLE_MATERIAL_INDEX_OF_REFRACTION,
            nu,
            1.0,
            ldplab::Vec3(0.0,0.0,0.0),
            ldplab::Vec3(0.0,0.0,0.0));
    }
    else if (GEOMETRY_TYPE == GeometryType::triangle_mesh)
    {
        // Load mesh
        std::vector<ldplab::Triangle> mesh;
        ldplab::AABB mesh_aabb;
        if (!ldplab::ObjLoader::loadTriangleMesh(OBJ_PATH, mesh, mesh_aabb))
            return;
        particle.position = ldplab::Vec3(0, 0, 0);
        particle.orientation = ldplab::Vec3(0, 0, 0);
        particle.geometry =
            std::make_shared<ldplab::TriangleMeshParticleGeometry>(mesh);
        ldplab::Vec3 bs_extent = mesh_aabb.max - mesh_aabb.min;
        ldplab::Vec3 bs_center = mesh_aabb.min + 0.5 * bs_extent;
        double bs_radius = glm::length(mesh_aabb.max - bs_center);
        particle.bounding_volume =
            std::make_shared<ldplab::BoundingVolumeSphere>(bs_center, bs_radius);
        const double delta_n = nu / 2.0; //nu / (2 * bs_radius);
        particle.material =
            std::make_shared<ldplab::ParticleMaterialLinearOneDirectional>(
                PARTICLE_MATERIAL_INDEX_OF_REFRACTION,
                delta_n,
                bs_center,
                ldplab::Vec3(0, 0, 1));
        particle.centre_of_mass = bs_center;
    }
    ldplab::BoundingVolumeSphere* bs =
        (ldplab::BoundingVolumeSphere*)particle.bounding_volume.get();
    particle_world_extent = bs->center.z + bs->radius;
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

    experimental_setup.particles.emplace_back(std::move(particle));
    experimental_setup.light_sources.emplace_back(std::move(light_source));
    experimental_setup.medium_reflection_index = MEDIUM_REFLEXION_INDEX;
}

void runSimulation(
    const ldplab::ExperimentalSetup& experimental_setup,
    double kappa,
    double nu,
    size_t branching_depth)
{
    // Start timing
    ldplab::Profiling::reset();
    std::chrono::steady_clock::time_point start =
        std::chrono::steady_clock::now();

    // Create ray tracing step
    double rts_step_size;
    ldplab::ParticleMaterialLinearOneDirectional* material =
        (ldplab::ParticleMaterialLinearOneDirectional*)experimental_setup.particles[0].material.get();
    if (std::abs(material->gradient) > 1e-5)
        rts_step_size = RTS_SOLVER_STEP_SIZE / std::abs(material->gradient);
    else
        rts_step_size = RTS_SOLVER_STEP_SIZE * std::pow(PARTICLE_VOLUME,1.0/3.0);
    ldplab::RayTracingStepCUDAInfo rtscuda_info;
    rtscuda_info.number_parallel_pipelines = NUM_RTS_THREADS;
    rtscuda_info.number_rays_per_buffer = NUM_RTS_RAYS_PER_BUFFER;
    ldplab::BoundingVolumeSphere* bs =
        (ldplab::BoundingVolumeSphere*)experimental_setup.particles[0].bounding_volume.get();
    rtscuda_info.light_source_ray_density_per_unit_area =
        NUM_RTS_RAYS_PER_WORLD_SPACE_SQUARE_UNIT / (bs->radius * bs->radius * const_pi());
    if (rtscuda_info.light_source_ray_density_per_unit_area == 0)
        rtscuda_info.light_source_ray_density_per_unit_area = 1;
    rtscuda_info.maximum_branching_depth = branching_depth;
    rtscuda_info.intensity_cutoff = RTS_INTENSITY_CUTOFF;
    rtscuda_info.solver_parameters = std::make_shared<ldplab::RK4Parameter>(
        rts_step_size);
    rtscuda_info.emit_warning_on_maximum_branching_depth_discardment = false;
    if (GEOMETRY_TYPE == GeometryType::triangle_mesh)
    {
        rtscuda_info.accelerator_structure_parameters =
            std::make_shared<ldplab::AcceleratorStructureOctreeParameter>(
                OCTREE_DEPTH);
    }

    std::shared_ptr<ldplab::IRayTracingStep> ray_tracing_step =
        ldplab::RayTracingStepFactory::createRayTracingStepCUDA(
            experimental_setup, rtscuda_info);

    if (ray_tracing_step == nullptr)
        return;

    // Output file
    ldplab::RayTracingStepOutput output;
    std::ofstream output_force = getFileStream(
        experimental_setup.particles[0], 
        nu, 
        OUTPUT_DIRECTORY, 
        "force", 
        branching_depth);
    std::ofstream output_torque = getFileStream(
        experimental_setup.particles[0], 
        nu, 
        OUTPUT_DIRECTORY, 
        "torque", 
        branching_depth);

    // Create simulation
    ldplab::SimulationState state{ experimental_setup };
    constexpr double offset = 0;
    constexpr double lim = const_pi();
    constexpr double step_size = (lim - offset) /
        static_cast<double>(NUM_SIM_ROTATION_STEPS - 1);
    constexpr double half_step_size = step_size / 2.0;

    ldplab::UID<ldplab::Particle> puid{ experimental_setup.particles[0].uid };
    
    for (double rotation_x = offset + step_size;
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

    std::stringstream identificator;
    identificator << "cuda_nu" << static_cast<int>(nu * 10000.0) <<
        "_k" << static_cast<int>(kappa * 100.0) <<
        "_l" << static_cast<int>(ROD_PARTICLE_L * 10.0) <<
        "_bd" << branching_depth <<
        "_u" << rtscuda_info.light_source_ray_density_per_unit_area <<
        "_rs" << NUM_SIM_ROTATION_STEPS;

    // Stop timing
    std::chrono::steady_clock::time_point end =
        std::chrono::steady_clock::now();
    const double elapsed_time = std::chrono::duration<double>(
        end - start).count();
    std::ofstream elapsed_time_file("logs/cuda_simulation_time_" + identificator.str());
    elapsed_time_file << elapsed_time << "s" << std::endl;

    // Profiling
    ldplab::Profiling::printReports("logs/cuda_profiling_report_" + identificator.str());
}
