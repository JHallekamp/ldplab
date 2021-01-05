#include "InitialStage.hpp"
#include "Context.hpp"
#include "../../Log.hpp"

ldplab::rtscpu::InitialStageBoundingSpheresHomogenousLight::
    InitialStageBoundingSpheresHomogenousLight(
        std::shared_ptr<Context> context)
    :
    m_context{ context },
    m_bounding_spheres{ ((BoundingSphereData*)
        context->bounding_volume_data.get())->sphere_data },
    m_batch_creation_light_index{ 0 },
    m_batch_creation_particle_index{ 0 },
    m_batch_creation_particle_initialized { false },
    m_rasterization_x{ 0 },
    m_rasterization_y{ 0 },
    m_rasterization_up{ true },
    m_rasterization_right{ true },
    m_rasterization_step_size{ 0 }
{
    LDPLAB_LOG_INFO("RTSCPU context %i: "\
        "InitialStageBoundingSpheresHomogenousLight instance created",
        m_context->uid);
    //LDPLAB_LOG_INFO("RTSCPU context %i: Created initial stage for bounding"\
    //    " spheres and homogenous light sources", m_context->uid);
}

void ldplab::rtscpu::InitialStageBoundingSpheresHomogenousLight::setup()
{
    LDPLAB_LOG_DEBUG("RTSCPU context %i: Project particles on light sources",
        m_context->uid);

    struct ProjectionPerLight
    {
        ProjectionPerLight()
            :
            center{ 0, 0 },
            radius{ 0 },
            depth{ 0 },
            particle_index{ 0 },
            light_index{ 0 },
            overlaps{ }
        { }
        Vec2 center;
        double radius;
        double depth;
        // Particle index
        size_t particle_index;
        // Light source index
        size_t light_index;
        // Indices of overlapping projections
        std::vector<size_t> overlaps;
    };

    // Projections per light source
    std::vector<std::vector<ProjectionPerLight>> projection_per_light_source;

    for (size_t i = 0; i < m_context->light_sources.size(); ++i)
    {
        const Vec3 plane_base = m_context->light_sources[i].origin_corner;
        const Vec3 light_direction = m_context->light_sources[i].orientation;

        const double division_term = 
            1.0 / -glm::dot(light_direction, light_direction);

        std::vector<ProjectionPerLight> light_projections;
        for (size_t j = 0; j < m_context->particles.size(); ++j)
        {
            //BoundingVolumeSphere* bounding_sphere = (BoundingVolumeSphere*) 
            //    m_context->particles[j].bounding_volume.get();
            ParticleTransformation& trans = m_context->
                particle_transformations[j];
            //const Vec3 bounding_sphere_center_world = trans.p2w_scale_rotation *
            //    bounding_sphere->center + trans.p2w_translation;
            const BoundingVolumeSphere& bounding_sphere =
                m_bounding_spheres[j];

            const double t =
                glm::dot(light_direction, 
                    plane_base - bounding_sphere.center) * division_term;

            if (t < 0)
                continue;

            const Vec3 wrldctr = bounding_sphere.center - t * light_direction;
            const Vec3 planectr = wrldctr - plane_base;
            const Vec2 center{ 
                glm::dot(planectr,  m_context->light_sources[i].horizontal_direction),
                glm::dot(planectr,  m_context->light_sources[i].vertical_direction) };
            const double radius = bounding_sphere.radius;
            if (!projLightOverlap(center, radius, m_context->light_sources[i]))
            {
                LDPLAB_LOG_TRACE("RTSCPU context %i: Particle %i has no"\
                    " projection onto light source %i",
                    m_context->uid, j, i);
                continue;
            }

            LDPLAB_LOG_DEBUG("RTSCPU context %i: Particle %i is projected"\
                " onto light source %i",
                m_context->uid, j, i);

            ProjectionPerLight proj;
            proj.particle_index = j;
            proj.light_index = i;
            proj.center = center;
            proj.radius = radius;
            proj.depth = glm::length(wrldctr - bounding_sphere.center);

            for (size_t k = 0; k < light_projections.size(); ++k)
            {
                if ((light_projections[k].center - proj.center).length() <=
                    (light_projections[k].radius + proj.radius))
                {
                    if (light_projections[k].depth < proj.depth)
                        proj.overlaps.push_back(k);
                    else
                        light_projections[k].overlaps.push_back(
                            light_projections.size());
                }
            }

            light_projections.push_back(proj);
        }
        projection_per_light_source.push_back(light_projections);
    }

    // Sort projections of each light source into projections per particle
    m_projections_per_particle.resize(m_context->particles.size());
    for (size_t i = 0; i < m_projections_per_particle.size(); ++i)
        m_projections_per_particle[i].clear();

    for (size_t i = 0; i < projection_per_light_source.size(); ++i)
    {
        for (size_t j = 0; j < projection_per_light_source[i].size(); ++j)
        {
            const size_t particle_index =
                projection_per_light_source[i][j].particle_index;
            projection_per_light_source[i][j].light_index =
                m_projections_per_particle[particle_index].size();;
            Projection proj;
            proj.center = projection_per_light_source[i][j].center;
            proj.radius = projection_per_light_source[i][j].radius;
            proj.depth = projection_per_light_source[i][j].depth;
            proj.light_index = i;
            m_projections_per_particle[particle_index].push_back(proj);
        }
    }

    // Now that all projections are present in matrix of projections, we can
    // add the pointers to overlapping projections
    for (size_t i = 0; i < projection_per_light_source.size(); ++i)
    {
        for (size_t j = 0; j < projection_per_light_source[i].size(); ++j)
        {
            const ProjectionPerLight* const original = 
                &projection_per_light_source[i][j];
            for (size_t k = 0; k < original->overlaps.size(); ++k)
            {
                const ProjectionPerLight* const overlap =
                    &projection_per_light_source[i][original->overlaps[k]];
                const size_t oli = original->light_index;
                const size_t opi = original->particle_index;
                const size_t ovli = overlap->light_index;
                const size_t ovpi = overlap->particle_index;
                m_projections_per_particle[opi][oli].overlaps.push_back(
                    &m_projections_per_particle[ovpi][ovli]);
            }
        }
    }

    // Reset batch creation values
    m_batch_creation_particle_index = 0;
    m_batch_creation_light_index = 0;
    m_batch_creation_particle_initialized = false;
    m_rasterization_step_size = 
        1.0 / static_cast<double>(m_context->parameters.number_rays_per_unit);

    LDPLAB_LOG_DEBUG("RTSCPU context %i: Particle projections created",
        m_context->uid);
}

bool ldplab::rtscpu::InitialStageBoundingSpheresHomogenousLight::projLightOverlap(
    const Vec2& center,
    const double radius,
    const LightSource& light_source) const
{
    if (center.x + radius < 0 || 
        center.x - radius > light_source.horizontal_size)
        return false;
    else if (center.y + radius < 0 || 
        center.y - radius > light_source.vertical_size)
        return false;
    
    if (center.x >= 0 && center.x <= light_source.horizontal_size)
        return true;
    if (center.y >= 0 && center.y <= light_source.vertical_size)
        return true;

    const double r2 = radius * radius;
    const double x2 = center.x * center.x;
    const double y2 = center.y * center.y;
    const double vx2 = (light_source.vertical_size - center.x) *
        (light_source.vertical_size - center.x);
    const double hy2 = (light_source.horizontal_size - center.y) *
        (light_source.horizontal_size - center.y);

    if ((r2 < x2 + y2) &&
        (r2 < vx2 + y2) &&
        (r2 < x2 + hy2) &&
        (r2 < vx2 + hy2))
        return false;

    return true;
}

bool ldplab::rtscpu::InitialStageBoundingSpheresHomogenousLight::hasToCreateRay
    (const Projection& projection, const LightSource& light_source) const
{
    const double ro_xxyy = m_rasterization_x * m_rasterization_x
        + m_rasterization_y * m_rasterization_y;
    const double ro_2x = 2.0 * m_rasterization_x;
    const double ro_2y = 2.0 * m_rasterization_y;

    const double pr_rr = projection.radius * projection.radius;
    const double pr_dist = ro_xxyy
        - projection.center.x * ro_2x
        - projection.center.y * ro_2y
        + projection.center.x * projection.center.x
        + projection.center.y * projection.center.y;
    if (pr_rr < pr_dist)
        return false;

    for (size_t i = 0; i < projection.overlaps.size(); ++i)
    {
        const Projection* const t_pr = projection.overlaps[i];
        const double t_rr = t_pr->radius * t_pr->radius;
        const double t_dist = ro_xxyy
            - t_pr->center.x * ro_2x
            - t_pr->center.y * ro_2y
            + t_pr->center.x * t_pr->center.x
            + t_pr->center.y * t_pr->center.y;
        if (t_dist <= t_rr)
            return false;
    }

    return true;
}

void ldplab::rtscpu::InitialStageBoundingSpheresHomogenousLight::
    advBatchCreationLight(size_t& li)
{
    ++li;
    m_batch_creation_particle_initialized = false;
}

void ldplab::rtscpu::InitialStageBoundingSpheresHomogenousLight::
    advBatchCreationParticle(size_t& pi)
{
    ++pi;
    // Reset values
    m_batch_creation_light_index = 0;
    m_batch_creation_particle_initialized = false;
}


inline void ldplab::rtscpu::InitialStageBoundingSpheresHomogenousLight::
    transformRayFromWorldToParticleSpace(Ray& ray, size_t pidx) const
{
    ray.origin = m_context->particle_transformations[pidx].w2p_rotation_scale *
        (ray.origin +
            m_context->particle_transformations[pidx].w2p_translation);
    ray.direction = glm::normalize(
        m_context->particle_transformations[pidx].w2p_rotation_scale *
        ray.direction);
}

bool ldplab::rtscpu::InitialStageBoundingSpheresHomogenousLight::createBatch(
    RayBuffer& initial_batch_buffer)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    LDPLAB_LOG_TRACE("RTSCPU context %i: Create initial batch rays for"\
        " batch buffer %i",
        m_context->uid,
        initial_batch_buffer.uid);

    initial_batch_buffer.active_rays = 0;
    initial_batch_buffer.world_space_rays = 0;
    if (m_batch_creation_particle_index >= m_projections_per_particle.size())
        return false;

    for (size_t i = 0; i < initial_batch_buffer.size; ++i)
        initial_batch_buffer.index_data[i] = -1;

    for (size_t& pi = m_batch_creation_particle_index;
        pi < m_projections_per_particle.size();
        advBatchCreationParticle(pi))
    {
        for (size_t& li = m_batch_creation_light_index;
            li < m_projections_per_particle[pi].size();
            advBatchCreationLight(li))
        {
            const Projection& projection =
                m_projections_per_particle[pi][li];
            const LightSource& light =
                m_context->light_sources[projection.light_index];

            if (!m_batch_creation_particle_initialized)
            {
                m_rasterization_x = projection.center.x;
                m_rasterization_y = projection.center.y;
                m_rasterization_up = true;
                m_rasterization_right = true;
                m_batch_creation_particle_initialized = true;
            }

            for (size_t& nr = initial_batch_buffer.active_rays;
                nr <= initial_batch_buffer.size;
                /* number of rays is increased during ray creation */)
            {
                // Check if the buffer is filled
                if (nr == initial_batch_buffer.size)
                {
                    // Set the correct number of active rays and return with
                    // true
                    //nr = initial_batch_buffer.size;
                    return true;
                }

                if (hasToCreateRay(projection, light))
                {
                    if (m_rasterization_x >= 0 && m_rasterization_y >= 0 &&
                        m_rasterization_x <= light.horizontal_size &&
                        m_rasterization_y <= light.vertical_size)
                    {
                        // Create ray
                        // Set ray origin
                        initial_batch_buffer.ray_data[nr].origin =
                            light.origin_corner
                            + m_rasterization_x * light.horizontal_direction
                            + m_rasterization_y * light.vertical_direction;
                        
                        // Set initial ray intensity
                        initial_batch_buffer.ray_data[nr].intensity =
                            ((LightDistributionHomogeneous*)
                                light.intensity_distribution.get())->intensity /
                            (m_context->parameters.number_rays_per_unit *
                            m_context->parameters.number_rays_per_unit);
                        
                        // Set initial ray direction
                        initial_batch_buffer.ray_data[nr].direction = 
                            light.orientation;

                        // Set minimum distance
                        initial_batch_buffer.min_bounding_volume_distance_data[nr] =
                            projection.depth;

                        // Set initial ray particle index
                        initial_batch_buffer.index_data[nr] = 
                            static_cast<int32_t>(pi);

                        // Transform ray to particle space
                        transformRayFromWorldToParticleSpace(
                            initial_batch_buffer.ray_data[nr], pi);

                        // Increase number of rays
                        ++nr;
                    }

                    if (m_rasterization_up)
                        m_rasterization_y += m_rasterization_step_size;
                    else
                        m_rasterization_y -= m_rasterization_step_size;
                }
                else
                {
                    // Update column
                    m_rasterization_up = !m_rasterization_up;
                    if (m_rasterization_right)
                        m_rasterization_x += m_rasterization_step_size;
                    else
                        m_rasterization_x -= m_rasterization_step_size;

                    if (abs(m_rasterization_x - projection.center.x) >
                        projection.radius)
                    {
                        if (m_rasterization_right)
                        {
                            // Move to rasterize left side of the projection
                            m_rasterization_x = projection.center.x;
                            m_rasterization_y =
                                projection.center.y - m_rasterization_step_size;
                            m_rasterization_up = false;
                            m_rasterization_right = false;
                        }
                        else
                        {
                            // Light source projection has no more rays
                            m_batch_creation_particle_initialized;
                            break;
                        }
                    }
                    else
                    {
                        if (m_rasterization_up)
                        {
                            while (!hasToCreateRay(projection, light))
                                m_rasterization_y += m_rasterization_step_size;
                        }
                        else
                        {
                            while (!hasToCreateRay(projection, light))
                                m_rasterization_y -= m_rasterization_step_size;
                        }
                    }
                }
            }
        }
    }

    return false;
}