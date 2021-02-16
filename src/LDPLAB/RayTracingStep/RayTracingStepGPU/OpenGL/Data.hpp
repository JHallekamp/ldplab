#ifndef WWU_LDPLAB_RTSGPU_OGL_DATA_HPP
#define WWU_LDPLAB_RTSGPU_OGL_DATA_HPP

#include <cstdint>
#include <memory>
#include <vector>

#include "../../../ExperimentalSetup/BoundingVolume.hpp"
#include "../../../ExperimentalSetup/ParticleMaterial.hpp"
#include "../../../Geometry.hpp"
#include "../../../Utils/UID.hpp"

#include "OpenGLContext.hpp"

namespace ldplab
{
    namespace rtsgpu_ogl
    {
        /** @brief Holds the data of a batch of rays. */
        struct RayBuffer
        {
            RayBuffer(size_t depth, size_t size)
                :
                uid{ },
                depth{ depth },
                size{ size },
                ray_origin_data{ nullptr },
                ray_direction_data{ nullptr },
                ray_intensity_data{ nullptr },
                index_data{ nullptr },
                min_bounding_volume_distance_data{ nullptr },
                active_rays{ 0 },
                world_space_rays{ 0 },
                inner_particle_rays{ false }
            { }
            /** @brief Array containing the origin for each ray. */
            Vec4* ray_origin_data;
            /** @brief Array containing the direction for each ray. */
            Vec4* ray_direction_data;
            /** @brief Array containing intensity for each ray element. */
            double* ray_intensity_data;
            /** @brief Array containing size (particle) indices. */
            int32_t* index_data;
            /** @brief Array containing the min distance to bounding volumes */
            double* min_bounding_volume_distance_data;
            /** @brief Counter for the still active rays inside this. */
            size_t active_rays;
            /** @brief Counter for the active rays that are in world space. */
            size_t world_space_rays;
            /** @brief States whether buffer rays are inside a particle. */
            bool inner_particle_rays;
            /** @brief Branching depth (number of times rays split before). */
            const size_t depth;
            /** @brief Number of elements in ray_data and index_data arrays. */
            const size_t size;
            /** @brief SSBO corresponding to ray_orgin_data. */
            std::shared_ptr<ShaderStorageBuffer> ray_origin_ssbo;
            /** @brief SSBO corresponding to ray_direction_data. */
            std::shared_ptr<ShaderStorageBuffer> ray_direction_ssbo;
            /** @brief SSBO corresponding to ray_intensity_data. */
            std::shared_ptr<ShaderStorageBuffer> ray_intensity_ssbo;
            /** @brief SSBO corresponding to index_data. */
            std::shared_ptr<ShaderStorageBuffer> index_ssbo;
            /** @brief SSBO corresponding to min_bounding_volume_distance_data. */
            std::shared_ptr<ShaderStorageBuffer> min_bounding_volume_distance_ssbo;
            /** @brief Unique identifier of the buffer. */
            UID<RayBuffer> uid;
        };

        /**
         * @brief Buffer holding force and torque change of the particle
         */
        struct OutputBuffer
        {
            /** @brief Array containing size force changes per particle. */
            Vec4* force_data;
            /** @brief Array containing size torque changes per particle. */
            Vec4* torque_data;
            /** @brief Number of particles. */
            size_t size;
            /** @brief Array used to mirror the according SSBO. */
            Vec4* force_per_ray_data;
            /** @brief Array used to mirror the according SSBO. */
            Vec4* torque_per_ray_data;
            /** @brief SSBO containing the output force per ray. */
            std::shared_ptr<ShaderStorageBuffer> force_per_ray_ssbo;
            /** @brief SSBO containing the output torque per ray. */
            std::shared_ptr<ShaderStorageBuffer> torque_per_ray_ssbo;
        };

        /**
         * @brief Buffer holding intersection points and the corresponding 
         *        normal of the particle surface. 
         */
        struct IntersectionBuffer
        {
            /** @brief Array containing size intersection points. */
            Vec4* point_data;
            /** @brief Array containing size intersection normals. */
            Vec4* normal_data;
            /** @brief Array containing indices of intersected particles. */
            int32_t* particle_index_data;
            /** @brief Number of elements in point and normal arrays. */
            size_t size;
            /** @brief SSBO corresponding to point_data. */
            std::shared_ptr<ShaderStorageBuffer> point_ssbo;
            /** @brief SSBO corresponding to normal_data. */
            std::shared_ptr<ShaderStorageBuffer> normal_ssbo;
            /** @brief SSBO corresponding to particle_index_data. */
            std::shared_ptr<ShaderStorageBuffer> particle_index_ssbo;
        };

        /**
         * @brief Holds data that is used to transform world space rays into
         *        particle space or the other way around.
         */
        struct ParticleTransformationData
        {
            /** 
             * @brief Vector from particle space origin (in world space) to 
             *        world space origin.
             */
            std::vector<Vec4> w2p_translation_data;
            /**
             * @brief Matrix that transforms a world space vector (when the
             *        vector is applied to the matrix from the right-hand side) 
             *        by first rotating and then scaling it into particle space
             *        (safe for translation).
             */
            std::vector<Mat4> w2p_rotation_scale_data;
            /**
             * @brief Vector to the particle space (in world space)
             */
            std::vector<Vec4> p2w_translation_data;
            /**
             * @brief Matrix that transforms a particle space vector (when the
             *        vector is applied to the matrix from the right-hand side)
             *        by first scaling and then rotating it into world space
             *        (safe for translation).
             */
            std::vector<Mat4> p2w_scale_rotation_data;
            /** @brief Containing the SSBOs. */
            struct
            {
                std::shared_ptr<ShaderStorageBuffer> w2p_translation;
                std::shared_ptr<ShaderStorageBuffer> w2p_rotation_scale;
                std::shared_ptr<ShaderStorageBuffer> p2w_translation;
                std::shared_ptr<ShaderStorageBuffer> p2w_scale_rotation;
            } ssbo;
            /** @brief Uploads transformation data to the GPU buffers. */
            void uploadSSBO();
        };

        /**
         * @brief Structure models the geometries of a rod like particle. The 
         *        particle is cylindric shaped with a spherical cap and a 
         *        spherical indent at the bottom.
         * @detail The orientation of the cylinder is pointing to the 
         *         z-direction. The origin is of the coordinate system is set 
         *         in the middle point of bottom surface where the indentation 
         *         is.
         */
        struct RodParticle
        {
            double cylinder_radius;
            double cylinder_length;
            double sphere_radius;
            Vec4 origin_cap;
            Vec4 origin_indentation;
        };

        /**
         * @brief Interface for structure containing the particle data rehashed
         *        to be used during ray tracing.
         */
        struct IParticleData
        {
            virtual ~IParticleData() { }
            /** @brief The type of the particles. */
            enum class Type { rod_particles };
            /** @brief Returns the type of the particles. */
            virtual Type type() const = 0;
            /** @brief Uploads the data to the gpu. */
            virtual void uploadSSBO() = 0;
        };

        /** @brief Contains particle data. */
        struct RodParticleData : public IParticleData
        {
        private:
            std::shared_ptr<Context> m_context;
        public:
            RodParticleData(std::shared_ptr<Context> context)
                :
                m_context{ context }
            { }

            Type type() const override { return Type::rod_particles; }
            void uploadSSBO() override;
            /** @brief Contains the rod particles. */
            std::vector<RodParticle> particle_data;
            /** @brief Contains SSBOs for the rod particle data. */
            struct
            {
                /** @brief Corresponding to RodParticle::cylinder_radius. */
                std::shared_ptr<ShaderStorageBuffer> cylinder_radius;
                /** @brief Corresponding to RodParticle::cylinder_length. */
                std::shared_ptr<ShaderStorageBuffer> cylinder_length;
                /** @brief Corresponding to RodParticle::sphere_radius. */
                std::shared_ptr<ShaderStorageBuffer> sphere_radius;
                /** @brief Corresponding to RodParticle::origin_cap. */
                std::shared_ptr<ShaderStorageBuffer> origin_cap;
                /** @brief Corresponding to RodParticle::origin_indentation. */
                std::shared_ptr<ShaderStorageBuffer> origin_indentation;
            } ssbo;
        };

        /**
         * @brief Interface for structure containing particle material data
         *        to be used during ray tracing.
         */
        struct IParticleMaterialData
        {
            virtual ~IParticleMaterialData() { }
            /** @brief The type of the particle material. */
            enum class Type { linear_one_directional };
            /** @brief Returns the type of the particles. */
            virtual Type type() const = 0;
            /** @brief Uploads the data to the gpu. */
            virtual void uploadSSBO() = 0;
        };

        /** @brief Contains particle material data. */
        struct ParticleMaterialLinearOneDirectionalData : 
            public IParticleMaterialData
        {
        private:
            std::shared_ptr<Context> m_context;
        public:
            ParticleMaterialLinearOneDirectionalData(
                std::shared_ptr<Context> context)
                :
                m_context{ context }
            { }
            
            Type type() const override { return Type::linear_one_directional; }
            void uploadSSBO() override;
            std::vector<ParticleMaterialLinearOneDirectional> material_data;
            /**
             * @brief Contains SSBOs for linear one directional particle 
             *        material data.
             */
            struct
            {
                /** 
                 * @brief Contains the sum terms, which is equal to
                 *        $i - dot(g * V, O)$, where i is the index of 
                 *        refraction, g is the gradient, V is the gradient
                 *        direction and O is the origin of the index change.
                 */
                std::shared_ptr<ShaderStorageBuffer> 
                    index_of_refraction_sum_term;
                /** @brief Contains the direction times the gradient. */
                std::shared_ptr<ShaderStorageBuffer>
                    direction_times_gradient;
            } ssbo;
        };

        /** 
         * @brief Interface for structures containing data of bounding 
         *        volumes. 
         */
        struct IBoundingVolumeData
        {
            virtual ~IBoundingVolumeData() { }
            /** @brief The type of the bounding volume data. */
            enum class Type { spheres };
            /** @brief Returns the type of the bounding volumes. */
            virtual Type type() const = 0;
            /** @brief Uploads the data to the gpu. */
            virtual void uploadSSBO() = 0;
        };

        /** @brief Contains the data of the transformed bounding spheres. */
        struct BoundingSphereData : public IBoundingVolumeData
        {
            Type type() const override { return Type::spheres; }
            /** @brief Bounding sphere data in world space. */
            std::vector<BoundingVolumeSphere> sphere_data;
            /** @brief Uploads the data to the gpu. */
            virtual void uploadSSBO() override;
            /** 
             * @brief Contains shader storage buffer objects for 
             *        bounding sphere data.
             */
            struct
            {
                /** @brief Contains bounding sphere centers. */
                std::shared_ptr<ShaderStorageBuffer> center;
                /** @brief Contains bounding sphere radii. */
                std::shared_ptr<ShaderStorageBuffer> radius;
            } ssbo;
        };
    }
}

#endif
