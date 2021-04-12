#ifndef WWU_LDPLAB_RTSGPU_OGL_DATA_HPP
#define WWU_LDPLAB_RTSGPU_OGL_DATA_HPP

#include <cstdint>
#include <memory>
#include <vector>

#include <LDPLAB/ExperimentalSetup/BoundingVolume.hpp>
#include <LDPLAB/ExperimentalSetup/ParticleMaterial.hpp>
#include <LDPLAB/Geometry.hpp>
#include <LDPLAB/UID.hpp>

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
                ray_properties_data{ nullptr },
                particle_index_data{ nullptr },
                active_rays{ 0 },
                world_space_rays{ 0 },
                inner_particle_rays{ false }
            { }
            /** @brief Contains GPU applicable ray data. */
            struct RayProperties
            {
                Vec3 origin;
                double intensity;
                Vec3 direction;
                double min_bounding_volume_distance;
            };
            /** @brief Array containing the ray properties per ray. */
            RayProperties* ray_properties_data;
            /** @brief Array containing the particle indices per ray. */
            int32_t* particle_index_data;
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
            /** @brief Unique identifier of the buffer. */
            UID<RayBuffer> uid;
            /** @brief Containing the shader buffer storage objects. */
            struct
            {
                /** @brief Corresponding to ray_properties_data. */
                std::shared_ptr<ShaderStorageBuffer> ray_properties;
                /** @brief Corresponding to index_data. */
                std::shared_ptr<ShaderStorageBuffer> particle_index;
            } ssbo;
        };

        /**
         * @brief Buffer holding force and torque change of the particle
         */
        struct OutputBuffer
        {
            /** @brief Contains GPU applicable scattered output element. */
            struct OutputData
            {
                Vec3 force;
                double UNUSED_00;
                Vec3 torque;
                double UNUSED_01;
            };
            /** @brief Number of particles. */
            size_t size;
            /** @brief Array used to mirror the according SSBO. */
            OutputData* output_per_ray_data;
            /** @brief Array used to mirror the according SSBO. */
            OutputData* output_gathered_data;
            /** @brief Containint the shader buffer storage objects. */
            struct
            {
                /** @brief Corresponding to output_per_ray. */
                std::shared_ptr<ShaderStorageBuffer> output_per_ray;
                /** @brief Corresponding to output_gathered. */
                std::shared_ptr<ShaderStorageBuffer> output_gathered;
                /** @brief Used for temporary storage in gather shader. */
                std::shared_ptr<ShaderStorageBuffer> gather_temp;
            } ssbo;
        };

        /**
         * @brief Buffer holding intersection points and the corresponding 
         *        normal of the particle surface. 
         */
        struct IntersectionBuffer
        {
            /** @brief Contains GPU applicable intersection data. */
            struct IntersectionProperties
            {
                Vec3 point;
                double UNUSED_00;
                Vec3 normal;
                double UNUSED_01;
            };
            /** @brief Array containing intersection properties. */
            IntersectionProperties* intersection_properties_data;
            /** @brief Array containing indices of intersected particles. */
            int32_t* particle_index_data;
            /** @brief Number of elements in point and normal arrays. */
            size_t size;
            /** @brief Containint the shader buffer storage objects. */
            struct
            {
                /** @brief Corresponding to intersection_properties_data. */
                std::shared_ptr<ShaderStorageBuffer> intersection_properties;
                /** @brief Corresponding to particle_index_data. */
                std::shared_ptr<ShaderStorageBuffer> particle_index;
            } ssbo;
        };

        /**
         * @brief Holds data that is used to transform world space rays into
         *        particle space or the other way around.
         */
        struct ParticleTransformationData
        {
            struct Transformation
            {
                Mat4 rotation_scale;
                Mat4 translation;
            };
            // Raw transformations
            std::vector<Mat3> w2p_rotation_scale;
            std::vector<Vec3> w2p_translation;
            std::vector<Mat3> p2w_scale_rotation;
            std::vector<Vec3> p2w_translation;
            /**
             * @brief Matrix that transforms a world space vector (when the
             *        vector is applied to the matrix from the right-hand side) 
             *        by first rotating and then scaling it into particle 
             *        space.
             */
            std::vector<Transformation> w2p_data;
            /**
             * @brief Matrix that transforms a particle space vector (when the
             *        vector is applied to the matrix from the right-hand side)
             *        by first scaling and then rotating it into world space.
             */
            std::vector<Transformation> p2w_data;
            /** @brief Containing the SSBOs. */
            struct
            {
                /** @brief Corresponds to w2p_data. */
                std::shared_ptr<ShaderStorageBuffer> w2p;
                /** @brief Corresponds to p2w_data. */
                std::shared_ptr<ShaderStorageBuffer> p2w;
            } ssbo;
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
            /** @brief Used to upload particle data to GPU. */
            virtual void uploadSSBO() = 0;
        };

        /** @brief Contains particle data. */
        struct RodParticleData : public IParticleData
        {
        private:
            Context& m_context;
        public:
            RodParticleData(Context& context)
                :
                m_context{ context }
            { }
            Type type() const override { return Type::rod_particles; }
            void uploadSSBO() override;
            /** @brief Contains GPU applicable rod particle data. */
            struct RodParticleProperties
            {
                double cap_origin_z;
                double indentation_origin_z;
                double cylinder_radius;
                double cylinder_length;
                double sphere_radius;
            };
            /** @brief Contains the rod particles. */
            std::vector<RodParticleProperties> rod_particles_data;
            /** @brief Contains SSBOs for the rod particle data. */
            struct
            {
                /** @brief Corresponding to particle_data. */
                std::shared_ptr<ShaderStorageBuffer> rod_particles;
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
            /** @brief Used to upload particle data to GPU. */
            virtual void uploadSSBO() = 0;
        };

        /** @brief Contains particle material data. */
        struct ParticleMaterialLinearOneDirectionalData : 
            public IParticleMaterialData
        {
        private:
            Context& m_context;
        public:
            ParticleMaterialLinearOneDirectionalData(
                Context& context)
                :
                m_context{ context }
            { }
            Type type() const override { return Type::linear_one_directional; }
            void uploadSSBO() override;
            /** @brief Contains GPU applicable material data. */
            struct LinearOneDirectionalMaterialProperties
            {
                Vec3 direction_times_gradient;
                double index_of_refraction_sum_term;
            };
            std::vector<LinearOneDirectionalMaterialProperties> material_data;
            /**
             * @brief Contains SSBOs for linear one directional particle 
             *        material data.
             */
            struct
            {
                /** @brief Corresponding to material_data. */
                std::shared_ptr<ShaderStorageBuffer> material;
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
            /** @brief Used to upload particle data to GPU. */
            virtual void uploadSSBO() = 0;
        };

        /** @brief Contains the data of the transformed bounding spheres. */
        struct BoundingSphereData : public IBoundingVolumeData
        {
            Type type() const override { return Type::spheres; }
            void uploadSSBO() override;
            /** @brief Contains GPU applicable bounding sphere data. */
            struct BoundingSphereProperties
            {
                Vec3 center;
                double radius;
            };
            /** @brief Bounding sphere data in world space. */
            std::vector<BoundingSphereProperties> sphere_properties_data;
            /** 
             * @brief Contains shader storage buffer objects for 
             *        bounding sphere data.
             */
            struct
            {
                /** @brief Corresponding to sphere_properties_data. */
                std::shared_ptr<ShaderStorageBuffer> sphere_properties;
            } ssbo;
        };
    }
}

#endif
