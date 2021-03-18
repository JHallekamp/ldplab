#ifndef WWU_LDPLAB_RTSGPU_OGL_OPEN_GL_CONTEXT_HPP
#define WWU_LDPLAB_RTSGPU_OGL_OPEN_GL_CONTEXT_HPP

#include <memory>
#include <mutex>
#include <string>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

namespace ldplab
{
    namespace rtsgpu_ogl
    {
        // Prototype
        struct Context;

        /** @brief Encapsulates an OpenGL shader program. */
        class ComputeShader
        {
            friend class OpenGLContext;
        public:
            /** @brief Destructs the shader program. */
            ~ComputeShader();
            /**
             * @brief Provides the name of the shader.
             * @returns The name as a null terminated C string.
             */
            const char* name() const;
            /**
             * @brief Provides the location of the specified shader uniform.
             * @param[in] name The name of the uniform buffer.
             * @returns If successful it returns the location of the uniform
             *          inside the shader program, otherwise -1.
             */
            GLint getUniformLocation(const std::string& name) const;
            /** @brief Uses shader in the current OpenGL rendering state. */
            void use() const;
            /**
             * @brief Executes the compute shader.
             * @param[in] work_group_size_x Workgroup size in dimension x.
             * @param[in] work_group_size_y Workgroup size in dimension y.
             * @param[in] work_group_size_z Workgroup size in dimension z.
             * @note You have to call use before execute.
             */
            void execute(size_t work_group_size_x,
                size_t work_group_size_y = 1,
                size_t work_group_size_z = 1);
        private:
            ComputeShader();
        private:
            std::shared_ptr<Context> m_context;
            /** @brief OpenGL ID of the shader program. */
            GLuint m_glid;
            /** @brief The name of the shader program. */
            std::string m_name;
        };

        /** @brief Encapsulates a shader storage buffer. */
        class ShaderStorageBuffer
        {
            friend class OpenGLContext;
        public:
            ~ShaderStorageBuffer();
            /** @brief Size of the buffer in bytes. */
            size_t size() const { return m_size; }
            /** 
             * @brief Binds the shader storage buffer to a given binding index.
             * @param[in] binding_index The SSBO index to which the buffer is 
             *                          bound.
             */
            void bindToIndex(GLuint binding_index) const;
            /** 
             * @brief Uploads data to the SSBO.
             * @param[in] offset Offset for SSBO in bytes.
             * @param[in] size Size of the copied range in bytes.
             * @param[in] data Pointer to the source data that is uploaded to
             *                 the shader buffer. This needs to be size bytes
             *                 in size.
             * @note This will bind the buffer.
             */
            void upload(size_t offset, size_t size, const void* data);
            /**
             * @brief Uploads data to the SSBO.
             * @param[in] data Pointer to the source data that is uploaded to 
             *                 the shader buffer. This needs to be of the same
             *                 size as the SSBO.
             * @note This will bind the buffer.
             */
            void upload(const void* data);
            /**
             * @brief Downloads data from the SSBO.
             * @param[in] offset Offset for the SSBO in bytes.
             * @param[in] size Size of the copied range in bytes.
             * @param[in,out] target Pointer to target where the data is copied
             *                       to. The target must be size bytes in size.
             * @note This will bind the buffer.
             */
            void download(size_t offset, size_t size, void* target);
            /**
             * @brief Downloads the complete data from the SSBO.
             * @param[in,out] target Pointer to target where the data is copied
             *                       to. The target must be of the same size as
             *                       the SSBO.
             * @note This will bind the buffer.
             */
            void download(void* target);
        private:
            ShaderStorageBuffer();
        private:
            /** @brief The shader storage buffer object (SSBO) OpenGL id. */
            GLuint m_glid;
            /** @brief Size of the buffer in bytes. */
            size_t m_size;
            /** @brief Usage pattern of the buffer. */
            GLenum m_usage;
        };

        /**
         * @brief Provides an internal interface to OpenGL and manages the 
         *        necessary data and context.
         */
        class OpenGLContext
        {
        public:
            OpenGLContext(std::shared_ptr<Context> context);
            ~OpenGLContext();
            /** 
             * @brief Initializes glew, opengl and context data. 
             * @returns true if the initialization was successful.
             */
            bool init();
            /** @brief Shuts down the opengl. */
            void shutdown();
            /**
             * @brief Creates a compute shader program.
             * @param[in] shader_name The name of the glsl shader.
             * @param[in] glsl_code The glsl shader code of the compute shader.
             * @returns Shared pointer to the shader or nullptr if the shader
             *          was not created successfully.
             */
            std::shared_ptr<ComputeShader> createComputeShader(
                const std::string& shader_name,
                const std::string& glsl_code) const;
            /**
             * @brief Creates a compute shader program from file.
             * @param[in] shader_name The name of the glsl shader.
             * @param[in] path The file path to the shader code.
             * @returns Shared pointer to the shader or nullptr if the shader
             *          has not been created successfully.
             */
            std::shared_ptr<ComputeShader> createComputeShaderFromFile(
                const std::string& shader_name,
                const std::string& path);
            /**
             * @brief Creates a shader storage buffer object (SSBO).
             * @param[in] buffer_size Size of the buffer in bytes.
             * @param[in] buffer_usage Specifies OpenGL usage pattern.
             * @returns Shared pointer to the shader storage buffer or nullptr
             *          if the buffer was not created successfully.
             */
            std::shared_ptr<ShaderStorageBuffer> createShaderStorageBuffer(
                size_t buffer_size,
                GLenum buffer_usage = GL_DYNAMIC_READ);
            /**
             * @brief Provides a mutex that can be used to sequenzialise GPU
             *        access.
             * @returns Reference to said mutex.
             */
            inline std::mutex& getGPUMutex() { return m_gpu_mutex; }
            /** @brief Binds the opengl context for the current thread. */
            void bindGlContext();
            /** @brief Unbinds the opengl context from the current thread. */
            void unbindGlContext();
        private:
            std::shared_ptr<Context> m_context;
            std::mutex m_gpu_mutex;
            GLFWwindow* m_gl_offscreen_context;
        };
    }
}

#endif