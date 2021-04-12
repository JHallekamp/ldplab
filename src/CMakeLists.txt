#file(GLOB HEADER_LIST CONFIGURE_DEPENDS "${LDPLAB_SOURCE_DIR}/include/LDPLAB/*.hpp")
set(HEADER_ROOT
	"${LDPLAB_SOURCE_DIR}/include/ldplab.hpp"
)
	
set(HEADER_LDPLAB
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/BuildInfo.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/Constants.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/Geometry.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/LogCallback.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/Profiling.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/SimulationState.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/UID.hpp"
)
	
set(HEADER_LDPLAB_EXPERIMENTAL_SETUP
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/ExperimentalSetup/BoundingVolume.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/ExperimentalSetup/ExperimentalSetup.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/ExperimentalSetup/LightDirection.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/ExperimentalSetup/LightDistribution.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/ExperimentalSetup/LightPolarization.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/ExperimentalSetup/Lightsource.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/ExperimentalSetup/Particle.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/ExperimentalSetup/ParticleGeometry.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/ExperimentalSetup/ParticleMaterial.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/ExperimentalSetup/Utils/PropertyGenerator.hpp"
)
	
set(HEADER_LDPLAB_RAY_TRACING_STEP
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/EikonalSolver.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/IRayTracingStep.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/RayTracingStepCPUInfo.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/RayTracingStepFactory.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/RayTracingStepGPUOpenGLInfo.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/RayTracingStepOutput.hpp"
)

set(HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_CPU
	RayTracingStep/RayTracingStepCPU/BufferControl.hpp
    RayTracingStep/RayTracingStepCPU/Context.hpp
    RayTracingStep/RayTracingStepCPU/Data.hpp
    RayTracingStep/RayTracingStepCPU/Factory.hpp
    RayTracingStep/RayTracingStepCPU/InitialStage.hpp
    RayTracingStep/RayTracingStepCPU/InnerParticlePropagationStage.hpp
    RayTracingStep/RayTracingStepCPU/Pipeline.hpp
    RayTracingStep/RayTracingStepCPU/RayBoundingVolumeIntersectionTestStage.hpp
    RayTracingStep/RayTracingStepCPU/RayParticleInteractionStage.hpp
    RayTracingStep/RayTracingStepCPU/RayParticleIntersectionTestStage.hpp
    RayTracingStep/RayTracingStepCPU/RayTracingStep.hpp
)
	
set(HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_GPU_OPEN_GL
	RayTracingStep/RayTracingStepGPU/OpenGL/BufferControl.hpp
    RayTracingStep/RayTracingStepGPU/OpenGL/Constants.hpp
    RayTracingStep/RayTracingStepGPU/OpenGL/Context.hpp
    RayTracingStep/RayTracingStepGPU/OpenGL/Data.hpp
    RayTracingStep/RayTracingStepGPU/OpenGL/Factory.hpp
    RayTracingStep/RayTracingStepGPU/OpenGL/InitialStage.hpp
    RayTracingStep/RayTracingStepGPU/OpenGL/InnerParticlePropagationStage.hpp
    RayTracingStep/RayTracingStepGPU/OpenGL/OpenGLContext.hpp
    RayTracingStep/RayTracingStepGPU/OpenGL/Pipeline.hpp
    RayTracingStep/RayTracingStepGPU/OpenGL/RayBoundingVolumeIntersectionTestStage.hpp
    RayTracingStep/RayTracingStepGPU/OpenGL/RayParticleInteractionStage.hpp
    RayTracingStep/RayTracingStepGPU/OpenGL/RayParticleIntersectionTestStage.hpp
    RayTracingStep/RayTracingStepGPU/OpenGL/RayTracingStep.hpp
    RayTracingStep/RayTracingStepGPU/OpenGL/SharedShaders.hpp
)

set(HEADER_INTERN_UTILS 
    Utils/Assert.hpp
    Utils/ComputeHelper.hpp
    Utils/Log.hpp
    Utils/Profiler.hpp
    Utils/ThreadPool.hpp
)

set(SOURCE_ROOT
	BuildInfo.cpp
    LogCallback.cpp
    Profiling.cpp
)

set(SOURCE_EXPERIMENTAL_SETUP_UTILS
	ExperimentalSetup/Utils/PropertyGenerator.cpp
)

set(SOURCE_RAY_TRACING_STEP
	RayTracingStep/RayTracingStepFactory.cpp
)

set(SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_CPU
	RayTracingStep/RayTracingStepCPU/BufferControl.cpp
    RayTracingStep/RayTracingStepCPU/Factory.cpp
    RayTracingStep/RayTracingStepCPU/InitialStage.cpp
    RayTracingStep/RayTracingStepCPU/InnerParticlePropagationStage.cpp
    RayTracingStep/RayTracingStepCPU/Pipeline.cpp
    RayTracingStep/RayTracingStepCPU/RayBoundingVolumeIntersectionTestStage.cpp
    RayTracingStep/RayTracingStepCPU/RayParticleInteractionStage.cpp
    RayTracingStep/RayTracingStepCPU/RayParticleIntersectionTestStage.cpp
    RayTracingStep/RayTracingStepCPU/RayTracingStep.cpp
)

set(SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_GPU_OPEN_GL
	RayTracingStep/RayTracingStepGPU/OpenGL/BufferControl.cpp
    RayTracingStep/RayTracingStepGPU/OpenGL/Data.cpp
    RayTracingStep/RayTracingStepGPU/OpenGL/Factory.cpp
    RayTracingStep/RayTracingStepGPU/OpenGL/InitialStage.cpp
    RayTracingStep/RayTracingStepGPU/OpenGL/InnerParticlePropagationStage.cpp
    RayTracingStep/RayTracingStepGPU/OpenGL/OpenGLContext.cpp
    RayTracingStep/RayTracingStepGPU/OpenGL/Pipeline.cpp
    RayTracingStep/RayTracingStepGPU/OpenGL/RayBoundingVolumeIntersectionTestStage.cpp
    RayTracingStep/RayTracingStepGPU/OpenGL/RayParticleInteractionStage.cpp
    RayTracingStep/RayTracingStepGPU/OpenGL/RayParticleIntersectionTestStage.cpp
    RayTracingStep/RayTracingStepGPU/OpenGL/RayTracingStep.cpp
    RayTracingStep/RayTracingStepGPU/OpenGL/SharedShaders.cpp
)

set(SOURCE_UTILS
	Utils/ComputeHelper.cpp
    Utils/Log.cpp
    Utils/Profiler.cpp
    Utils/ThreadPool.cpp
)

add_library(ldplab STATIC 
    ${HEADER_ROOT}
    ${HEADER_LDPLAB}
    ${HEADER_LDPLAB_EXPERIMENTAL_SETUP}
    ${HEADER_LDPLAB_RAY_TRACING_STEP}
    ${HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_CPU}
    ${HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_GPU_OPEN_GL}
    ${HEADER_INTERN_UTILS}
    ${SOURCE_ROOT}
    ${SOURCE_EXPERIMENTAL_SETUP_UTILS}
    ${SOURCE_RAY_TRACING_STEP}
    ${SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_CPU}
    ${SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_GPU_OPEN_GL}
    ${SOURCE_UTILS}
)

source_group("Header Files\\LDPLAB" FILES ${HEADER_LDPLAB})
source_group("Header Files\\LDPLAB\\ExperimentalSetup" FILES ${HEADER_LDPLAB_EXPERIMENTAL_SETUP})
source_group("Header Files\\LDPLAB\\RayTracingStep" FILES ${HEADER_LDPLAB_RAY_TRACING_STEP})
source_group("Header Files\\Intern\\RayTracingStep\\RayTracingStepCPU" FILES ${HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_CPU})
source_group("Header Files\\Intern\\RayTracingStep\\RayTracingStepGPU\\OpenGL" FILES ${HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_GPU_OPEN_GL})
source_group("Header Files\\Intern\\Utils" FILES ${HEADER_INTERN_UTILS})

source_group("Source Files\\ExperimentalSetup\\Utils" FILES ${SOURCE_EXPERIMENTAL_SETUP_UTILS})
source_group("Source Files\\RayTracingStep" FILES ${SOURCE_RAY_TRACING_STEP})
source_group("Source Files\\RayTracingStep\\RayTracingStepCPU" FILES ${SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_CPU})
source_group("Source Files\\RayTracingStep\\RayTracingStepGPU\\OpenGL" FILES ${SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_GPU_OPEN_GL})
source_group("Source Files\\Utils" FILES ${SOURCE_UTILS})

target_include_directories(ldplab 
    PUBLIC 
        ../include
        ../external/glm
)

target_link_libraries(ldplab 
    PRIVATE
        glm
        Threads::Threads
)

if(LDPLAB_BUILD_OPTION_DISABLE_RTSGPU_OGL)
    target_compile_definitions(ldplab PRIVATE LDPLAB_BUILD_OPTION_DISABLE_RTSGPU_OGL)
endif()
target_compile_definitions(ldplab PRIVATE "$<$<CONFIG:Debug>:LDPLAB_BUILD_OPTION_ENABLE_DEBUG_LOGGING>")


target_compile_features(ldplab PUBLIC cxx_std_17)
