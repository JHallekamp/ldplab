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
set(SOURCE_LDPLAB
	BuildInfo.cpp
    LogCallback.cpp
    Profiling.cpp
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
)

set(HEADER_LDPLAB_EXPERIMENTAL_SETUP_UTILS
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/ExperimentalSetup/Utils/PropertyGenerator.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/ExperimentalSetup/Utils/ObjLoader.hpp"
)
set(SOURCE_EXPERIMENTAL_SETUP_UTILS
	ExperimentalSetup/Utils/PropertyGenerator.cpp
	ExperimentalSetup/Utils/ObjLoader.cpp
)

set(HEADER_LDPLAB_RAY_TRACING_STEP
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/AcceleratorStructureParameter.hpp"
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/EikonalSolverParameter.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/IRayTracingStep.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/RayTracingStepCPUInfo.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/RayTracingStepCUDAInfo.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/RayTracingStepFactory.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/RayTracingStepOpenGLInfo.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/RayTracingStepOutput.hpp"
)
set(SOURCE_RAY_TRACING_STEP
	RayTracingStep/RayTracingStepFactory.cpp
)

set(HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_CPU
	RayTracingStep/RayTracingStepCPU/AcceleratorStructures.hpp
	RayTracingStep/RayTracingStepCPU/BufferControl.hpp
    RayTracingStep/RayTracingStepCPU/Context.hpp
    RayTracingStep/RayTracingStepCPU/Data.hpp
    RayTracingStep/RayTracingStepCPU/Factory.hpp
    RayTracingStep/RayTracingStepCPU/GenericGeometry.hpp
    RayTracingStep/RayTracingStepCPU/InitialStage.hpp
    RayTracingStep/RayTracingStepCPU/InnerParticlePropagationStage.hpp
	RayTracingStep/RayTracingStepCPU/IntersectionTests.hpp
    RayTracingStep/RayTracingStepCPU/Pipeline.hpp
    RayTracingStep/RayTracingStepCPU/RayBoundingVolumeIntersectionTestStage.hpp
    RayTracingStep/RayTracingStepCPU/RayParticleInteractionStage.hpp
    RayTracingStep/RayTracingStepCPU/RayParticleIntersectionTestStage.hpp
    RayTracingStep/RayTracingStepCPU/RayTracingStep.hpp
)
set(SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_CPU
	RayTracingStep/RayTracingStepCPU/AcceleratorStructures.cpp
	RayTracingStep/RayTracingStepCPU/BufferControl.cpp
    RayTracingStep/RayTracingStepCPU/Factory.cpp
    RayTracingStep/RayTracingStepCPU/GenericGeometry.cpp
    RayTracingStep/RayTracingStepCPU/InitialStage.cpp
    RayTracingStep/RayTracingStepCPU/InnerParticlePropagationStage.cpp
	RayTracingStep/RayTracingStepCPU/IntersectionTests.cpp
    RayTracingStep/RayTracingStepCPU/Pipeline.cpp
    RayTracingStep/RayTracingStepCPU/RayBoundingVolumeIntersectionTestStage.cpp
    RayTracingStep/RayTracingStepCPU/RayParticleInteractionStage.cpp
    RayTracingStep/RayTracingStepCPU/RayParticleIntersectionTestStage.cpp
    RayTracingStep/RayTracingStepCPU/RayTracingStep.cpp
)

if (LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA)
    set(HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_CUDA
		RayTracingStep/RayTracingStepCUDA/Context.hpp
		RayTracingStep/RayTracingStepCUDA/CudaResource.hpp
		RayTracingStep/RayTracingStepCUDA/Data.hpp
		RayTracingStep/RayTracingStepCUDA/Factory.hpp
		RayTracingStep/RayTracingStepCUDA/GenericBoundingVolume.hpp
		RayTracingStep/RayTracingStepCUDA/GenericParticleGeometry.hpp
		RayTracingStep/RayTracingStepCUDA/GenericParticleMaterial.hpp
		RayTracingStep/RayTracingStepCUDA/IntersectionTests.hpp
		RayTracingStep/RayTracingStepCUDA/Pipeline.hpp
		RayTracingStep/RayTracingStepCUDA/PipelineBoundingVolumeIntersection.hpp
		RayTracingStep/RayTracingStepCUDA/PipelineBufferSetup.hpp
		RayTracingStep/RayTracingStepCUDA/PipelineGatherOutput.hpp
		RayTracingStep/RayTracingStepCUDA/PipelineInitial.hpp
		RayTracingStep/RayTracingStepCUDA/PipelineInnerParticlePropagation.hpp
		RayTracingStep/RayTracingStepCUDA/PipelineParticleInteraction.hpp
		RayTracingStep/RayTracingStepCUDA/PipelineParticleIntersection.hpp
		RayTracingStep/RayTracingStepCUDA/PipelineRayBufferReduce.hpp
		RayTracingStep/RayTracingStepCUDA/RayTracingStep.hpp
    )
    set(SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_CUDA
		RayTracingStep/RayTracingStepCUDA/CudaResource.cu
		RayTracingStep/RayTracingStepCUDA/Data.cu
		RayTracingStep/RayTracingStepCUDA/Factory.cu
		RayTracingStep/RayTracingStepCUDA/GenericBoundingVolume.cu
		RayTracingStep/RayTracingStepCUDA/GenericParticleGeometry.cu
		RayTracingStep/RayTracingStepCUDA/GenericParticleMaterial.cu
		RayTracingStep/RayTracingStepCUDA/IntersectionTests.cu
		RayTracingStep/RayTracingStepCUDA/Pipeline.cu
		RayTracingStep/RayTracingStepCUDA/PipelineBoundingVolumeIntersection.cu
		RayTracingStep/RayTracingStepCUDA/PipelineBufferSetup.cu
		RayTracingStep/RayTracingStepCUDA/PipelineGatherOutput.cu
		RayTracingStep/RayTracingStepCUDA/PipelineInitial.cu
		RayTracingStep/RayTracingStepCUDA/PipelineInnerParticlePropagation.cu
		RayTracingStep/RayTracingStepCUDA/PipelineParticleInteraction.cu
		RayTracingStep/RayTracingStepCUDA/PipelineParticleIntersection.cu
		RayTracingStep/RayTracingStepCUDA/PipelineRayBufferReduce.cu
		RayTracingStep/RayTracingStepCUDA/RayTracingStep.cu
    )
else()
	set(HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_CUDA)
	set(SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_CUDA)
endif()

if(LDPLAB_BUILD_OPTION_ENABLE_RTSOGL)
    set(HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_OPENGL
    	RayTracingStep/RayTracingStepOpenGL/BufferControl.hpp
        RayTracingStep/RayTracingStepOpenGL/Constants.hpp
        RayTracingStep/RayTracingStepOpenGL/Context.hpp
        RayTracingStep/RayTracingStepOpenGL/Data.hpp
        RayTracingStep/RayTracingStepOpenGL/Factory.hpp
        RayTracingStep/RayTracingStepOpenGL/InitialStage.hpp
        RayTracingStep/RayTracingStepOpenGL/InnerParticlePropagationStage.hpp
        RayTracingStep/RayTracingStepOpenGL/OpenGLContext.hpp
        RayTracingStep/RayTracingStepOpenGL/Pipeline.hpp
        RayTracingStep/RayTracingStepOpenGL/RayBoundingVolumeIntersectionTestStage.hpp
        RayTracingStep/RayTracingStepOpenGL/RayParticleInteractionStage.hpp
        RayTracingStep/RayTracingStepOpenGL/RayParticleIntersectionTestStage.hpp
        RayTracingStep/RayTracingStepOpenGL/RayTracingStep.hpp
        RayTracingStep/RayTracingStepOpenGL/SharedShaders.hpp
    )
    set(SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_OPENGL
    	RayTracingStep/RayTracingStepOpenGL/BufferControl.cpp
        RayTracingStep/RayTracingStepOpenGL/Data.cpp
        RayTracingStep/RayTracingStepOpenGL/Factory.cpp
        RayTracingStep/RayTracingStepOpenGL/InitialStage.cpp
        RayTracingStep/RayTracingStepOpenGL/InnerParticlePropagationStage.cpp
        RayTracingStep/RayTracingStepOpenGL/OpenGLContext.cpp
        RayTracingStep/RayTracingStepOpenGL/Pipeline.cpp
        RayTracingStep/RayTracingStepOpenGL/RayBoundingVolumeIntersectionTestStage.cpp
        RayTracingStep/RayTracingStepOpenGL/RayParticleInteractionStage.cpp
        RayTracingStep/RayTracingStepOpenGL/RayParticleIntersectionTestStage.cpp
        RayTracingStep/RayTracingStepOpenGL/RayTracingStep.cpp
        RayTracingStep/RayTracingStepOpenGL/SharedShaders.cpp
    )
else()
    set(HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_OPENGL)
    set(SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_OPENGL)
endif()

set(HEADER_INTERN_UTILS
    Utils/Assert.hpp
    Utils/ComputeHelper.hpp
    Utils/Log.hpp
    Utils/Profiler.hpp
    Utils/ThreadPool.hpp
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
	${SOURCE_LDPLAB}
    ${HEADER_LDPLAB_EXPERIMENTAL_SETUP}
    ${HEADER_LDPLAB_EXPERIMENTAL_SETUP_UTILS}
    ${SOURCE_EXPERIMENTAL_SETUP_UTILS}
    ${HEADER_LDPLAB_RAY_TRACING_STEP}
    ${SOURCE_RAY_TRACING_STEP}
    ${HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_CPU}
    ${SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_CPU}
    ${HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_CUDA}
    ${SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_CUDA}
    ${HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_OPENGL}
    ${SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_OPENGL}
    ${HEADER_INTERN_UTILS}
    ${SOURCE_UTILS}
)

source_group("Header Files\\LDPLAB" FILES ${HEADER_LDPLAB})
source_group("Header Files\\LDPLAB\\ExperimentalSetup" FILES ${HEADER_LDPLAB_EXPERIMENTAL_SETUP})
source_group("Header Files\\LDPLAB\\ExperimentalSetup\\Utils" FILES ${HEADER_LDPLAB_EXPERIMENTAL_SETUP_UTILS})
source_group("Source Files\\ExperimentalSetup\\Utils" FILES ${SOURCE_EXPERIMENTAL_SETUP_UTILS})
source_group("Header Files\\LDPLAB\\RayTracingStep" FILES ${HEADER_LDPLAB_RAY_TRACING_STEP})
source_group("Source Files\\RayTracingStep" FILES ${SOURCE_RAY_TRACING_STEP})
source_group("Header Files\\Intern\\Utils" FILES ${HEADER_INTERN_UTILS})
source_group("Source Files\\Utils" FILES ${SOURCE_UTILS})

source_group("Header Files\\Intern\\RayTracingStep\\RayTracingStepCPU" FILES ${HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_CPU})
source_group("Source Files\\RayTracingStep\\RayTracingStepCPU" FILES ${SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_CPU})
if(LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA)
    source_group("Header Files\\Intern\\RayTracingStep\\RayTracingStepCUDA" FILES ${HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_CUDA})
    source_group("Source Files\\RayTracingStep\\RayTracingStepCUDA" FILES ${SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_CUDA})
endif()
if (LDPLAB_BUILD_OPTION_ENABLE_RTSOGL)
    source_group("Header Files\\Intern\\RayTracingStep\\RayTracingStepOpenGL" FILES ${HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_OPENGL})
    source_group("Source Files\\RayTracingStep\\RayTracingStepOpenGL" FILES ${SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_OPENGL})
endif()

target_compile_features(ldplab PUBLIC cxx_std_14)

target_include_directories(ldplab
    PUBLIC
        ../include
        ../external/glm
		../external/tinyobjloader
)

target_link_libraries(ldplab
    PRIVATE
        glm
        Threads::Threads
)

if(LDPLAB_BUILD_OPTION_ENABLE_RTSOGL)
    target_compile_definitions(ldplab PRIVATE LDPLAB_BUILD_OPTION_ENABLE_RTSOGL)
endif()

if(LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA)
    target_compile_definitions(ldplab PRIVATE LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA)
	set_target_properties(ldplab PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
	set_target_properties(ldplab PROPERTIES CUDA_RESOLVE_DEVICE_SYMBOLS  ON)
endif()

if(LDPLAB_BUILD_OPTION_ENABLE_PROFILING)
    target_compile_definitions(ldplab PRIVATE LDPLAB_BUILD_OPTION_ENABLE_PROFILING)
endif()

if(LDPLAB_BUILD_OPTION_DISABLE_LOGGING)
    target_compile_definitions(ldplab PRIVATE LDPLAB_BUILD_OPTION_DISABLE_LOGGING)
endif()

if (TINYOBJLOADER_USE_DOUBLE)
	target_compile_definitions(ldplab PRIVATE TINYOBJLOADER_USE_DOUBLE)
endif()

target_compile_definitions(ldplab PRIVATE "$<$<CONFIG:Debug>:LDPLAB_BUILD_OPTION_ENABLE_DEBUG_LOGGING>")
