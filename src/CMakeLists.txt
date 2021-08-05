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

set(HEADER_RAY_TRACING_STEP_RAY_TRACING_STEP_CPU
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CPU/Data.hpp"
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CPU/DefaultBoundingVolumeIntersectionFactories.hpp"
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CPU/DefaultGenericGeometryFactories.hpp"
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CPU/DefaultInitialStageFactories.hpp"
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CPU/DefaultInnerParticlePropagationFactories.hpp"
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CPU/DefaultParticleIntersectionFactories.hpp"
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CPU/DefaultSurfaceInteractionFactories.hpp"
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CPU/IBoundingVolumeIntersection.hpp"
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CPU/IGenericGeometry.hpp"
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CPU/IInitialStage.hpp"
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CPU/IInnerParticlePropagation.hpp"
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CPU/IParticleIntersection.hpp"
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CPU/IPipelineStage.hpp"
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CPU/ISurfaceInteraction.hpp"
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CPU/PipelineConfiguration.hpp"
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CPU/StageFactories.hpp"
)

set(HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_CPU
    RayTracingStep/CPU/Factory.hpp
    RayTracingStep/CPU/ImplBoundingVolumeIntersection.hpp
    RayTracingStep/CPU/ImplGenericGeometry.hpp
    RayTracingStep/CPU/ImplInitialStage.hpp
    RayTracingStep/CPU/ImplInnerParticlePropagation.hpp
    RayTracingStep/CPU/ImplParticleIntersection.hpp
    RayTracingStep/CPU/ImplSurfaceInteraction.hpp
    RayTracingStep/CPU/IntersectionTests.hpp
    RayTracingStep/CPU/MemoryControl.hpp
    RayTracingStep/CPU/Pipeline.hpp
    RayTracingStep/CPU/RayTracingStepCPU.hpp
)
set(SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_CPU
    RayTracingStep/CPU/DefaultBoundingVolumeIntersectionFactories.cpp
    RayTracingStep/CPU/DefaultGenericGeometryFactories.cpp
    RayTracingStep/CPU/DefaultInitialStageFactories.cpp
    RayTracingStep/CPU/DefaultInnerParticlePropagationFactories.cpp
    RayTracingStep/CPU/DefaultParticleIntersectionFactories.cpp
    RayTracingStep/CPU/DefaultSurfaceInteractionFactories.cpp
    RayTracingStep/CPU/Factory.cpp
    RayTracingStep/CPU/ImplBoundingVolumeIntersection.cpp
    RayTracingStep/CPU/ImplGenericGeometry.cpp
    RayTracingStep/CPU/ImplInitialStage.cpp
    RayTracingStep/CPU/ImplInnerParticlePropagation.cpp
    RayTracingStep/CPU/ImplParticleIntersection.cpp
    RayTracingStep/CPU/ImplSurfaceInteraction.cpp
    RayTracingStep/CPU/IntersectionTests.cpp
    RayTracingStep/CPU/IPipelineStage.cpp
    RayTracingStep/CPU/MemoryControl.cpp
    RayTracingStep/CPU/Pipeline.cpp
    RayTracingStep/CPU/RayTracingStepCPU.cpp
)

if (LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA)
    set(HEADER_RAY_TRACING_STEP_RAY_TRACING_STEP_CUDA
        "${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CUDA/Data.hpp"
	    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CUDA/DefaultBoundingVolumeIntersectionFactories.hpp"
	    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CUDA/DefaultGenericGeometryFactories.hpp"
	    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CUDA/DefaultGenericMaterialFactories.hpp"
	    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CUDA/DefaultInitialStageFactories.hpp"
	    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CUDA/DefaultInnerParticlePropagationFactories.hpp"
	    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CUDA/DefaultParticleIntersectionFactories.hpp"
	    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CUDA/DefaultSurfaceInteractionFactories.hpp"
	    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CUDA/DeviceResource.hpp"
	    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CUDA/Factories.hpp"
	    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CUDA/IBoundingVolumeIntersection.hpp"
	    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CUDA/IGenericGeometry.hpp"
	    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CUDA/IGenericMaterial.hpp"
	    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CUDA/IInitialStage.hpp"
	    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CUDA/IInnerParticlePropagation.hpp"
	    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CUDA/IParticleIntersection.hpp"
	    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CUDA/IPipelineStage.hpp"
	    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CUDA/ISurfaceInteraction.hpp"
	    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CUDA/PipelineConfiguration.hpp"
    )
    set(HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_CUDA
        RayTracingStep/CUDA/ImplBoundingVolumeIntersection.hpp
        RayTracingStep/CUDA/ImplGenericGeometry.hpp
        RayTracingStep/CUDA/ImplGenericMaterial.hpp
        RayTracingStep/CUDA/ImplInitialStage.hpp
        RayTracingStep/CUDA/ImplInnerParticlePropagation.hpp
        RayTracingStep/CUDA/ImplParticleIntersection.hpp
        RayTracingStep/CUDA/ImplSurfaceInteraction.hpp
        RayTracingStep/CUDA/IntersectionTests.hpp
        RayTracingStep/CUDA/IPipeline.hpp
        RayTracingStep/CUDA/PipelineDeviceBound.hpp
        RayTracingStep/CUDA/PipelineHostBound.hpp
        RayTracingStep/CUDA/RayTracingStepCUDA.hpp
        RayTracingStep/CUDA/StageBufferSetup.hpp
        RayTracingStep/CUDA/StageGatherOutput.hpp
        RayTracingStep/CUDA/StageRayBufferReduce.hpp
    )
    set(SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_CUDA
        RayTracingStep/CUDA/DefaultBoundingVolumeIntersectionFactories.cu
        RayTracingStep/CUDA/DefaultGenericGeometryFactories.cu
        RayTracingStep/CUDA/DefaultGenericMaterialFactories.cu
        RayTracingStep/CUDA/DefaultInitialStageFactories.cu
        RayTracingStep/CUDA/DefaultInnerParticlePropagationFactories.cu
        RayTracingStep/CUDA/DefaultParticleIntersectionFactories.cu
        RayTracingStep/CUDA/DefaultSurfaceInteractionFactories.cu
        RayTracingStep/CUDA/DeviceResource.cu
        RayTracingStep/CUDA/ImplBoundingVolumeIntersection.cu
        RayTracingStep/CUDA/ImplGenericGeometry.cu
        RayTracingStep/CUDA/ImplGenericMaterial.cu
        RayTracingStep/CUDA/ImplInitialStage.cu
        RayTracingStep/CUDA/ImplInnerParticlePropagation.cu
        RayTracingStep/CUDA/ImplParticleIntersection.cu
        RayTracingStep/CUDA/ImplSurfaceInteraction.cu
        RayTracingStep/CUDA/IntersectionTests.cu
        RayTracingStep/CUDA/IPipeline.cu
        RayTracingStep/CUDA/PipelineDeviceBound.cu
        RayTracingStep/CUDA/PipelineHostBound.cu
        RayTracingStep/CUDA/RayTracingStepCUDA.cu
        RayTracingStep/CUDA/StageBufferSetup.cu
        RayTracingStep/CUDA/StageGatherOutput.cu
        RayTracingStep/CUDA/StageRayBufferReduce.cu
    )
    set(HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_CUDA_DEPRECATED
		RayTracingStep/CUDA/deprecated/Context.hpp
		RayTracingStep/CUDA/deprecated/CudaResource.hpp
		RayTracingStep/CUDA/deprecated/Data.hpp
		RayTracingStep/CUDA/deprecated/Factory.hpp
		RayTracingStep/CUDA/deprecated/GenericBoundingVolume.hpp
		RayTracingStep/CUDA/deprecated/GenericParticleGeometry.hpp
		RayTracingStep/CUDA/deprecated/GenericParticleMaterial.hpp
		RayTracingStep/CUDA/deprecated/IntersectionTests.hpp
		RayTracingStep/CUDA/deprecated/Pipeline.hpp
		RayTracingStep/CUDA/deprecated/PipelineBoundingVolumeIntersection.hpp
		RayTracingStep/CUDA/deprecated/PipelineBufferSetup.hpp
		RayTracingStep/CUDA/deprecated/PipelineGatherOutput.hpp
		RayTracingStep/CUDA/deprecated/PipelineInitial.hpp
		RayTracingStep/CUDA/deprecated/PipelineInnerParticlePropagation.hpp
		RayTracingStep/CUDA/deprecated/PipelineParticleInteraction.hpp
		RayTracingStep/CUDA/deprecated/PipelineParticleIntersection.hpp
		RayTracingStep/CUDA/deprecated/PipelineRayBufferReduce.hpp
		RayTracingStep/CUDA/deprecated/RayTracingStep.hpp
    )
    set(SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_CUDA_DEPRECATED
		RayTracingStep/CUDA/deprecated/CudaResource.cu
		RayTracingStep/CUDA/deprecated/Data.cu
		RayTracingStep/CUDA/deprecated/Factory.cu
		RayTracingStep/CUDA/deprecated/GenericBoundingVolume.cu
		RayTracingStep/CUDA/deprecated/GenericParticleGeometry.cu
		RayTracingStep/CUDA/deprecated/GenericParticleMaterial.cu
		RayTracingStep/CUDA/deprecated/IntersectionTests.cu
		RayTracingStep/CUDA/deprecated/Pipeline.cu
		RayTracingStep/CUDA/deprecated/PipelineBoundingVolumeIntersection.cu
		RayTracingStep/CUDA/deprecated/PipelineBufferSetup.cu
		RayTracingStep/CUDA/deprecated/PipelineGatherOutput.cu
		RayTracingStep/CUDA/deprecated/PipelineInitial.cu
		RayTracingStep/CUDA/deprecated/PipelineInnerParticlePropagation.cu
		RayTracingStep/CUDA/deprecated/PipelineParticleInteraction.cu
		RayTracingStep/CUDA/deprecated/PipelineParticleIntersection.cu
		RayTracingStep/CUDA/deprecated/PipelineRayBufferReduce.cu
		RayTracingStep/CUDA/deprecated/RayTracingStep.cu
    )
else()
    set(HEADER_RAY_TRACING_STEP_RAY_TRACING_STEP_CUDA)
	set(HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_CUDA)
	set(SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_CUDA)
	set(HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_CUDA_DEPRECATED)
	set(SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_CUDA_DEPRECATED)
endif()

if(LDPLAB_BUILD_OPTION_ENABLE_RTSOGL)
    set(HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_OPENGL
    	RayTracingStep/OpenGL/BufferControl.hpp
        RayTracingStep/OpenGL/Constants.hpp
        RayTracingStep/OpenGL/Context.hpp
        RayTracingStep/OpenGL/Data.hpp
        RayTracingStep/OpenGL/Factory.hpp
        RayTracingStep/OpenGL/InitialStage.hpp
        RayTracingStep/OpenGL/InnerParticlePropagationStage.hpp
        RayTracingStep/OpenGL/OpenGLContext.hpp
        RayTracingStep/OpenGL/Pipeline.hpp
        RayTracingStep/OpenGL/RayBoundingVolumeIntersectionTestStage.hpp
        RayTracingStep/OpenGL/RayParticleInteractionStage.hpp
        RayTracingStep/OpenGL/RayParticleIntersectionTestStage.hpp
        RayTracingStep/OpenGL/RayTracingStep.hpp
        RayTracingStep/OpenGL/SharedShaders.hpp
    )
    set(SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_OPENGL
    	RayTracingStep/OpenGL/BufferControl.cpp
        RayTracingStep/OpenGL/Data.cpp
        RayTracingStep/OpenGL/Factory.cpp
        RayTracingStep/OpenGL/InitialStage.cpp
        RayTracingStep/OpenGL/InnerParticlePropagationStage.cpp
        RayTracingStep/OpenGL/OpenGLContext.cpp
        RayTracingStep/OpenGL/Pipeline.cpp
        RayTracingStep/OpenGL/RayBoundingVolumeIntersectionTestStage.cpp
        RayTracingStep/OpenGL/RayParticleInteractionStage.cpp
        RayTracingStep/OpenGL/RayParticleIntersectionTestStage.cpp
        RayTracingStep/OpenGL/RayTracingStep.cpp
        RayTracingStep/OpenGL/SharedShaders.cpp
    )
else()
    set(HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_OPENGL)
    set(SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_OPENGL)
endif()

set(HEADER_INTERN_UTILS
	Utils/Array.hpp
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
	${HEADER_RAY_TRACING_STEP_RAY_TRACING_STEP_CPU}
    ${HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_CPU}
    ${SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_CPU}
    ${HEADER_RAY_TRACING_STEP_RAY_TRACING_STEP_CUDA}
    ${HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_CUDA}
    ${SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_CUDA}
    ${HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_CUDA_DEPRECATED}
    ${SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_CUDA_DEPRECATED}
    ${HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_OPENGL}
    ${SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_OPENGL}
    ${HEADER_INTERN_UTILS}
    ${SOURCE_UTILS}
)

source_group("include" FILES ${HEADER_ROOT})
source_group("include\\LDPLAB" FILES ${HEADER_LDPLAB})
source_group("include\\LDPLAB\\ExperimentalSetup" FILES ${HEADER_LDPLAB_EXPERIMENTAL_SETUP})
source_group("include\\LDPLAB\\ExperimentalSetup\\Utils" FILES ${HEADER_LDPLAB_EXPERIMENTAL_SETUP_UTILS})
source_group("include\\LDPLAB\\RayTracingStep" FILES ${HEADER_LDPLAB_RAY_TRACING_STEP})
source_group("include\\LDPLAB\\RayTracingStep\\CPU" FILES ${HEADER_RAY_TRACING_STEP_RAY_TRACING_STEP_CPU})
source_group("src" FILES ${SOURCE_LDPLAB})
source_group("src\\ExperimentalSetup\\Utils" FILES ${SOURCE_EXPERIMENTAL_SETUP_UTILS})
source_group("src\\RayTracingStep" FILES ${SOURCE_RAY_TRACING_STEP})
source_group("src\\Utils" FILES ${HEADER_INTERN_UTILS})
source_group("src\\Utils" FILES ${SOURCE_UTILS})
source_group("src\\RayTracingStep\\CPU" FILES ${HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_CPU})
source_group("src\\RayTracingStep\\CPU" FILES ${SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_CPU})

if(LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA)
    source_group("include\\LDPLAB\\RayTracingStep\\CUDA\\" FILES ${HEADER_RAY_TRACING_STEP_RAY_TRACING_STEP_CUDA})
    source_group("src\\RayTracingStep\\CUDA" FILES ${HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_CUDA})
    source_group("src\\RayTracingStep\\CUDA" FILES ${SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_CUDA})
    source_group("src\\RayTracingStep\\CUDA\\deprecated" FILES ${HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_CUDA_DEPRECATED})
    source_group("src\\RayTracingStep\\CUDA\\deprecated" FILES ${SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_CUDA_DEPRECATED})
endif()
if (LDPLAB_BUILD_OPTION_ENABLE_RTSOGL)
    source_group("src\\RayTracingStep\\OpenGL" FILES ${HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_OPENGL})
    source_group("src\\RayTracingStep\\OpenGL" FILES ${SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_OPENGL})
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
