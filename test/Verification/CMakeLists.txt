set(HEADER_VERIFICATION
	ITest.hpp
)

set(HEADER_VERIFICATION_UNIT_TESTS
	UnitTests/IntersectionCommon.hpp
	UnitTests/RayAABBOverlapTest.hpp
	UnitTests/RaySphereIntersectionTest.hpp
	UnitTests/RayTriangleIntersectionTest.hpp
	UnitTests/SegmentAABBOverlapTest.hpp
	UnitTests/SegmentTriangleIntersectionTest.hpp
	UnitTests/TriangleAABBOverlapTest.hpp
)

set(SOURCE_VERIFICATION
	Main.cpp
)

set(SOURCE_VERIFICATION_UNIT_TESTS
	UnitTests/RayAABBOverlapTest.cpp
	UnitTests/RaySphereIntersectionTest.cpp
	UnitTests/RayTriangleIntersectionTest.cpp
	UnitTests/SegmentAABBOverlapTest.cpp
	UnitTests/SegmentTriangleIntersectionTest.cpp
	UnitTests/TriangleAABBOverlapTest.cpp
)

add_executable(ldplab_verification
	${HEADER_VERIFICATION}
	${HEADER_VERIFICATION_UNIT_TESTS}
	${SOURCE_VERIFICATION}
	${SOURCE_VERIFICATION_UNIT_TESTS}
)

source_group("Header Files\\" FILES ${HEADER_VERIFICATION})
source_group("Header Files\\UnitTests" FILES ${HEADER_VERIFICATION_UNIT_TESTS})
source_group("Source Files\\" FILES ${SOURCE_VERIFICATION})
source_group("Source Files\\UnitTests" FILES ${SOURCE_VERIFICATION_UNIT_TESTS})

target_include_directories(ldplab_verification
	PUBLIC
		"${LDPLAB_SOURCE_DIR}/include"
		"${LDPLAB_SOURCE_DIR}/src"
)

target_link_libraries(ldplab_verification
	PRIVATE
		ldplab
)

