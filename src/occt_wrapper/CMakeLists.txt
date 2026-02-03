cmake_minimum_required(VERSION 3.13)
project(OCCTWrapper)

# --- 1. Add OCCT 7.6.1 first ---
add_cmake_project(OCCT
    URL https://github.com/Open-Cascade-SAS/OCCT/archive/refs/tags/V7_6_1.zip
    URL_HASH SHA256=b7cf65430d6f099adc9df1749473235de7941120b5b5dd356067d12d0909b1d3
    CMAKE_ARGS
        -DINSTALL_DIR_LAYOUT=Unix
        -DBUILD_LIBRARY_TYPE=Static
        -DUSE_TK=OFF
        -DUSE_TBB=OFF
        -DUSE_FREETYPE=OFF
        -DUSE_FFMPEG=OFF
        -DUSE_VTK=OFF
)

# --- 2. Point to the exact install folder ---
set(OpenCASCADE_DIR "${CMAKE_BINARY_DIR}/_deps/OCCT/lib/cmake/opencascade")

# Optional: ignore system-installed OCCT
set(CMAKE_IGNORE_PATH "D:/a/SliCR-3D_Internal/SliCR-3D_Internal/deps/build/destdir/usr/local/lib/cmake/opencascade")

# --- 3. Find OCCT 7.6.1 ---
find_package(OpenCASCADE 7.6.1 REQUIRED)

# --- 4. Your wrapper library ---
if (APPLE)
    add_library(OCCTWrapper STATIC OCCTWrapper.cpp)
else ()
    add_library(OCCTWrapper MODULE OCCTWrapper.cpp)
endif ()

set_target_properties(OCCTWrapper
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/src"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/src"
    PREFIX ""
)

include(GenerateExportHeader)
generate_export_header(OCCTWrapper)
if (OpenCASCADE_VERSION VERSION_LESS 7.8.0)
    message(STATUS "OpenCASCADE_VERSION version < 7.8.0")
    set(OCCT_LIBS
        TKXDESTEP
        TKSTEP
        TKSTEP209
        TKSTEPAttr
        TKSTEPBase
        TKXCAF
        TKXSBase
        TKVCAF
        TKCAF
        TKLCAF
        TKCDF
        TKV3d
        TKService
        TKMesh
        TKBO
        TKPrim
        TKHLR
        TKShHealing
        TKTopAlgo
        TKGeomAlgo
        TKBRep
        TKGeomBase
        TKG3d
        TKG2d
        TKMath
        TKernel
    )
else()
    message(STATUS "OpenCASCADE_VERSION version >= 7.8.0")
    set(OCCT_LIBS
            TKXCAF
            TKXSBase
            TKVCAF
            TKCAF
            TKLCAF
            TKCDF
            TKV3d
            TKService
            TKMesh
            TKBO
            TKPrim
            TKHLR
            TKShHealing
            TKTopAlgo
            TKGeomAlgo
            TKBRep
            TKGeomBase
            TKG3d
            TKG2d
            TKMath
            TKernel
            TKDESTEP
            TKXSDRAWSTEP
    )
endif()

message(STATUS "OpenCASCADE_VERSION: '${OpenCASCADE_VERSION}'")

message(STATUS "OCCT_LIBS: '${OCCT_LIBS}'")
slic3r_remap_configs("${OCCT_LIBS}" RelWithDebInfo Release)

target_include_directories(OCCTWrapper PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_include_directories(OCCTWrapper PUBLIC ${OpenCASCADE_INCLUDE_DIR})
target_link_libraries(OCCTWrapper ${OCCT_LIBS})
target_link_libraries(OCCTWrapper libslic3r admesh)


include(GNUInstallDirs)

install(TARGETS OCCTWrapper DESTINATION "${CMAKE_INSTALL_LIBDIR}/superslicer")

