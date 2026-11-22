include(FetchContent)

# FetchContent_Populate(<name>) is used below to grab Eigen's headers without
# running Eigen's own (large) CMake project.
if(POLICY CMP0169)
    cmake_policy(SET CMP0169 OLD)
endif()

# ---------------------------------------------------------------------------
# Eigen (header-only linear algebra)
# Uses an installed Eigen if one is found (vcpkg, apt, brew), otherwise downloads it.
# ---------------------------------------------------------------------------
find_package(Eigen3 3.4 QUIET NO_MODULE)

if(NOT TARGET Eigen3::Eigen)
    message(STATUS "Eigen3 not found locally, fetching 3.4.0")
    FetchContent_Declare(eigen
        URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
    FetchContent_GetProperties(eigen)
    if(NOT eigen_POPULATED)
        FetchContent_Populate(eigen)
    endif()
    add_library(Eigen3::Eigen INTERFACE IMPORTED GLOBAL)
    target_include_directories(Eigen3::Eigen SYSTEM INTERFACE ${eigen_SOURCE_DIR})
endif()

# ---------------------------------------------------------------------------
# Tracy profiler (optional): cmake -DSIM_ENABLE_TRACY=ON
# Run the Tracy GUI (same version) and connect while the sim runs.
# ---------------------------------------------------------------------------
if(SIM_ENABLE_TRACY)
    set(TRACY_ENABLE ON CACHE BOOL "" FORCE)
    set(TRACY_ON_DEMAND ON CACHE BOOL "" FORCE)
    FetchContent_Declare(tracy
        GIT_REPOSITORY https://github.com/wolfpld/tracy.git
        GIT_TAG v0.11.1
        GIT_SHALLOW TRUE
        GIT_PROGRESS TRUE)
    FetchContent_MakeAvailable(tracy)
endif()
