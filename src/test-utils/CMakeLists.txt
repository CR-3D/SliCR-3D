project(test-utils)

# convert stl to c++ code to inclusion in test
add_executable(stl_to_cpp stl_to_cpp.cpp)

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_libraries(stl_to_cpp libslic3r xcb)
else ()
    target_link_libraries(stl_to_cpp libslic3r)
endif ()

if (WIN32)
    prusaslicer_copy_dlls(stl_to_cpp)
endif()



# test your vendor config, helping to convert from prusaslicer.
add_executable(convert_config convert_config.cpp)

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_libraries(convert_config libslic3r xcb)
else ()
    target_link_libraries(convert_config libslic3r)
endif ()

if (WIN32)
    prusaslicer_copy_dlls(convert_config)
endif()
