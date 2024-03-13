get_filename_component(_TEST_NAME ${CMAKE_CURRENT_LIST_DIR} NAME)


function(create_test test_name)
    add_executable(${test_name}_test
        ${test_name}.cpp
        test_data.hpp
        test_data.cpp
        )
    target_link_libraries(${test_name}_test test_common libslic3r)
    set_property(TARGET ${test_name}_test PROPERTY FOLDER "tests")
    if (WIN32)
        prusaslicer_copy_dlls(${test_name}_test)
    endif()
    add_test(${test_name}_test ${test_name}_test ${CATCH_EXTRA_ARGS})
endfunction()

create_test(test_complete_objects)
create_test(test_clipper_utils)
create_test(test_dense_infill)
create_test(test_denserinfill)
create_test(test_extrusion_entity)
create_test(test_fan_mover)
create_test(test_fill)
create_test(test_flow)
create_test(test_gcodewriter)
create_test(test_geometry)
create_test(test_model)
create_test(test_print)
create_test(test_skirt_brim)
create_test(test_thin)
