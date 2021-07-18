
set(support_ics
        st7789v2
        ili9341
        )
foreach(ic ${support_ics})
    set(target_name lvgl_port_${ic})
    add_library(${target_name})
    target_sources(${target_name} PRIVATE lvgl_port_${ic}.c)
    target_link_libraries(${target_name} PRIVATE stm32cube::hal lvgl)
    target_include_directories(${target_name} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
    add_library(lvgl::port::${ic} ALIAS ${target_name})
endforeach()


