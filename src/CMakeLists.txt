add_subdirectory(control)
add_subdirectory(devices)
add_subdirectory(mcu)
add_subdirectory(util)

target_sources(${PROJECT_NAME}
    PRIVATE
        main.c
        sysmem.c
)
