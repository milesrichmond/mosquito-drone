target_sources(${PROJECT_NAME}
    PRIVATE
        flight_control.c

    PUBLIC
    FILE_SET HEADERS
    BASE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}"
    FILES
        flight_control.h
        parameters.h
)
