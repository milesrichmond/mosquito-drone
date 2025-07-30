target_sources(${PROJECT_NAME}
    PRIVATE
        debug.c
        quaternion.c

    PUBLIC
    FILE_SET HEADERS
    BASE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}"
    FILES
        debug.h
        error_types.h
        quaternion.h
)
