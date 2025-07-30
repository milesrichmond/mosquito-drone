target_sources(${PROJECT_NAME}
    PRIVATE
        bmp390.c
        bno055.c

    PUBLIC
    FILE_SET HEADERS
    BASE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}"
    FILES
        bmp390.h
        bno055.h
)
