target_sources(${PROJECT_NAME}
    PRIVATE
        i2c.c
        pwm.c
        timer.c
        types.c

    PUBLIC
    FILE_SET HEADERS
    BASE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}"
    FILES
        i2c.h
        pwm.h
        timer.h
        types.h
)
