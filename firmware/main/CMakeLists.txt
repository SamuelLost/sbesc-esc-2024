set(APP_VERSION 1)

if (APP_VERSION EQUAL 1)
    idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS ".")
else()
    idf_component_register(SRCS "receiver.c"
                    INCLUDE_DIRS ".")
endif()