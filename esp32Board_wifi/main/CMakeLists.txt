set(src_dirs        .
                    src
                    ai)

set(include_dirs    include
                    ai
                    .)
idf_component_register(SRC_DIRS ${src_dirs}
                       INCLUDE_DIRS ${include_dirs}
                       PRIV_REQUIRES esp_new_jpeg)
#idf_component_register(SRCS "app_main.cpp")
