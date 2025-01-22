file(GLOB_RECURSE SOURCES_C *.c)
file(GLOB_RECURSE SOURCES_CPP *.cpp)

# require_component(esp_driver_ledc)

idf_component_register(
    SRCS ${SOURCES_C} ${SOURCES_CPP} 
    INCLUDE_DIRS "." 
    
    #REQUIRES 
    # REQUIRES esp_driver_ledc
    # PRIV_REQUIRES esp_driver_ledc
    )