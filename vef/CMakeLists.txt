define_module(LIBRARY vef DEPENDS math utility dbglog jsoncpp
  Boost_FILESYSTEM
  )

set(vef_SOURCES
    vef.hpp vef.cpp
)

add_library(vef STATIC ${vef_SOURCES})
buildsys_library(vef)

target_link_libraries(vef ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(vef ${MODULE_DEFINITIONS})
