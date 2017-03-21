# bump version here
set(vef_VERSION 1.0)

define_module(LIBRARY vef=${vef_VERSION}
  DEPENDS roarchive>=1.0 geo>=1.24 math>=1.2 utility>=1.11 dbglog>=1.4 jsoncpp>=1.1
  MAGIC # needed by tar
  Boost_FILESYSTEM
  )

set(vef_SOURCES
    vef.hpp vef.cpp
)

add_library(vef STATIC ${vef_SOURCES})
buildsys_library(vef)

target_link_libraries(vef ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(vef ${MODULE_DEFINITIONS})
