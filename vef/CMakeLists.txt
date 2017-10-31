# bump version here
set(vef_VERSION 1.2)

set(vef_EXTRA_DEPENDS)
set(vef_EXTRA_SOURCES)

if(MODULE_roarchive_FOUND)
  message(STATUS "vef: compiling in roarchive support")
  list(APPEND vef_EXTRA_DEPENDS roarchive)
  list(APPEND vef_EXTRA_SOURCES reader.hpp reader.cpp)
else()
  message(STATUS "vef: compiling without roarchive support")
endif()

define_module(LIBRARY vef=${vef_VERSION}
  DEPENDS ${vef_EXTRA_DEPENDS} geo>=1.24 math>=1.2 utility>=1.11 dbglog>=1.4
  jsoncpp>=1.1
  Boost_FILESYSTEM Boost_IOSTREAMS
  )

set(vef_SOURCES
  vef.hpp vef.cpp
  ${vef_EXTRA_SOURCES}
)

add_library(vef STATIC ${vef_SOURCES})
buildsys_library(vef)

target_link_libraries(vef ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(vef ${MODULE_DEFINITIONS})
