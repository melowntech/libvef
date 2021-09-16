# bump version here
set(vef_VERSION 1.5)

set(vef_EXTRA_DEPENDS)
set(vef_EXTRA_SOURCES)

if(MODULE_roarchive_FOUND)
  message(STATUS "vef: compiling in roarchive support")
  list(APPEND vef_EXTRA_DEPENDS roarchive)
  list(APPEND vef_EXTRA_SOURCES reader.hpp reader.cpp)
else()
  message(STATUS "vef: compiling without roarchive support")
endif()

if(MODULE_roarchive_FOUND AND MODULE_geometry_FOUND)
  message(STATUS "vef: compiling in geometry support")
  list(APPEND vef_EXTRA_DEPENDS geometry)
  list(APPEND vef_EXTRA_SOURCES
    tiling.hpp tiling.cpp
    tiling-po.hpp tiling-po.cpp
    )
else()
  message(STATUS "vef: compiling without geometry support")
endif()

if(MODULE_roarchive_FOUND AND MODULE_vts-libs-tools-support_FOUND)
  message(STATUS "vef: compiling in VTS tools support")
  list(APPEND vef_EXTRA_DEPENDS vts-libs-tools-support>=2.15)
  list(APPEND vef_EXTRA_SOURCES tilecutter.hpp tilecutter.cpp)
else()
  message(STATUS "vef: compiling without VTS tools support")
endif()

define_module(LIBRARY vef=${vef_VERSION}
  DEPENDS ${vef_EXTRA_DEPENDS} geo>=1.35 math>=1.2 utility>=1.11 dbglog>=1.4
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

# various tools etc.
if(MODULE_service_FOUND)
  add_subdirectory(tools EXCLUDE_FROM_ALL)
endif()
