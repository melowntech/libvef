if(TARGET roarchive)
  define_module(BINARY vef2vef
    DEPENDS vef roarchive service
    )

  # load tool
  set(vef2vef_SOURCES
    vef2vef.cpp
    )

  add_executable(vef2vef ${vef2vef_SOURCES})
  target_link_libraries(vef2vef ${MODULE_LIBRARIES})
  buildsys_target_compile_definitions(vef2vef PRIVATE ${MODULE_DEFINITIONS})
  buildsys_binary(vef2vef)
endif()

if(TARGET vef2vef AND vef2vef-install_component)
  set_property(TARGET vef2vef PROPERTY EXCLUDE_FROM_ALL NO)
  install(TARGETS vef2vef RUNTIME DESTINATION bin
    COMPONENT ${vef2vef-install_component})
endif()
