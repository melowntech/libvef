if(TARGET roarchive)
  define_module(BINARY vef2vef
    DEPENDS vef roarchive service geometry>=1.17
    )

  # load tool
  set(vef2vef_SOURCES
    vef2vef.cpp
    ogrpoly.hpp ogrpoly.cpp
    )

  add_executable(vef2vef ${vef2vef_SOURCES})
  target_link_libraries(vef2vef ${MODULE_LIBRARIES})
  target_compile_definitions(vef2vef PRIVATE ${MODULE_DEFINITIONS})
  buildsys_binary(vef2vef)

  # load tool
  set(vefupdate_SOURCES
    vefupdate.cpp
    )

  add_executable(vefupdate ${vefupdate_SOURCES})
  target_link_libraries(vefupdate ${MODULE_LIBRARIES})
  buildsys_target_compile_definitions(vefupdate PRIVATE ${MODULE_DEFINITIONS})
  buildsys_binary(vefupdate)

  if(vef-install_component)
    install(TARGETS vef2vef vefupdate RUNTIME DESTINATION bin
      COMPONENT ${vef-install_component})
  endif()
endif()

