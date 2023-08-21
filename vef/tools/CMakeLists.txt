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
  target_compile_definitions(vefupdate PRIVATE ${MODULE_DEFINITIONS})
  buildsys_binary(vefupdate)

  # load tool
  set(vefinfo_SOURCES
    vefinfo.cpp
    )

  add_executable(vefinfo ${vefinfo_SOURCES})
  target_link_libraries(vefinfo ${MODULE_LIBRARIES})
  target_compile_definitions(vefinfo PRIVATE ${MODULE_DEFINITIONS})
  buildsys_binary(vefinfo)

  if(vef-install_component)
    install(TARGETS vef2vef vefupdate vefinfo RUNTIME DESTINATION bin
      COMPONENT ${vef-install_component})
  endif()
endif()

