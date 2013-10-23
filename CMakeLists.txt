set(the_description "The UbiTrack Firewire Camera Component")
ut_add_component(firewirecamera DEPS utcore utdataflow utvision)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake" )
# ALSO NEEDS OPENCV FOR IMAGE CLASS !!!
FIND_PACKAGE(DC1394)
IF(DC1394_2_FOUND)
	SET(HAVE_DC1394 1)
	add_definitions(-DHAVE_DC1394)
	ut_component_include_directories(${UBITRACK_CORE_DEPS_INCLUDE_DIR} ${DC1394_INCLUDE_DIR})
	ut_glob_component_sources(HEADERS "src/Components/*.h" SOURCES "src/Components/*.cpp")
	ut_create_multi_component(${DC1394_LIBRARIES})
	ut_install_utql_patterns()
ENDIF(DC1394_2_FOUND)

