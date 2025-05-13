cmake_minimum_required (VERSION 2.6)
project (AminoTutorials)

# pkg-config setup
find_package(PkgConfig)

# Search for the pkg-config packages
pkg_search_module(SDL2 REQUIRED sdl2)
pkg_search_module(GL REQUIRED gl)
pkg_search_module(AMINO REQUIRED amino)
pkg_search_module(AMINO_GL REQUIRED amino-gl)

# Compile scene graph to C
add_custom_command(
  OUTPUT 7dofexistmedium-scene.c 7dofexistmedium-scene.c.h
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/7dofexistmedium.robray
  COMMAND aarxc "${CMAKE_SOURCE_DIR}/7dofexistmedium.robray" -o 7dofexistmedium-scene.c -n "7dofexistmedium"
  )



# Compile plugin (shared library)
add_library( scene MODULE 7dofexistmedium-scene.c )
target_include_directories( scene PUBLIC
  ${AMINO_INCLUDE_DIRS}
  )
target_link_libraries( scene
  ${AMINO_GL_LIBRARIES}
  ${AMINO_LIBRARIES}
  )

# Force the .so to appear in this subfolder, named libscene.so
set_target_properties(scene PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  OUTPUT_NAME "scene"
)