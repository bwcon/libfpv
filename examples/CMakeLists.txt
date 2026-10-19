add_executable(fpv_minimal minimal.c)
target_link_libraries(fpv_minimal PRIVATE libfpv::fpv)

# The standalone "game": its own SDL3 (video + audio) + OpenGL 3.3.
if(EMSCRIPTEN)
  # libfpv has no SDL in browser builds; the game brings its own
  include(${PROJECT_SOURCE_DIR}/cmake/InternalSDL.cmake)
endif()
if(TARGET SDL3::SDL3-static)
  add_executable(fpv_standalone standalone/main.cpp)
  target_include_directories(fpv_standalone PRIVATE standalone ${PROJECT_SOURCE_DIR}/docs/renderers)
  target_link_libraries(fpv_standalone PRIVATE libfpv::fpv SDL3::SDL3-static)
  if(WIN32)
    target_link_options(fpv_standalone PRIVATE -static-libgcc -static-libstdc++ -static)
    set_target_properties(fpv_standalone PROPERTIES WIN32_EXECUTABLE OFF)
  endif()
  if(EMSCRIPTEN)
    # The web page: everything in <build>/web (index.html, index.js,
    # index.wasm), ready for any static web host (docs/WEB.md).
    set_target_properties(fpv_standalone PROPERTIES OUTPUT_NAME index SUFFIX .html
      RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/web)
    target_link_options(fpv_standalone PRIVATE
      -sMIN_WEBGL_VERSION=2 -sMAX_WEBGL_VERSION=2 -sALLOW_MEMORY_GROWTH=1 -sENVIRONMENT=web
      "SHELL:--shell-file ${CMAKE_CURRENT_SOURCE_DIR}/standalone/web/shell.html")
    set_property(TARGET fpv_standalone APPEND PROPERTY LINK_DEPENDS
      ${CMAKE_CURRENT_SOURCE_DIR}/standalone/web/shell.html ${PROJECT_SOURCE_DIR}/src/web/libfpv_web.js)
  endif()
endif()
