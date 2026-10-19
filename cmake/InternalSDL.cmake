# SDL3, statically linked into libfpv for radio/gamepad input. Every SDL
# symbol stays hidden inside libfpv (version script / exported symbols
# list / no dllexport), so a game that loads its own SDL is unaffected.
# The examples link the same static library into their own executable,
# which reproduces "two SDL instances in one process" on purpose.
if(NOT TARGET SDL3-static)
  set(SDL_SHARED OFF CACHE BOOL "" FORCE)
  set(SDL_STATIC ON CACHE BOOL "" FORCE)
  set(SDL_TEST_LIBRARY OFF CACHE BOOL "" FORCE)
  set(SDL_TESTS OFF CACHE BOOL "" FORCE)
  set(SDL_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(SDL_INSTALL OFF CACHE BOOL "" FORCE)
  set(SDL_CAMERA OFF CACHE BOOL "" FORCE)
  set(SDL_DIALOG OFF CACHE BOOL "" FORCE)
  set(SDL_TRAY OFF CACHE BOOL "" FORCE)
  set(SDL_POWER OFF CACHE BOOL "" FORCE)
  set(SDL_HAPTIC OFF CACHE BOOL "" FORCE)
  set(SDL_GPU OFF CACHE BOOL "" FORCE)
  if(NOT EMSCRIPTEN)  # (WebAssembly is linked statically: no PIC)
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
  endif()
  add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/SDL ${CMAKE_BINARY_DIR}/third_party/SDL EXCLUDE_FROM_ALL)
  if(NOT EMSCRIPTEN)
    set_target_properties(SDL3-static PROPERTIES POSITION_INDEPENDENT_CODE ON)
  endif()
endif()
add_library(libfpv_sdl3 INTERFACE)
target_link_libraries(libfpv_sdl3 INTERFACE SDL3::SDL3-static)
