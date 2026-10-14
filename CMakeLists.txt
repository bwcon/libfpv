cmake_minimum_required(VERSION 3.24)
project(libfpv VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(LIBFPV_WITH_SDL "Read radios and gamepads with an embedded SDL3" ON)
option(LIBFPV_WITH_OVERLAY "Build the Dear ImGui overlay" ON)
option(LIBFPV_BUILD_TESTS "Build the test suite" ON)
option(LIBFPV_BUILD_EXAMPLES "Build the examples" ON)

include(cmake/SimITLCore.cmake)
include(cmake/EmbedFiles.cmake)

# --- embedded assets -----------------------------------------------------------
file(GLOB_RECURSE LIBFPV_PRESET_FILES CONFIGURE_DEPENDS
  ${PROJECT_SOURCE_DIR}/assets/presets/*.json
  ${PROJECT_SOURCE_DIR}/assets/presets/*.diff.txt)
list(SORT LIBFPV_PRESET_FILES)
set(LIBFPV_GEN_DIR ${CMAKE_BINARY_DIR}/generated)
libfpv_embed_files(${LIBFPV_GEN_DIR}/embedded_presets.cpp kEmbeddedPresets
  ${PROJECT_SOURCE_DIR}/assets/presets ${LIBFPV_PRESET_FILES})
file(GLOB LIBFPV_FONT_FILES CONFIGURE_DEPENDS ${PROJECT_SOURCE_DIR}/assets/fonts/*.mcm)
libfpv_embed_files(${LIBFPV_GEN_DIR}/embedded_fonts.cpp kEmbeddedFonts
  ${PROJECT_SOURCE_DIR}/assets/fonts ${LIBFPV_FONT_FILES})

# --- libfpv --------------------------------------------------------------------
set(LIBFPV_SOURCES
  src/api.cpp
  src/api_present.cpp
  src/frame.cpp
  src/config/bf_config.cpp
  src/config/preset.cpp
  src/input/input.cpp
  src/input/sdl_backend.cpp
  src/sim/flight.cpp
  src/sim/runner.cpp
  src/util/fs.cpp
  src/util/log.cpp
  src/util/platform.cpp
  src/world/collision.cpp
  src/osd/osd.cpp
  src/audio/synth.cpp
  ${LIBFPV_GEN_DIR}/embedded_presets.cpp
  ${LIBFPV_GEN_DIR}/embedded_fonts.cpp
)

add_library(fpv SHARED ${LIBFPV_SOURCES})
add_library(libfpv::fpv ALIAS fpv)
set_target_properties(fpv PROPERTIES
  OUTPUT_NAME fpv
  C_VISIBILITY_PRESET hidden
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON
  POSITION_INDEPENDENT_CODE ON
  VERSION ${PROJECT_VERSION}
  SOVERSION 0)
target_compile_definitions(fpv PRIVATE LIBFPV_BUILD FMT_HEADER_ONLY=1)
target_include_directories(fpv
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/third_party
    ${SIMITL_DIR}/src
    ${SIMITL_DIR}/lib
    ${FMT_DIR}/include)

# Betaflight registers its parameter groups in a linker section that nothing
# references by name: link every object of the core.
target_link_libraries(fpv PRIVATE "$<LINK_LIBRARY:WHOLE_ARCHIVE,simitl_core>" websockets)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  target_link_options(fpv PRIVATE
    -T${SIMITL_LINKER_SCRIPT}
    -Wl,--version-script=${PROJECT_SOURCE_DIR}/cmake/libfpv.map
    -Wl,--exclude-libs,ALL
    -Wl,-z,nodelete
    -Wl,-Bsymbolic
    -Wl,--no-undefined)
  target_link_libraries(fpv PRIVATE pthread dl m)
elseif(APPLE)
  target_link_options(fpv PRIVATE
    -Wl,-exported_symbols_list,${PROJECT_SOURCE_DIR}/cmake/libfpv.exp)
elseif(WIN32)
  # libfpv.def lets MSVC hosts make an import library: lib /def:libfpv.def /machine:x64
  target_link_options(fpv PRIVATE -T${SIMITL_LINKER_SCRIPT} -static-libgcc -static-libstdc++ -static
    -Wl,--output-def,${CMAKE_BINARY_DIR}/libfpv.def)
  target_link_libraries(fpv PRIVATE ws2_32 wsock32 winmm iphlpapi userenv)
endif()

if(LIBFPV_WITH_OVERLAY)
  set(IMGUI_DIR ${PROJECT_SOURCE_DIR}/third_party/imgui)
  target_sources(fpv PRIVATE
    ${IMGUI_DIR}/imgui.cpp ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp ${IMGUI_DIR}/imgui_widgets.cpp
    src/overlay/overlay.cpp src/overlay/bf_cache.cpp src/overlay/radio_wizard.cpp)
  target_include_directories(fpv PRIVATE ${IMGUI_DIR})
else()
  target_sources(fpv PRIVATE src/overlay/overlay_stub.cpp)
endif()

if(LIBFPV_WITH_SDL)
  include(cmake/InternalSDL.cmake)
  target_compile_definitions(fpv PRIVATE LIBFPV_WITH_SDL=1)
  target_link_libraries(fpv PRIVATE libfpv_sdl3)
endif()

# --- tests and examples ----------------------------------------------------------
if(LIBFPV_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
if(LIBFPV_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()
