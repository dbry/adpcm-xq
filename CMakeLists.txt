cmake_minimum_required(VERSION 3.16)

# set macOS build options
set(CMAKE_OSX_ARCHITECTURES "arm64;x86_64" CACHE STRING "" FORCE)

# set the project name
project(adpcm-xq LANGUAGES C)

include(CheckLibraryExists)

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_C_COMPILER_ID MATCHES "Clang")
# lots of warnings and all warnings as errors
  add_compile_options(-Wall)
# add_compile_options(-Wextra)
endif()

# define as library
add_library (adpcm-lib STATIC adpcm-lib.c adpcm-dns.c)

# build executable
add_executable (adpcm-xq-exe adpcm-xq.c)
set_property(TARGET adpcm-xq-exe PROPERTY OUTPUT_NAME adpcm-xq)

check_library_exists(m pow "" HAVE_LIBM)
if(HAVE_LIBM)
  target_link_libraries(adpcm-lib m)
  target_link_libraries(adpcm-xq-exe adpcm-lib)
  target_link_libraries(adpcm-xq-exe m)
endif()

# define location for header files
target_include_directories(adpcm-xq-exe PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} )
