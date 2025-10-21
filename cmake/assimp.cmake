# Try the modern config first
find_package(assimp QUIET CONFIG)

# Also accept the FindAssimp.cmake module if it defines assimp::assimp
if(NOT TARGET assimp::assimp)
  find_package(assimp QUIET)
endif()

# If still no imported target, make our own (Linux/Windows)
if((NOT TARGET assimp::assimp) OR (NOT assimp_FOUND))

  # Detect arch (used by some Windows layouts)
  if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(ASSIMP_ARCHITECTURE "64")
  elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
    set(ASSIMP_ARCHITECTURE "32")
  endif()

  if(MSVC)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(PC_ASSIMP REQUIRED assimp)

    # Toolset hinting (covers vc120/vc140/vc141/vc142 etc.)
    if(MSVC_TOOLSET_VERSION)
      set(ASSIMP_MSVC_VERSION "vc${MSVC_TOOLSET_VERSION}")
    elseif(MSVC14)
      set(ASSIMP_MSVC_VERSION "vc140")
    elseif(MSVC12)
      set(ASSIMP_MSVC_VERSION "vc120")
    endif()

    # Headers
    find_path(assimp_INCLUDE_DIR
      NAMES assimp/scene.h
      HINTS ${PC_ASSIMP_INCLUDEDIR} ${PC_ASSIMP_INCLUDE_DIRS}
      REQUIRED)

    # Lib dir and library
    find_path(assimp_LIBRARY_DIR
      NAMES assimp-${ASSIMP_MSVC_VERSION}-mt.lib assimp-vc140-mt.lib assimp-vc141-mt.lib assimp-vc120-mt.lib
      HINTS ${PC_ASSIMP_LIBDIR} "${PC_ASSIMP_PREFIX}/Lib"
      REQUIRED)

    find_library(assimp_LIBRARY
      NAMES assimp-${ASSIMP_MSVC_VERSION}-mt.lib assimp-vc140-mt.lib assimp-vc141-mt.lib assimp-vc120-mt.lib assimp
      PATHS ${assimp_LIBRARY_DIR}
      REQUIRED)
  else()
    # ---- Linux / macOS
    find_path(assimp_INCLUDE_DIR
      NAMES assimp/scene.h assimp/version.h
      HINTS /usr/include /usr/local/include
      PATH_SUFFIXES assimp ..
      REQUIRED)

    # Prefer shared, but accept static if that’s all there is
    find_library(assimp_LIBRARY
      NAMES assimp
      HINTS /usr/lib /usr/lib64 /usr/local/lib
      REQUIRED)
  endif()

  include(FindPackageHandleStandardArgs)
  if(MSVC)
    find_package_handle_standard_args(assimp REQUIRED_VARS assimp_INCLUDE_DIR assimp_LIBRARY assimp_LIBRARY_DIR)
  else()
    find_package_handle_standard_args(assimp REQUIRED_VARS assimp_INCLUDE_DIR assimp_LIBRARY)
  endif()

  # Create imported target
  add_library(assimp::assimp UNKNOWN IMPORTED)
  set_target_properties(assimp::assimp PROPERTIES
    IMPORTED_LOCATION "${assimp_LIBRARY}")

  # Let consumers do #include <assimp/xxx>
  # If assimp_INCLUDE_DIR points to ".../assimp", export its parent
  get_filename_component(_ASSIMP_INC_PARENT "${assimp_INCLUDE_DIR}" DIRECTORY)
  if(EXISTS "${assimp_INCLUDE_DIR}/scene.h")
    # include dir is already the parent
    set(_ASSIMP_INCLUDE_ROOT "${assimp_INCLUDE_DIR}")
  else()
    set(_ASSIMP_INCLUDE_ROOT "${_ASSIMP_INC_PARENT}")
  endif()
  set_target_properties(assimp::assimp PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${_ASSIMP_INCLUDE_ROOT}")

  # Mark found + legacy vars (useful for catkin_package DEPENDS)
  set(assimp_FOUND TRUE)
  set(assimp_INCLUDE_DIRS "${_ASSIMP_INCLUDE_ROOT}")
  set(assimp_LIBRARIES "${assimp_LIBRARY}")
  if(DEFINED assimp_LIBRARY_DIR)
    set(assimp_LIBRARY_DIRS "${assimp_LIBRARY_DIR}")
  endif()
endif()

# ---- Workaround for buggy exports like "/usr/lib/include"
# Read whatever includes the imported target exposes and fix if invalid.
get_target_property(_ASSIMP_IFACE_INCS assimp::assimp INTERFACE_INCLUDE_DIRECTORIES)
if(_ASSIMP_IFACE_INCS)
  set(_ASSIMP_FIXED_INCS "")
  foreach(_inc IN LISTS _ASSIMP_IFACE_INCS)
    if(EXISTS "${_inc}")
      list(APPEND _ASSIMP_FIXED_INCS "${_inc}")
    endif()
  endforeach()

  # If none of the exported paths exist, compute the right one and set it.
  if(_ASSIMP_FIXED_INCS STREQUAL "")
    # Re-discover include dir robustly
    find_path(_ASSIMP_HDR_ROOT
      NAMES assimp/scene.h
      HINTS /usr/include /usr/local/include ${assimp_INCLUDE_DIRS}
      PATH_SUFFIXES assimp ..)

    if(_ASSIMP_HDR_ROOT)
      # As above, ensure root contains 'assimp/'
      if(EXISTS "${_ASSIMP_HDR_ROOT}/assimp/scene.h")
        set(_ASSIMP_EXPORT_ROOT "${_ASSIMP_HDR_ROOT}")
      else()
        get_filename_component(_ASSIMP_EXPORT_ROOT "${_ASSIMP_HDR_ROOT}" DIRECTORY)
      endif()
      set_property(TARGET assimp::assimp PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${_ASSIMP_EXPORT_ROOT}")
      message(STATUS "assimp: fixed INTERFACE_INCLUDE_DIRECTORIES → ${_ASSIMP_EXPORT_ROOT}")
    else()
      # If we can’t fix it, clear to avoid non-existent paths tripping CMake
      set_property(TARGET assimp::assimp PROPERTY INTERFACE_INCLUDE_DIRECTORIES "")
      message(WARNING "assimp: exported include dirs were invalid and could not be fixed automatically")
    endif()
  endif()
endif()

# Some broken exports omit IMPORTED_CONFIGURATIONS
get_target_property(_ASSIMP_IMPCFG assimp::assimp IMPORTED_CONFIGURATIONS)
if(NOT _ASSIMP_IMPCFG)
  set_property(TARGET assimp::assimp APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
endif()
