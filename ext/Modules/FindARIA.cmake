# - Try to find ARIA

find_path(ARIA_INCLUDE_DIR Aria.h
          HINTS "C:/Program Files/MobileRobots/ARIA/include" "/usr/include" "/usr/local/include")

find_path(ARIA_LIBRARY_DIR AriaVC12.lib
             HINTS "C:/Program Files/MobileRobots/ARIA/lib" "/usr/lib" "/usr/local/lib")

find_library(ARIA_LIBRARY NAMES AriaVC12 ArNetworkingVC12
             HINTS "C:/Program Files/MobileRobots/ARIA/lib" )

set(ARIA_LIBRARIES optimized AriaVC12 optimized ArNetworkingVC12 debug AriaDebugVC12 debug ArNetworkingDebugVC12) 
set(ARIA_INCLUDE_DIRS ${ARIA_INCLUDE_DIR} )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Aria  DEFAULT_MSG
                                  ARIA_LIBRARY ARIA_INCLUDE_DIR)

mark_as_advanced(ARIA_INCLUDE_DIR ARIA_LIBRARY )