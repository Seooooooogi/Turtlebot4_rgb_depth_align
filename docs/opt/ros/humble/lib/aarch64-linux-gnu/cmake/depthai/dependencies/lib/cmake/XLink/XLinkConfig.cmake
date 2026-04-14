# Get whether library was build as shared or not
set(XLINK_SHARED_LIBS OFF)

# Get library options
set(XLINK_ENABLE_LIBUSB ON)
set(XLINK_LIBUSB_LOCAL )
set(XLINK_LIBUSB_SYSTEM OFF)

# Specify that this is config mode (Called by find_package)
set(CONFIG_MODE TRUE)

# Compute the installation prefix relative to this file.
set(_IMPORT_PREFIX "./dependencies")

# Add dependencies file
include("${CMAKE_CURRENT_LIST_DIR}/XLinkDependencies.cmake")

# Add the targets file
include("${CMAKE_CURRENT_LIST_DIR}/XLinkTargets.cmake")

# Cleanup
set(_IMPORT_PREFIX)
