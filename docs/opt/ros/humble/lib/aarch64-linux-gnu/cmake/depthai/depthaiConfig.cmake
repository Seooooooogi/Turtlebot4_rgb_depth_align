# Get whether library was build as shared or not
set(DEPTHAI_SHARED_LIBS ON)

# Get whether library was build with Backward or not
set(DEPTHAI_ENABLE_BACKWARD ON)

set(DEPTHAI_ENABLE_CURL ON)

# Specify that this is config mode (Called by find_package)
set(CONFIG_MODE TRUE)

# Compute the installation prefix relative to this file.
set(_IMPORT_PREFIX "./dependencies")

# Add dependencies file
include("${CMAKE_CURRENT_LIST_DIR}/depthaiDependencies.cmake")

# Add the targets file
include("${CMAKE_CURRENT_LIST_DIR}/depthaiTargets.cmake")

# Cleanup
set(_IMPORT_PREFIX)
