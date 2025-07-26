# ProjectConfig.cmake - Common configuration for all packages in this project

# Find the project root directory by looking for this config file's parent directory
get_filename_component(PROJECT_ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)

# Make PROJECT_ROOT_DIR available as a compile definition
add_definitions(-DPROJECT_ROOT_DIR="${PROJECT_ROOT_DIR}")

# Optionally set other common variables
set(PROJECT_ASSETS_DIR "${PROJECT_ROOT_DIR}/assets")
set(PROJECT_COMMON_DIR "${PROJECT_ROOT_DIR}/common")

# You can add other common settings here
message(STATUS "Project root directory: ${PROJECT_ROOT_DIR}")