# Try to find matio library
# Once done, this will define the following
# MATIO_FOUND - system has matio
# MATIO_INCLUDE_DIRS - matio library include directories
# MATIO_LIBRARIES - libraries needed to use matio

find_path(MATIO_INCLUDE_DIR matio.h)

find_library(MATIO_LIBRARY NAMES matio libmatio)

set(MATIO_INCLUDE_DIRS ${MATIO_INCLUDE_DIR})

set(MATIO_LIBRARIES ${MATIO_LIBRARY})
