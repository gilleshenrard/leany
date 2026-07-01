# SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
#
# SPDX-License-Identifier: MIT

# Retrieve the reduced hash code of the latest git commit
execute_process(
    COMMAND git log -1 --format=%h
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
    OUTPUT_VARIABLE COMMIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Populate softVersion.h with the softVersion.h.in template
message("Populating softversion.h with CMake values")
configure_file(
    softversion.h.in
    ${PROJECT_SOURCE_DIR}/utils/softversion.h
    @ONLY
)

# Populate README.md with the README.md.in template
message("Populating README.md with CMake values")
configure_file(
    README.md.in
    ${PROJECT_SOURCE_DIR}/README.md
    @ONLY
)

# Populate Doxyfile with the Doxyfile.in template
message("Populating Doxyfile with CMake values")
configure_file(
    Doxyfile.in
    ${PROJECT_SOURCE_DIR}/Doxyfile
    @ONLY
)
