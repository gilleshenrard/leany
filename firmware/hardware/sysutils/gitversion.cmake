# SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
#
# SPDX-License-Identifier: MIT

# Retrieve the reduced hash code of the latest git commit
execute_process(
    COMMAND git log -1 --format=%h
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
    OUTPUT_VARIABLE kGitCommitHash
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Populate softVersion.c with the softVersion.c.in template
configure_file(
    softversion.c.in
    softversion.c
    @ONLY
)