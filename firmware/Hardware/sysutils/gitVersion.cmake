# Retrieve the reduced hash code of the latest git commit
execute_process(
    COMMAND git log -1 --format=%h
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
    OUTPUT_VARIABLE GIT_COMMIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Populate softVersion.c with the softVersion.c.in template
configure_file(
    softVersion.c.in
    softVersion.c
    @ONLY
)