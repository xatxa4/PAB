# Writes OUTPUT with PAB_GIT_VERSION set to `git describe` of SOURCE_DIR.
# Called at build time by the pab_version target in CMakeLists.txt.
#
# "-dirty" means uncommitted changes were built in, which is exactly the case
# where a version string is otherwise misleading. Outside a git checkout the
# version is "unknown" rather than a build failure.

execute_process(
    COMMAND git describe --always --dirty --abbrev=10
    WORKING_DIRECTORY ${SOURCE_DIR}
    OUTPUT_VARIABLE PAB_GIT_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    RESULT_VARIABLE result
)
if (NOT result EQUAL 0 OR PAB_GIT_VERSION STREQUAL "")
    set(PAB_GIT_VERSION "unknown")
endif()

set(content "#define PAB_GIT_VERSION \"${PAB_GIT_VERSION}\"\n")

# Leave the file alone when nothing changed, so its timestamp does not trigger
# a rebuild of everything that includes it.
set(old "")
if (EXISTS ${OUTPUT})
    file(READ ${OUTPUT} old)
endif()
if (NOT old STREQUAL content)
    file(WRITE ${OUTPUT} "${content}")
endif()
