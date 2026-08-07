# tests/ and sim/ compile Core/Src/Lib with the host compiler, which needs
# static_assert as a bare C23 keyword - Core/Inc/Lib/hw_setup.h uses it, and
# GCC did not support that until GCC 13. Older GCC does not reject it kindly:
# it reads `static_assert` as an implicit-int function declaration and fails
# several files deep with "expected declaration specifiers", which does not
# say what the actual problem is. Caught here instead, with a message that
# does.
#
# Not a concern for emcmake's build (Emscripten's Clang has had this for
# years) - only ever fires for CMAKE_C_COMPILER_ID STREQUAL "GNU".
if(CMAKE_C_COMPILER_ID STREQUAL "GNU" AND CMAKE_C_COMPILER_VERSION VERSION_LESS 13)
  message(FATAL_ERROR
    "Host GCC ${CMAKE_C_COMPILER_VERSION} is too old for this build - GCC 13+ "
    "needed (Core/Inc/Lib/hw_setup.h uses static_assert as a bare C23 "
    "keyword, which GCC did not support before 13). Ubuntu 24.04 and Debian "
    "13 ship this by default; on an older system, install a newer gcc "
    "alongside the system one and point CC at it rather than replacing the "
    "system compiler - see docs/setup.md.")
endif()
