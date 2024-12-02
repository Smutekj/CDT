add_compile_options($<$<CONFIG:Debug>:-O0>)
# in Release mode, add aggressive optimizations
add_compile_options($<$<CONFIG:Release>:-O2>)
add_compile_options($<$<CONFIG:Release>:-fno-finite-math-only>)


if (CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
    # add_compile_options($<$<CONFIG:Release>:-march=native>)
endif()

option(ENABLE_APPROXMATH "Use approximate math" off) 
if (ENABLE_APPROXMATH)
    add_definitions(-DFAU_APPROXMATH)
endif ()
