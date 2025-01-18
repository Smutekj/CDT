
if(MSVC)
add_compile_options($<$<CONFIG:Debug>:/Od>)
# in Release mode, add aggressive optimizations
add_compile_options($<$<CONFIG:Release>:/O2>)
else()
add_compile_options($<$<CONFIG:Debug>:-O0>)
# in Release mode, add aggressive optimizations
add_compile_options($<$<CONFIG:Release>:-O2>)
endif()

if (CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
    add_compile_options($<$<CONFIG:Release>:-march=native>)
endif()
