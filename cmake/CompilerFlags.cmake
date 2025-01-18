
function(set_compiler_flags project_name)
    if(MSVC)
    target_compile_options(${project_name} PRIVATE $<$<CONFIG:Debug>:/Od>)
    # in Release mode, add aggressive optimizations
    target_compile_options(${project_name} PRIVATE $<$<CONFIG:Release>:/O2>)
    else()
    target_compile_options(${project_name} PRIVATE $<$<CONFIG:Debug>:-O0>)
    # in Release mode, add aggressive optimizations
    target_compile_options(${project_name} PRIVATE $<$<CONFIG:Release>:-O2>)
    endif()

    if (CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
    target_compile_options(${project_name} PRIVATE $<$<CONFIG:Release>:-march=native>)
    endif()

endfunction()
