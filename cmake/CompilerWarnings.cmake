# cmake/CompilerWarnings.cmake
# Configures pedantic compiler warnings for targets in VAPFL

function(vapfl_set_compiler_warnings target_name)
  option(VAPFL_ENABLE_WERROR "Treat warnings as errors" ON)
  option(VAPFL_ENABLE_PEDANTIC_WARNINGS "Enable pedantic compiler warnings" ON)

  if(NOT VAPFL_ENABLE_PEDANTIC_WARNINGS)
    return()
  endif()

  set(CLANG_GCC_WARNINGS
    -Wall
    -Wextra
    -Wpedantic
    -Wconversion            # Warn on implicit type conversions
    -Wsign-conversion       # Warn on sign conversions
    -Wshadow                # Warn when a local variable shadows another
    -Wnon-virtual-dtor      # Warn if a polymorphic class has non-virtual destructor
    -Wold-style-cast        # Enforce C++ static_cast / reinterpret_cast
    -Wcast-align            # Warn on alignment-increasing pointer casts
    -Wunused                # Warn on unused variables and functions
    -Woverloaded-virtual    # Warn if an overload hides a virtual method
    -Wnull-dereference      # Warn on potential null pointer dereferences
    -Wdouble-promotion      # Warn when float is implicitly promoted to double
    -Wformat=2              # Strict format string validation
  )

  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # Additional Clang-specific warnings if desired
  endif()

  if(VAPFL_ENABLE_WERROR)
    list(APPEND CLANG_GCC_WARNINGS -Werror)
  endif()

  if(MSVC)
    set(MSVC_WARNINGS
      /W4
      /permissive-
    )
    if(VAPFL_ENABLE_WERROR)
      list(APPEND MSVC_WARNINGS /WX)
    endif()
    target_compile_options(${target_name} PRIVATE ${MSVC_WARNINGS})
  else()
    target_compile_options(${target_name} PRIVATE ${CLANG_GCC_WARNINGS})
  endif()
endfunction()
