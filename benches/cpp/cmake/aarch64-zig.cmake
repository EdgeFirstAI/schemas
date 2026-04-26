# Zig-based cross-compile toolchain for aarch64-linux-gnu.
# Usage: cmake -DCMAKE_TOOLCHAIN_FILE=cmake/aarch64-zig.cmake ...
#
# Requires `zig` on PATH (>= 0.13). zig c++ ships clang + libc++ + glibc
# headers, producing self-contained aarch64 ELFs that run on the target.

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# Wrapper scripts are required because CMake treats `zig ar` (a multi-word
# string) as a list and emits it with a `;` separator on invocation. The
# wrappers in cmake/zig-wrappers/ are simple `exec zig <subcommand> "$@"` shims.
set(_zig_wrap "${CMAKE_CURRENT_LIST_DIR}/zig-wrappers")
set(CMAKE_C_COMPILER   "${_zig_wrap}/zig-cc")
set(CMAKE_CXX_COMPILER "${_zig_wrap}/zig-cxx")
set(CMAKE_AR           "${_zig_wrap}/zig-ar"     CACHE FILEPATH "")
set(CMAKE_RANLIB       "${_zig_wrap}/zig-ranlib" CACHE FILEPATH "")

# Foonathan-memory enables -Werror and uses deprecated literal-operator syntax
# that zig's clang-derived front-end flags more aggressively than gcc. Demote
# the corresponding warnings to non-fatal so the FetchContent build succeeds.
set(CMAKE_C_FLAGS_INIT   "-Wno-error=deprecated-declarations -Wno-error=deprecated-literal-operator")
set(CMAKE_CXX_FLAGS_INIT "-Wno-error=deprecated-declarations -Wno-error=deprecated-literal-operator")

# Avoid trying to run target binaries on the host.
set(CMAKE_CROSSCOMPILING TRUE)
