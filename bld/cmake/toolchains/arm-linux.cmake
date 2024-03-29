set(CMAKE_CROSSCOMPILE TRUE)
set(CMAKE_SYSTEM_PROCESS arm)

set(toolchain_root /usr/bin)
set(bin_prefix arm-linux-gnueabihf)

set(CMAKE_C_COMPILER ${toolchain_root}/${bin_prefix}-gcc)
set(CMAKE_CXX_COMPILER ${toolchain_root}/${bin_prefix}-g++)
set(CMAKE_AR ${toolchain_root}/${bin_prefix}-ar CACHE FILEPATH "Arhiver")
set(CMAKE_AS ${toolchain_root}/${bin_prefix}-as CACHE FILEPATH "Assmebler")
set(CMAKE_LD ${toolchain_root}/${bin_prefix}-ld CACHE FILEPATH "Linker")
set(CMAKE_NM ${toolchain_root}/${bin_prefix}-nm CACHE FILEPATH "NM")
set(CMAKE_RANLIB ${toolchain_root}/${bin_prefix}-ranlib CACHE FILEPATH "Ranlib")
set(CMAKE_STRIP ${toolchain_root}/${bin_prefix}-strip CACHE FILEPATH "Strip")
set(CMAKE_SIZE ${toolchain_root}/${bin_prefix}-size CACHE FILEPATH "Size")