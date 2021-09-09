cmake_minimum_required(VERSION 3.10)

project(ecat_master)

#add_compile_options(-O2)

set(xeno_cflags_params "--posix" "--cflags")
execute_process(
    COMMAND xeno-config ${xeno_cflags_params}
    OUTPUT_VARIABLE xeno_cflags
    OUTPUT_STRIP_TRAILING_WHITESPACE)

set(xeno_ldflags_params "--posix" "--ldflags" )
execute_process(
    COMMAND xeno-config ${xeno_ldflags_params}
    OUTPUT_VARIABLE xeno_ldflags
    OUTPUT_STRIP_TRAILING_WHITESPACE)


#set(xeno_ldflags "-Wl,--no-as-needed -Wl,@/usr/xenomai/lib/cobalt.wrappers -Wl,@/usr/xenomai/lib/modechk.wrappers  /usr/xenomai/lib/xenomai/bootstrap.o -Wl,--wrap=main -Wl,--dynamic-list=/usr/xenomai/lib/dynlist.ld -L/usr/xenomai/lib -lcobalt -lmodechk -lpthread -lrt")

# # Compiler and linker options
set(CMAKE_C_FLAGS          "${CMAKE_C_FLAGS} ${xeno_cflags} ${xeno_ldflags}")
# set(CMAKE_CXX_FLAGS        "${CMAKE_CXX_FLAGS}")
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${xeno_ldflags}")
#set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${xeno_ldflags}")
#set(CMAKE_STATIC_LINKER_FLAGS "${CMAKE_STATIC_LINKER_FLAGS} ${xeno_ldflags}")


include_directories(
    include
    /usr/local/include/soem
)

add_executable(ecat_master src/ecat_red.c src/ecat_settings.c)
target_link_libraries(ecat_master
-lsoem
)