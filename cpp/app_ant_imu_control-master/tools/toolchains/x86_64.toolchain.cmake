# Copy and paste this into your actual project and use it in your main CMakeLists.txt

set(CMAKE_SYSTEM_NAME                   Linux)
set(CMAKE_SYSTEM_PROCESSOR              x86_64)

set(CMAKE_AR                            /usr/bin/x86_64-linux-gnueabihf-ar)
set(CMAKE_ASM_COMPILER                  /usr/bin/x86_64-linux-gnueabihf-as)
set(CMAKE_C_COMPILER                    /usr/bin/x86_64-linux-gnueabihf-gcc-10)
set(CMAKE_CXX_COMPILER                  /usr/bin/x86_64-linux-gnueabihf-g++-10)
set(CMAKE_LINKER                        /usr/bin/x86_64-linux-gnueabihf-ld)
set(CMAKE_OBJCOPY                       /usr/bin/x86_64-linux-gnueabihf-objcopy)
set(CMAKE_RANLIB                        /usr/bin/x86_64-linux-gnueabihf-ranlib)
set(CMAKE_SIZE                          /usr/bin/x86_64-linux-gnueabihf-size)
set(CMAKE_STRIP                         /usr/bin/x86_64-linux-gnueabihf-strip)

# we use an empty string for rootfs, this way we could easily adapt it later for different root locations
set(ROOTFS )
set(CMAKE_SYSROOT                       ${ROOT_FS}/)
set(CMAKE_FIND_ROOT_PATH                ${ROOT_FS}/)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM   BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY   BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE   BOTH)

set(LIB_DIR ${ROOT_FS}/lib/x86_64-linux-gnueabihf)
message(NOTICE "-- Olix LIB_DIR: ${LIB_DIR}")

set(USR_LIB_DIR ${ROOT_FS}/usr/lib/x86_64-linux-gnueabihf)
message(NOTICE "-- Olix USR_LIB_DIR: ${USR_LIB_DIR}")

set(USR_X86_64_INCLUDE_DIR ${ROOT_FS}/usr/include/x86_64-linux-gnueabihf)
message(NOTICE "-- Olix USR_X86_64_INCLUDE_DIR: ${USR_X86_64_INCLUDE_DIR}")

set(USR_INCLUDE_DIR ${ROOT_FS}/usr/include)
message(NOTICE "-- Olix USR_INCLUDE_DIR: ${USR_INCLUDE_DIR}")

set(ROS2_INSTALL_DIR ${ROOT_FS}/opt/ros2/humble/install)
message(NOTICE "-- Olix ROS2_INSTALL_DIR: ${ROS2_INSTALL_DIR}")

set(Olive_INCLUDE_DIR ${ROOT_FS}/opt/olive/include)
message(NOTICE "-- Olix Olive_INCLUDE_DIR: ${Olive_INCLUDE_DIR}")

set(Olive_LIB_DIR ${ROOT_FS}/opt/olive/lib)
message(NOTICE "-- Olix Olive_LIB_DIR: ${Olive_LIB_DIR}")

link_directories(${LIB_DIR} ${USR_LIB_DIR} ${Olive_LIB_DIR})
include_directories(SYSTEM ${USR_INCLUDE_DIR} ${USR_X86_64_INCLUDE_DIR} ${Olive_INCLUDE_DIR})