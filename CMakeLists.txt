cmake_minimum_required(VERSION 3.20)

# ################################setting##################################
# cmake setting
project(SLAM_LYJ_src)
if(TARGET SLAM_LYJ_src)
    message(STATUS "目标 @NAME@ 已存在")
    return()
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")

if(WIN32)
    if(MSVC)
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Zi")

        # set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /O2 /Ob2")
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Od /Ob0")

        set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Zi")

        # set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /O2 /Ob2")
        set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Od /Ob0")

        # set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /DEBUG")
        add_link_options(/DEBUG) # 生成调试信息（测试在visual studio中有效）
    endif()
elseif(LINUX)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -g")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -g")

    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g")
endif()

if(MSVC)
    set_property(GLOBAL PROPERTY USE_FOLDERS ON)
endif()
option(USE_CONAN "use conan" OFF)

set(${PROJECT_NAME}_LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/Output)
set(${PROJECT_NAME}_CMAKE_FILE)
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install)


# ################################cmake##################################
if(DEFINED ${PROJECT_NAME}_CMAKE_FILE)
    include(${${PROJECT_NAME}_CMAKE_FILE})
elseif(EXISTS ${CMAKE_CURRENT_LIST_DIR}/../cmake/FuncLyj.cmake)
    include(${CMAKE_CURRENT_LIST_DIR}/../cmake/FuncLyj.cmake)
else()
    include(${CMAKE_CURRENT_LIST_DIR}/cmake/FuncLyj.cmake)
endif()

# # ###############################conan###################################
# if(USE_CONAN)
#     CheckConan(CONAN_STORAGE_PATH)

#     if(NOT CONAN_STORAGE_PATH)
#         message(FATAL_ERROR)
#     endif()

#     message(STATUS ${CONAN_STORAGE_PATH})
#     set(CONAN_PKG_NAMES Eigen OpenCV Qt GLFW GLEW)

#     foreach(PKG_NAME IN LISTS CONAN_PKG_NAMES)
#         message(STATUS "START CONFIG ${PKG_NAME} BY CONAN ...")

#         if(CONAN_${PKG_NAME}_PACKAGE_FOLDER)
#             message(STATUS "Has Found, skip")
#             list(APPEND CMAKE_MODULE_PATH "${CONAN_${PKG_NAME}_PACKAGE_FOLDER}")
#             continue()
#         endif()

#         GetPKGInfoInConan(${PKG_NAME} PKG_VERSION PKG_USER PKG_CHANNEL)
#         message(STATUS "Newest ${PKG_NAME}, version: ${PKG_VERSION}, user: ${PKG_USER}, channel: ${PKG_CHANNEL}")
#         FindConanPKG(${PKG_NAME} ${PKG_VERSION} ${PKG_USER} ${PKG_CHANNEL})

#         if(CONAN_${PKG_NAME}_PACKAGE_FOLDER)
#             list(APPEND CMAKE_MODULE_PATH "${CONAN_${PKG_NAME}_PACKAGE_FOLDER}")
#         endif()
#     endforeach(PKG_NAME IN LISTS CONAN_PKG_NAMES)
# endif()

# ################################package##################################
set(${PROJECT_NAME}_INCLUDE_DIRS)
set(${PROJECT_NAME}_LIBRARIES)
set(${PROJECT_NAME}_BINARY_DIRS)
# find_package(Eigen3)
# list(APPEND ${PROJECT_NAME}_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
# enable_language(CUDA)
# find_package(CUDA)
# set(CMAKE_CUDA_ARCHITECTURES 89)
# list(APPEND ${PROJECT_NAME}_INCLUDE_DIRS ${CUDA_INCLUDE_DIRS})
# list(APPEND ${PROJECT_NAME}_LIBRARIES ${CUDA_LIBRARIES})

# list(APPEND ${PROJECT_NAME}_BINARY_DIRS )
include_directories(${${PROJECT_NAME}_INCLUDE_DIRS})

# ################################code##################################
set(TOUPPER_NAME ${PROJECT_NAME})
list(TRANSFORM TOUPPER_NAME TOUPPER)
set(API_NAME ${TOUPPER_NAME}_API)
if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}_Defines.h)
    file(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}_Defines.h
"#ifndef ${TOUPPER_NAME}_DEFINES_H\n#define ${TOUPPER_NAME}_DEFINES_H\n\n
#ifdef WIN32
#ifdef _MSC_VER
#define ${API_NAME} __declspec(dllexport)
#else
#define ${API_NAME}
#endif
#else
#define ${API_NAME}
#endif
\n\n\nnamespace ${PROJECT_NAME}\n{
\n
\n}
\n#endif//${PROJECT_NAME}_DEFINES_H"
    )
endif()
if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}_Defines.cpp)
    file(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}_Defines.cpp
        "#include \"${PROJECT_NAME}_Defines.h\"
        \n\n\nnamespace ${PROJECT_NAME}\n{
        \n
        \n}"
    )
endif()
if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}_Include.h)
    file(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}_Include.h
"#ifndef ${TOUPPER_NAME}_INCLUDE_H\n#define ${TOUPPER_NAME}_INCLUDE_H\n
\n#include \"${PROJECT_NAME}_Defines.h\"
#include <stdio.h>
\n\n\nnamespace ${PROJECT_NAME}\n{
\n
${API_NAME} void print_${PROJECT_NAME}_Test();
\n
\n}
\n#endif//${PROJECT_NAME}_INCLUDE_H"
    )
endif()
if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}_Include.cpp)
    file(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}_Include.cpp
"#include \"${PROJECT_NAME}_Include.h\"
\n\n\nnamespace ${PROJECT_NAME}\n{
\n
${API_NAME} void print_${PROJECT_NAME}_Test()
{
    printf(\"Hello ${PROJECT_NAME}!\");
}
\n
\n}"
    )
endif()
set(${PROJECT_NAME}_SRC_FILES)
GroupFiles(${PROJECT_NAME}_SRC_FILES ${CMAKE_CURRENT_LIST_DIR} ".")

# ################################library##################################
add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SRC_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME} PRIVATE
    ${${PROJECT_NAME}_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PRIVATE
    ${${PROJECT_NAME}_LIBRARIES}
)
target_compile_definitions(${PROJECT_NAME} PRIVATE SLAM_LYJ_API_EXPORTS) # TODO

# ################################output##################################
if(DEFINED ${PROJECT_NAME}_LIBRARY_OUTPUT_PATH)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${${PROJECT_NAME}_LIBRARY_OUTPUT_PATH}
        ARCHIVE_OUTPUT_DIRECTORY ${${PROJECT_NAME}_LIBRARY_OUTPUT_PATH}
        RUNTIME_OUTPUT_DIRECTORY ${${PROJECT_NAME}_LIBRARY_OUTPUT_PATH}
    )
endif()

# ################################example##################################
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/example)
# 创建示例源码
if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/example/main.cpp)
    file(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/example/main.cpp
        "#include <iostream>\n#include <${PROJECT_NAME}_Defines.h>\n#include <${PROJECT_NAME}_Include.h>\n
        \nint main()\n{
        \n    ${PROJECT_NAME}::print_${PROJECT_NAME}_Test();
        \n}"
    )
endif()
add_executable(${PROJECT_NAME}Bin example/main.cpp)
target_include_directories(${PROJECT_NAME}Bin PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(${PROJECT_NAME}Bin PRIVATE ${PROJECT_NAME})

if(DEFINED ${PROJECT_NAME}_LIBRARY_OUTPUT_PATH)
    set_target_properties(${PROJECT_NAME}Bin PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${${PROJECT_NAME}_LIBRARY_OUTPUT_PATH}
        ARCHIVE_OUTPUT_DIRECTORY ${${PROJECT_NAME}_LIBRARY_OUTPUT_PATH}
        RUNTIME_OUTPUT_DIRECTORY ${${PROJECT_NAME}_LIBRARY_OUTPUT_PATH}
    )
endif()

if(MSVC)
    set_target_properties(${PROJECT_NAME}Bin PROPERTIES VS_DEBUGGER_ENVIRONMENT "PATH=${${PROJECT_NAME}_BINARY_DIRS}")
endif()

# ################################install##################################
file(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
    "@PACKAGE_INIT@\ninclude(\"\${CMAKE_CURRENT_LIST_DIR}/${PROJECT_NAME}Targets.cmake\" )"
)

# 安装库和头文件
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib # DLL 在 Windows 下会被安装到 bin/
    RUNTIME DESTINATION bin # 适用于 DLL 的运行时组件
    INCLUDES DESTINATION include
)

# #file
# install(FILES test.h
# DESTINATION include
# PERMISSIONS OWNER_READ GROUP_READ WORLD_READ)
# #directory
# install(DIRECTORY ./          # 注意末尾的斜杠
# DESTINATION include
# FILES_MATCHING PATTERN "*.h" # 仅匹配 .h 文件
# PERMISSIONS OWNER_READ GROUP_READ WORLD_READ
# )
# install(DIRECTORY ./
# DESTINATION include
# FILES_MATCHING
# PATTERN "*.h"            # 包含所有 .h 文件
# PATTERN "build/*"             # 匹配子目录名
# EXCLUDE                  # 排除该目录及其内容
# PATTERN "cmake/*"             # 匹配子目录名
# EXCLUDE                  # 排除该目录及其内容
# PATTERN "install/*"             # 匹配子目录名
# EXCLUDE                  # 排除该目录及其内容
# PATTERN "Lib/*"             # 匹配子目录名
# EXCLUDE                  # 排除该目录及其内容
# PERMISSIONS
# OWNER_READ GROUP_READ WORLD_READ
# )
install(DIRECTORY ./
    DESTINATION include
    FILES_MATCHING
    PATTERN "*.h" # 包含所有 .h 文件

    # 排除指定目录（递归匹配）
    PATTERN "build" EXCLUDE # 排除所有名为 build 的目录及内容
    PATTERN "cmake" EXCLUDE # 排除所有名为 cmake 的目录及内容
    PATTERN "install" EXCLUDE # 排除所有名为 install 的目录及内容
    PATTERN "Lib" EXCLUDE # 排除所有名为 Lib 的目录及内容
    PATTERN ".git" EXCLUDE # 排除所有名为 Lib 的目录及内容
    PATTERN ".vscode" EXCLUDE # 排除所有名为 Lib 的目录及内容
    PATTERN "example" EXCLUDE # 排除所有名为 Lib 的目录及内容
    PATTERN "Output" EXCLUDE # 排除所有名为 Lib 的目录及内容

    # 或使用正则表达式精确匹配路径（更严格）
    # PATTERN "/(build|cmake|install|Lib)(/.*)?$" EXCLUDE
    # 设置权限
    PERMISSIONS
    OWNER_READ GROUP_READ WORLD_READ
)

# 安装后删除空目录（可选）
install(SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/cleanup.cmake")
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/cleanup.cmake [[
file(REMOVE_RECURSE "${CMAKE_INSTALL_PREFIX}/include/build")
file(REMOVE_RECURSE "${CMAKE_INSTALL_PREFIX}/include/cmake")
file(REMOVE_RECURSE "${CMAKE_INSTALL_PREFIX}/include/install")
file(REMOVE_RECURSE "${CMAKE_INSTALL_PREFIX}/include/Lib")
]])

# set(TARGETNAME ${PROJECT_NAME}Targets)
# 导出目标配置
install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION lib/cmake
)

# 生成包配置文件
include(CMakePackageConfigHelpers)
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION lib/cmake
)
write_basic_package_version_file(
    ${PROJECT_NAME}ConfigVersion.cmake
    VERSION 1.0.0
    COMPATIBILITY SameMajorVersion
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    DESTINATION lib/cmake
)