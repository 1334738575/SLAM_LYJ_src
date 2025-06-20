cmake_minimum_required(VERSION 3.10)
project(SLAM_LYJ)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")
if(WIN32)
    if(MSVC)
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Zi")
        # set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /O2 /Ob2")
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Od /Ob0")
        set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /DEBUG")
        add_link_options(/DEBUG) #生成调试信息（测试在visual studio中有效）
    endif()
elseif(LINUX)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -g")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -g")
endif()

#load config
set(SLAM_LYJ_HOME_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../")
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/../config/config.h.in
    ${CMAKE_CURRENT_SOURCE_DIR}/../config/config.h
    @ONLY
)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../config)

#add file and subdirectory
file(GLOB SLAM_LYJ_BASE
    base/*.h
    base/*.CPP)
file(GLOB SLAM_LYJ_COMMON
    common/*.h
    common/*.cpp)
file(GLOB SLAM_LYJ_CUDA
    # cudaImp/*.h
    # cudaImp/*.cu
    )
file(GLOB SLAM_LYJ_VULKAN
    vulkanImp/*.h
    vulkanImp/*.cpp)
file(GLOB SLAM_LYJ_EXTRACTOR
    extractor/*.h
    extractor/*.cpp)
file(GLOB SLAM_LYJ_MATCHER
    matcher/*.h
    matcher/*.cpp)
file(GLOB SLAM_LYJ_OPTIMIZER
    optimizer/*.h
    optimizer/*.cpp)
file(GLOB SLAM_LYJ_PROCESSOR
    processor/*.h
    processor/*.cpp)
file(GLOB SLAM_LYJ_DEBUGGER
    debugger/*.h
    debugger/*.cpp)
file(GLOB SLAM_LYJ_OUT
    *.h
    *.cpp)
file(GLOB THIRDPARTY_STLPLUS
    thirdParty/STLPlus/include/*.h
    thirdParty/STLPlus/include/*.cpp)
file(GLOB SLAM_LYJ_VULKAN_TEXTURE_SHADER
    vulkanImp/shader/texture/*.frag
    vulkanImp/shader/texture/*.vert)
file(GLOB SLAM_LYJ_VULKAN_TRIANGLE_SHADER
    vulkanImp/shader/triangle/*.frag
    vulkanImp/shader/triangle/*.vert)
source_group(base/${_source_path_msvc} FILES ${SLAM_LYJ_BASE})
source_group(common/${_source_path_msvc} FILES ${SLAM_LYJ_COMMON})
source_group(cudaImp/${_source_path_msvc} FILES ${SLAM_LYJ_CUDA})
source_group(vulkanImp/${_source_path_msvc} FILES ${SLAM_LYJ_VULKAN})
source_group(extractor/${_source_path_msvc} FILES ${SLAM_LYJ_EXTRACTOR})
source_group(matcher/${_source_path_msvc} FILES ${SLAM_LYJ_MATCHER})
source_group(optimizer/${_source_path_msvc} FILES ${SLAM_LYJ_OPTIMIZER})
source_group(processor/${_source_path_msvc} FILES ${SLAM_LYJ_PROCESSOR})
source_group(debugger/${_source_path_msvc} FILES ${SLAM_LYJ_DEBUGGER})
source_group(./${_source_path_msvc} FILES ${SLAM_LYJ_OUT})
source_group(thirdParty/STLPlus/${_source_path_msvc} FILES ${THIRDPARTY_STLPLUS})
source_group(vulkanImp/shader/texture/${_source_path_msvc} FILES ${SLAM_LYJ_VULKAN_TEXTURE_SHADER})
source_group(vulkanImp/shader/triangle/${_source_path_msvc} FILES ${SLAM_LYJ_VULKAN_TRIANGLE_SHADER})

# vulkan shader compile
macro(GenShader args)
    foreach(arg ${args})
        execute_process(COMMAND ${SHADERCOMPILER} -V ${arg} -o ${arg}.spv)
    endforeach()
endmacro()
if(NOT USE_OLD_SHADER)
    file(GLOB ALLSHADERS ${CMAKE_CURRENT_SOURCE_DIR}/vulkanImp/shader/*)
    foreach(SHADER ${ALLSHADERS})
        if(IS_DIRECTORY ${SHADER})
            file(GLOB VERTRSHADERS ${SHADER}/*.vert)
            list(LENGTH VERTRSHADERS lenTmp)
            if(NOT lenTmp EQUAL 0)
                GenShader(${VERTRSHADERS})
            endif()
            file(GLOB FRAGSHADERS ${SHADER}/*.frag)
            list(LENGTH FRAGSHADERS lenTmp)
            if(NOT lenTmp EQUAL 0)
                GenShader(${FRAGSHADERS})
            endif()
            file(GLOB COMPSHADERS ${SHADER}/*.comp)
            list(LENGTH COMPSHADERS lenTmp)
            if(NOT lenTmp EQUAL 0)
                GenShader(${COMPSHADERS})
            endif()
        endif()
    endforeach()
endif()

# if compile apart
if(MULTI_DLL EQUAL 0)
    # message(FATAL_ERROR "single")
    #后续拆分为多个库
    include_directories(
        .
        common
        base
        vulkanImp
        extractor
        matcher
        optimizer
        processor
        debugger
        thirdParty
    )

    add_library(${PROJECT_NAME} SHARED
        ${THIRDPARTY_STLPLUS}
        ${SLAM_LYJ_BASE}
        ${SLAM_LYJ_COMMON}
        ${SLAM_LYJ_VULKAN}
        ${SLAM_LYJ_EXTRACTOR}
        ${SLAM_LYJ_MATCHER}
        ${SLAM_LYJ_OPTIMIZER}
        ${SLAM_LYJ_PROCESSOR}
        ${SLAM_LYJ_DEBUGGER}
        ${SLAM_LYJ_OUT}
        ${SLAM_LYJ_CUDA}
        ${SLAM_LYJ_VULKAN_TEXTURE_SHADER}
        ${SLAM_LYJ_VULKAN_TRIANGLE_SHADER}
        )

    target_link_libraries(${PROJECT_NAME} 
        Ceres::ceres 
        glm::glm 
        ${OpenCV_LIBS} 
        ${VULKAN_LIBRARIES} 
        ${GLFW_LIBRARIES} 
        ${Boost_LIBRARIES}
        ${PCL_LIBRARIES}
        Qt5::Core Qt5::Gui Qt5::Widgets
        ${GLEW_LIBRARIES}
        ${CUDA_LIBRARIES}
    )
else()
    # message(FATAL_ERROR "multi")
    #generate dll libraries
    # file(GLOB SLAM_LYJ_BASE_H base/*.h)
    # file(GLOB SLAM_LYJ_COMMON_H common/*.h)
    # file(GLOB SLAM_LYJ_VULKAN_H vulkanImp/*.h)
    # file(GLOB SLAM_LYJ_EXTRACTOR_H extractor/*.h)
    # file(GLOB SLAM_LYJ_MATCHER_H matcher/*.h)
    # file(GLOB SLAM_LYJ_OPTIMIZER_H optimizer/*.h)
    # file(GLOB SLAM_LYJ_PROCESSOR_H processor/*.h)
    # file(GLOB SLAM_LYJ_DEBUGGER_H debugger/*.h)
    # file(GLOB SLAM_LYJ_H *.h)
    # file(GLOB THIRDPARTY_STLPLUS_H thirdParty/STLPlus/include/*.hpp)
    aux_source_directory(base SLAM_LYJ_BASE_SRC)
    aux_source_directory(common SLAM_LYJ_COMMON_SRC)
    # aux_source_directory(vulkanImp SLAM_LYJ_VULKAN_SRC)
    aux_source_directory(extractor SLAM_LYJ_EXTRACTOR_SRC)
    aux_source_directory(matcher SLAM_LYJ_MATCHER_SRC)
    aux_source_directory(optimizer SLAM_LYJ_OPTIMIZER_SRC)
    # aux_source_directory(processor SLAM_LYJ_PROCESSOR_SRC)
    aux_source_directory(debugger SLAM_LYJ_DEBUGGER_SRC)
    aux_source_directory(. SLAM_LYJ_SRC)
    aux_source_directory(thirdParty/STLPlus/src THIRDPARTY_STLPLUS_SRC)
    add_library(SLAM_LYJ_BASE_LIB SHARED ${SLAM_LYJ_BASE_SRC})
    target_include_directories(SLAM_LYJ_BASE_LIB PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/base>
        $<INSTALL_INTERFACE:include>)
    add_library(SLAM_LYJ_COMMON_LIB SHARED ${SLAM_LYJ_COMMON_SRC})
    target_include_directories(SLAM_LYJ_COMMON_LIB PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/common>
        $<INSTALL_INTERFACE:include>)
    # add_library(SLAM_LYJ_VULKAN_LIB SHARED ${SLAM_LYJ_VULKAN_SRC})
    # target_include_directories(SLAM_LYJ_VULKAN_LIB PUBLIC
    #     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    #     # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vulkanImp>
    #     $<INSTALL_INTERFACE:include>)
    # target_link_libraries(SLAM_LYJ_VULKAN_LIB glm::glm)
    add_library(SLAM_LYJ_EXTRACTOR_LIB SHARED ${SLAM_LYJ_EXTRACTOR_SRC})
    target_include_directories(SLAM_LYJ_EXTRACTOR_LIB PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/extractor>
        $<INSTALL_INTERFACE:include>)
    add_library(SLAM_LYJ_MATCHER_LIB SHARED ${SLAM_LYJ_MATCHER_SRC})
    target_include_directories(SLAM_LYJ_MATCHER_LIB PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/matcher>
        $<INSTALL_INTERFACE:include>)
    add_library(SLAM_LYJ_OPTIMIZER_LIB SHARED ${SLAM_LYJ_OPTIMIZER_SRC})
    target_include_directories(SLAM_LYJ_OPTIMIZER_LIB PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/optimizer>
        $<INSTALL_INTERFACE:include>)
    # add_library(SLAM_LYJ_PROCESSOR_LIB SHARED ${SLAM_LYJ_PROCESSOR_SRC})
    # target_include_directories(SLAM_LYJ_PROCESSOR_LIB PUBLIC
    #     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    #     # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/processor>
    #     $<INSTALL_INTERFACE:include>)
    add_library(SLAM_LYJ_DEBUGGER_LIB SHARED ${SLAM_LYJ_DEBUGGER_SRC})
    target_include_directories(SLAM_LYJ_DEBUGGER_LIB PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/debugger>
        $<INSTALL_INTERFACE:include>)
    add_library(THIRDPARTY_STLPLUS_LIB SHARED ${THIRDPARTY_STLPLUS_SRC})
    target_include_directories(THIRDPARTY_STLPLUS_LIB PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdParty>
        # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/STLPlus/include>
        $<INSTALL_INTERFACE:include>/thirdParty)

    add_library(SLAM_LYJ SHARED ${SLAM_LYJ_SRC})
    target_include_directories(SLAM_LYJ PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>)

    target_link_libraries(${PROJECT_NAME}
        Ceres::ceres
        SLAM_LYJ_BASE_LIB
        SLAM_LYJ_COMMON_LIB
        # SLAM_LYJ_VULKAN_LIB
        SLAM_LYJ_EXTRACTOR_LIB
        SLAM_LYJ_MATCHER_LIB
        SLAM_LYJ_OPTIMIZER_LIB
        # SLAM_LYJ_PROCESSOR_LIB
        SLAM_LYJ_DEBUGGER_LIB
        THIRDPARTY_STLPLUS_LIB
        )

endif()

# set debug info
if(WIN32)
    if(MSVC)
        target_compile_options(${PROJECT_NAME} PRIVATE "$<$<CONFIG:Release>:/O2;/Ob2>")
        target_link_options(${PROJECT_NAME} PRIVATE "$<$<CONFIG:Release>:/DEBUG>")
    endif()
endif()

# show as a folder
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER ${PROJECT_NAME})
target_compile_definitions(${PROJECT_NAME} PRIVATE SLAM_LYJ_API_EXPORTS)


if(DEFINED LIBRARY_LYJ_OUTPUT_PATH)
    set_target_properties(${PROJECT_NAME} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${LIBRARY_LYJ_OUTPUT_PATH}
    ARCHIVE_OUTPUT_DIRECTORY ${LIBRARY_LYJ_OUTPUT_PATH}
    RUNTIME_OUTPUT_DIRECTORY ${LIBRARY_LYJ_OUTPUT_PATH}
    )
endif()

# # copy library after build using command
# add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
# COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${PROJECT_NAME}> ${CMAKE_CURRENT_BINARY_DIR}/../example/Release
# COMMENT "Copying ${PROJECT_NAME}.dll to ${CMAKE_CURRENT_BINARY_DIR}/../example/Release"
# )


####################install components##############################
# cmake_minimum_required(VERSION 3.5)

# project(MyLibrary VERSION 1.0)

# # 添加库
# add_library(MyLibrary SHARED src/MyLibrary.cpp)
# add_library(MyLibraryExtra SHARED src/MyLibraryExtra.cpp)

# # 设置库的公共头文件
# target_include_directories(MyLibrary PUBLIC include)
# target_include_directories(MyLibraryExtra PUBLIC include)

# # 设置安装规则
# install(TARGETS MyLibrary MyLibraryExtra
#         EXPORT MyLibraryTargets
#         LIBRARY DESTINATION lib
#         ARCHIVE DESTINATION lib
#         RUNTIME DESTINATION bin)

# # 安装导出文件
# install(EXPORT MyLibraryTargets
#         FILE MyLibraryTargets.cmake
#         NAMESPACE MyLibrary::
#         DESTINATION lib/cmake/MyLibrary)

# # 创建配置文件
# include(CMakePackageConfigHelpers)

# # 生成配置文件模板
# configure_package_config_file(MyLibraryConfig.cmake.in
#                               MyLibraryConfig.cmake
#                               INSTALL_DESTINATION lib/cmake/MyLibrary
#                               NO_SET_AND_CHECK_MACRO)

# # 安装配置文件
# install(FILES "${CMAKE_CURRENT_BINARY_DIR}/MyLibraryConfig.cmake"
#         DESTINATION lib/cmake/MyLibrary)

# # 安装头文件
# install(FILES include/MyLibrary.h include/MyLibraryExtra.h
#         DESTINATION include/MyLibrary)