cmake_minimum_required(VERSION 3.20)
project(AscomCppBindings LANGUAGES CXX)

# **C++20支持**
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# **Windows平台检查**
if(NOT WIN32)
    message(FATAL_ERROR "ASCOM is only supported on Windows")
endif()

# **编译定义**
add_definitions(-DWIN32_LEAN_AND_MEAN -DNOMINMAX -D_WIN32_DCOM)

# **查找线程库**
find_package(Threads REQUIRED)

# **源文件**
set(ASCOM_HEADERS
    AscomInterfaces.h
    AscomSmartPtr.h
    AscomDriver.h
    AscomTelescope.h
    AscomCamera.h
)

set(ASCOM_SOURCES
    main.cpp
)

# **创建可执行文件**
add_executable(AscomExample ${ASCOM_SOURCES} ${ASCOM_HEADERS})

# **链接库**
target_link_libraries(AscomExample
    PRIVATE
    ole32
    oleaut32
    uuid
    advapi32
    Threads::Threads
)

# **编译器特定设置**
if(MSVC)
    target_compile_options(AscomExample PRIVATE
        /W4
        /permissive-
        /Zc:__cplusplus
        /std:c++20
    )
    
    # **调试配置**
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        target_compile_options(AscomExample PRIVATE /Od /Zi /MDd)
        target_link_options(AscomExample PRIVATE /DEBUG)
    else()
        target_compile_options(AscomExample PRIVATE /O2 /MD)
    endif()
endif()

# **输出目录设置**
set_target_properties(AscomExample PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# **创建输出目录**
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/bin/images)

# **安装规则**
install(TARGETS AscomExample
    RUNTIME DESTINATION bin
)

install(FILES ${ASCOM_HEADERS}
    DESTINATION include/ascom
)