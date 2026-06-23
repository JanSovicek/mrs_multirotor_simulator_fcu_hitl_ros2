# filt/CMakeLists.txt

# Collect all C/C++ source files inside src/
file(GLOB FILT_SRC CONFIGURE_DEPENDS "src/*.c" "src/*.cpp")

#Define the library target
add_library(filt SHARED ${FILT_SRC})

# Add include directory so other targets can include this library headers
target_include_directories(filt PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
    $<INSTALL_INTERFACE:include>
)

# Link dependencies
target_link_libraries(filt
    PUBLIC
        uMsgLib
        Eigen3::Eigen
)