cmake_minimum_required(VERSION 3.5)
project(Vanttec_CANLib)

set(CMAKE_CXX_STANDARD 14)

find_package( Boost REQUIRED )
include_directories( ${Boost_INCLUDE_DIRS} )

file(GLOB_RECURSE Vanttec_CANLib_SRC src/*.cpp)

add_library(Vanttec_CANLibLIB ${Vanttec_CANLib_SRC})
target_link_libraries(Vanttec_CANLibLIB ${Boost_LIBRARIES} )

include_directories(src src/Vanttec_CANLib)
add_executable(Vanttec_CANLib main.cpp)
target_link_libraries(Vanttec_CANLib pthread Vanttec_CANLibLIB)


file(GLOB_RECURSE Vanttec_CANLib_Tests tests/*.cpp)

if(${CMAKE_VERSION} VERSION_GREATER 3.11)
include(FetchContent)
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()
add_executable(Vanttec_CANLib_test ${Vanttec_CANLib_Tests})
target_link_libraries(Vanttec_CANLib_test gtest_main Vanttec_CANLibLIB)

include(GoogleTest)
gtest_discover_tests(Vanttec_CANLib_test)
endif()