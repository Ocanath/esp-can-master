cmake_minimum_required(VERSION 3.10)
project(esp-can-cortex-tests C)

# Use C99
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

#------------------------------------------------------------------------------
# 1) Unity
#------------------------------------------------------------------------------
add_library(Unity STATIC
    vendor/unity/src/unity.c
)
target_include_directories(Unity PUBLIC
    vendor/unity/src
)

#------------------------------------------------------------------------------
# 2) IK library
#------------------------------------------------------------------------------
add_library(sauron-eye-closedform-ik STATIC
    Core/Src/sauron-eye-closedform-ik.c
	Core/Src/sin-math.c
)
target_include_directories(sauron-eye-closedform-ik PUBLIC
    Core/Inc/
)

#------------------------------------------------------------------------------
# 3) Test runner executable
#------------------------------------------------------------------------------
add_executable(test_runner
    tests/test_runner.c
    tests/test_sauron-eye-closedform-ik.c
)
# Include headers for both IK and Unity
target_include_directories(test_runner PRIVATE
    Core/Inc
    vendor/unity/src
)
# Link in the IK code and Unity test framework
target_link_libraries(test_runner PRIVATE
	sauron-eye-closedform-ik
    Unity
)
