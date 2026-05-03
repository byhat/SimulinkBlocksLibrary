cmake_minimum_required(VERSION 3.5)

# Add OpenXLSX once at this level (shared by FlightGearLogger and FlightGearTestsForNN)
# Disable samples and tests to avoid duplicate targets
set(OPENXLSX_BUILD_SAMPLES OFF CACHE BOOL "" FORCE)
set(OPENXLSX_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(OPENXLSX_CREATE_DOCS OFF CACHE BOOL "" FORCE)
add_subdirectory(FlightGearLogger/external/OpenXLSX)

add_subdirectory(Main)
add_subdirectory(FlightGearExample)
add_subdirectory(FlightGearLogger)
add_subdirectory(FlightGearTestsForNN)
add_subdirectory(JoystickExample)
