cmake_minimum_required(VERSION 3.5)

project(FlightGearTestsForNN LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Boost with required components
find_package(Boost REQUIRED)

# Create executable
add_executable(FlightGearTestsForNN main.cpp
    AsyncExcelWriter.h
)

# Link libraries
target_link_libraries(FlightGearTestsForNN
    PUBLIC
        Boost::boost
        OpenXLSX::OpenXLSX
)

# Include directories
target_include_directories(FlightGearTestsForNN
    PRIVATE
        ${CMAKE_SOURCE_DIR}/../include
)

# Installation
include(GNUInstallDirs)
install(TARGETS FlightGearTestsForNN
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
