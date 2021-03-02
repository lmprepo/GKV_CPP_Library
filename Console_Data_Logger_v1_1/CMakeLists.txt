# CMakeList.txt: проект CMake для Console_Data_Logger_linux; включите исходный код и определения,
# укажите здесь логику для конкретного проекта.
#
cmake_minimum_required (VERSION 3.8)

add_executable (Console_Data_Logger_v1_1 Console_Data_Logger_v1_1.cpp)

target_compile_features(Console_Data_Logger_v1_1 PRIVATE cxx_std_17)

target_link_libraries(Console_Data_Logger_v1_1
    PRIVATE LMP_CommunicationLibrary)
