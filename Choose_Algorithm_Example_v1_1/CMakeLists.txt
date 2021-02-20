# CMakeList.txt: проект CMake для Receive_ADC_Data_Example; включите исходный код и определения,
# укажите здесь логику для конкретного проекта.
#
cmake_minimum_required (VERSION 3.8)

# Добавьте источник в исполняемый файл этого проекта.
add_executable (Choose_Algorithm_Example_1_1 Choose_Algorithm_Example_v1_1.cpp)

# TODO: Добавьте тесты и целевые объекты, если это необходимо.
target_link_libraries(Choose_Algorithm_Example_1_1 PRIVATE LMP_CommunicationLibrary)