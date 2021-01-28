# CMakeList.txt: проект CMake для Receive_ADC_Data_Example; включите исходный код и определения,
# укажите здесь логику для конкретного проекта.
#
cmake_minimum_required (VERSION 3.8)

# Добавьте источник в исполняемый файл этого проекта.
add_executable (Choose_Algorithm_Example_linux Choose_Algorithm_Example_linux.cpp)

# TODO: Добавьте тесты и целевые объекты, если это необходимо.
target_link_libraries(Choose_Algorithm_Example_linux PRIVATE LMP_CommunicationLibrary)