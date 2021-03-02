# CMakeList.txt: файл проекта CMake верхнего уровня; проведите глобальную конфигурацию
# и включите сюда подпроекты.
#
cmake_minimum_required (VERSION 3.11)
project (Examples)

# Включите подпроекты.
add_subdirectory (LMP_CommunicationLibrary)

add_subdirectory (Receive_Data_in_Console)

add_subdirectory (Receive_ADC_Data_Example)

add_subdirectory (Choose_Algorithm_Example_v1_1)

add_subdirectory (Console_Data_Logger_v1_1)

add_subdirectory (Receive_ADC_Data_Example_linux)

add_subdirectory (Choose_Algorithm_Example_linux)

add_subdirectory (Choose_Algorithm_Example_linux_v1_1)

add_subdirectory (Console_Data_Logger_linux_v1_1)


