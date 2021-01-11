cmake_minimum_required(VERSION 3.11)

add_library(LMP_CommunicationLibrary 
    src/LMP_Device.cpp     
    src/LMP_AlgParamPacket.h
    src/LMP_AlorithmPackets.h
    src/LMP_BasePacket.h
    src/LMP_Device.h
    src/LMP_CustomPacket.h
    src/LMP_FilterPacket.h
    src/LMP_GyroOffsetPacket.h
    src/LMP_SelfTestPacket.h
    src/LMP_SettingsPacket.h
    src/LMP_TestPacket.h)

    target_include_directories(LMP_CommunicationLibrary PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")