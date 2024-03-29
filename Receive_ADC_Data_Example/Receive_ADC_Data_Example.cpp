﻿#ifdef _WIN32

#include <windows.h>
#include <iostream>
#include <stdio.h>
#include "LMP_Device.h"
using namespace Gyrovert;
using namespace std;

HANDLE hSerial;
LMP_Device* GKV;
uint8_t algorithm_selected = 0;

bool InitSerialPort(string port_name, int32_t baudrate);
char* ReadCOM();
void WriteCOM(GKV_PacketBase* buf);
void ShowPacketData(LMP_Device* GKV, GKV_ADCData* buf);

int main()
{
    /* Select Serial Port */
    string com_port;
    cout << "Set Serial Port:";
    cin >> com_port;
    cout << "#start connecting to " << com_port << "\n";
    //Create LMP Device Object GKV
    GKV = new LMP_Device();
    //Serial Port Settings For Windows
    com_port = "\\\\.\\" + com_port;
    if (!(InitSerialPort(com_port, 921600))) return 1;
    // GKV Settings
    GKV->SetSendDataFunction(WriteCOM);//Set User Function That Sends Data to Serial Port connected to GKV
    GKV->SetReceiveDataFunction(ReadCOM);//Set User Function That Receives Data From Serial Port And Returns Received Byte
    GKV->SetADCDataReceivedCallback(ShowPacketData);//Set User Callback for Parsed ADC GKV Packet
    GKV->RunDevice();//Run Thread For Receiving Data From GKV
    while (!(algorithm_selected))
    {
        GKV->SetDefaultAlgorithmPacket();
        GKV->SetAlgorithm(GKV_ADC_CODES_ALGORITHM);
        _sleep(10);
    }
    cout << "#start main loop\n";
    while (1)
    {
        _sleep(1000);
        //do something
    }
    return 0;
}

bool InitSerialPort(string port_name, int32_t baudrate)
{
    hSerial = ::CreateFileA(port_name.c_str(), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
    if (hSerial == INVALID_HANDLE_VALUE)
    {
        if (GetLastError() == ERROR_FILE_NOT_FOUND)
        {
            cout << "serial port does not exist.\n";
            return 0;
        }
        cout << "some other error occurred.\n";
        return 0;
    }
    cout << "#connect ok\n";
    DCB dcbSerialParams = { 0 };
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(hSerial, &dcbSerialParams))
    {
        cout << "getting state error\n";
        return 0;
    }
    cout << "#get state ok\n";
    dcbSerialParams.BaudRate = baudrate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    if (!SetCommState(hSerial, &dcbSerialParams))
    {
        cout << "error setting serial port state\n";
        return 0;
    }
    return 1;
}

void WriteCOM(GKV_PacketBase* buf)
{
    DWORD dwBytesWritten;
    char iRet = WriteFile(hSerial, buf, buf->length + 8, &dwBytesWritten, NULL);
    Sleep(1);
}

char* ReadCOM()
{
    static char ReceivedData[2048] = { 0 };
    DWORD iSize;
    char iRet = 0;
    while (true)
    {
        iRet = ReadFile(hSerial, &ReceivedData, sizeof(ReceivedData), &iSize, 0);
        if (iRet)
        {
            if (iSize > 0)
            {
                GKV->SetReceiveBufferSize(iSize);
                return ReceivedData;
            }
        }
        _sleep(10);
    }
    return 0;
}

void ShowPacketData(LMP_Device* GKV, GKV_ADCData* packet)
{
    string output_str;
    output_str.append("ADC Data Packet: ");
    output_str.append("Sample Counter = " + std::to_string(packet->sample_cnt));
    output_str.append(" ax = " + std::to_string((int32_t)packet->a[0]));
    output_str.append(" ay = " + std::to_string((int32_t)packet->a[1]));
    output_str.append(" az = " + std::to_string((int32_t)packet->a[2]));
    output_str.append(" wx = " + std::to_string((int32_t)packet->w[0]));
    output_str.append(" wy = " + std::to_string((int32_t)packet->w[1]));
    output_str.append(" wz = " + std::to_string((int32_t)packet->w[2]));
    cout << output_str << endl;
    algorithm_selected = 1;
}
#else
int main()
{
}
#endif
