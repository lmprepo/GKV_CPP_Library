
#include <windows.h>
#include <iostream>
#include <stdio.h>
#include "LMP_Device.h"
using namespace std;

HANDLE hSerial;
uint8_t algorithm_selected = 0;

char ReadCOM();
void WriteCOM(PacketBase* buf);
void ShowPacketData(ADCData* buf);

int main()
{
    string com_port;
    cout << "Set Serial Port:";
    cin >> com_port;
    cout << "#start connecting to " << com_port << "\n";
    uint8_t Packet_is_Correct = 0;
    uint8_t algorithm_packet = GKV_ADC_CODES_PACKET;
    //Create LMP Device Object GKV
    LMP_Device *GKV = new LMP_Device();
    //Serial Port Settings For Windows
    std::string sPortName = "\\\\.\\" + std::string(com_port);
    hSerial = ::CreateFileA(sPortName.c_str(), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
    if (hSerial == INVALID_HANDLE_VALUE)
    {
        if (GetLastError() == ERROR_FILE_NOT_FOUND)
        {
            cout << "serial port does not exist.\n";
            return 1;
        }
        cout << "some other error occurred.\n";
        return 1;
    }
    cout << "#connect ok\n";
    DCB dcbSerialParams = { 0 };
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(hSerial, &dcbSerialParams))
    {
        cout << "getting state error\n";
        return 1;
    }
    cout << "#get state ok\n";
    dcbSerialParams.BaudRate = 921600;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    if (!SetCommState(hSerial, &dcbSerialParams))
    {
        cout << "error setting serial port state\n";
        return 1;
    }
    // GKV Settings
    GKV->SetSendDataFunction(WriteCOM);//Set User Function That Sends Data to Serial Port connected to GKV
    GKV->SetReceiveDataFunction(ReadCOM);//Set User Function That Receives Data From Serial Port And Returns Received Byte
    GKV->SetADCDataReceivedCallback(ShowPacketData);//Set User Callback for Parsed ADC GKV Packet
    GKV->RunDevice();//Run Thread For Receiving Data From GKV
    while (!(algorithm_selected))
    {
        GKV->SetAlgorithm(ADC_CODES_ALGORITHM);
    }
    cout << "#start main loop\n";
    while (1)
    {
        //do something
    }
    return 0;
}

void WriteCOM(PacketBase* buf)
{
    DWORD dwBytesWritten;
    char iRet = WriteFile(hSerial, buf, buf->length + 8, &dwBytesWritten, NULL);
    Sleep(1);
}

char ReadCOM()
{
    DWORD iSize;
    char sReceivedChar;
    while (true)
    {
        ReadFile(hSerial, &sReceivedChar, 1, &iSize, 0);
        if (iSize > 0)
            return sReceivedChar;
    }
}

void ShowPacketData(ADCData* packet)
{
        char str[30];
        sprintf(str, "%d", packet->sample_cnt);
        cout << "Sample Counter = " << str << ' ';
        sprintf_s(str, "%d", packet->a[0]);
        cout << "ax = " << str << ' ';
        sprintf_s(str, "%d", packet->a[1]);
        cout << "ay = " << str << ' ';
        sprintf_s(str, "%d", packet->a[2]);
        cout << "az = " << str << ' ';
        sprintf_s(str, "%d", packet->w[0]);
        cout << "wx = " << str << ' ';
        sprintf_s(str, "%d", packet->w[1]);
        cout << "wy = " << str << ' ';
        sprintf_s(str, "%d", packet->w[2]);
        cout << "wz = " << str << endl;
        algorithm_selected = 1;
}