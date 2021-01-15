
#include <windows.h>
#include <iostream>
#include <stdio.h>
#include "LMP_Device.h"
using namespace std;

HANDLE hSerial;

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
    uint8_t algorithm_selected = 0;

    LMP_Device *GKV = new LMP_Device();
    GKV->SetSendDataFunction(WriteCOM);
    GKV->SetReceiveDataFunction(ReadCOM);
    GKV->SetADCDataReceivedCallback(ShowPacketData);
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
    GKV->RunDevice();
    while (!(algorithm_selected))
    {
        GKV->SetAlgorithm(ADC_CODES_ALGORITHM);
        if (GKV->GetInputPacketType() == algorithm_packet)
        {
            algorithm_selected = 1;
        }
    }
    cout << "#start main loop\n";
    while (1)
    {
        cout << "#do something\n";
        Sleep(100);
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
}