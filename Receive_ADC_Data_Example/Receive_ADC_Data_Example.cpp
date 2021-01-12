
#include <windows.h>
#include <iostream>
#include <stdio.h>
#include "LMP_Device.h"
using namespace std;

HANDLE hSerial;

char ReadCOM();
void WriteCOM(PacketBase* buf);
void RecognisePacket(PacketBase* buf);

int main()
{

    string com_port;
    cout << "Set Serial Port:";
    cin >> com_port;
    cout << "#start connecting to " << com_port << "\n";

    uint8_t Packet_is_Correct = 0;
    uint8_t algorithm = ADC_CODES_ALGORITHM;
    uint8_t algorithm_packet = GKV_ADC_CODES_PACKET;
    uint8_t algorithm_selected = 0;

    LMP_Device *GKV = new LMP_Device();
    GKV->SetSendDataFunction(WriteCOM);
    GKV->SetReceivedPacketProcessingFunction(RecognisePacket);
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
    const int MAX_INCORRECT_CNT = 1000;
    int incorrectCnt = 0;
    while (!(algorithm_selected))
    {
        GKV->Set_Algorithm(algorithm);
        Packet_is_Correct = 0;
        while (!(Packet_is_Correct))
        {
            Packet_is_Correct=GKV->Receive_Process(ReadCOM());
            incorrectCnt++;
            if (incorrectCnt > MAX_INCORRECT_CNT)
            {
                cout << "error too many incorrect packets\n";
                return 1;
            }
        }
        if (GKV->GetInputPacketType() == algorithm_packet)
        {
            algorithm_selected = 1;
        }
    }
    cout << "#start read loop\n";
    while (1)
    {
        GKV->Receive_Process(ReadCOM());
    }
    return 0;
}


void WriteCOM(PacketBase* buf)
{
    DWORD dwBytesWritten;
    char iRet = WriteFile(hSerial, buf, buf->length + 8, &dwBytesWritten, NULL);
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

void RecognisePacket(PacketBase* buf)
{
    char str[30];

    switch (buf->type)
    {
    case GKV_ADC_CODES_PACKET:
    {
        ADCData* packet;
        packet = (ADCData*)&buf->data;
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
        break;
    }
    case GKV_RAW_DATA_PACKET:
    {
        RawData* packet;
        packet = (RawData*)&buf->data;
        sprintf_s(str, "%d", packet->sample_cnt);
        cout << "Sample Counter = " << str << ' ';
        sprintf_s(str, "%f", packet->a[0]);
        cout << "ax = " << str << ' ';
        sprintf_s(str, "%f", packet->a[1]);
        cout << "ay = " << str << ' ';
        sprintf_s(str, "%f", packet->a[2]);
        cout << "az = " << str << ' ';
        sprintf_s(str, "%f", packet->w[0]);
        cout << "wx = " << str << ' ';
        sprintf_s(str, "%f", packet->w[1]);
        cout << "wy = " << str << ' ';
        sprintf_s(str, "%f", packet->w[2]);
        cout << "wz = " << str << endl;
        break;
    }
    case GKV_CUSTOM_PACKET:
    {
        CustomData* packet;
        packet = (CustomData*)&buf->data;
        cout << "CustomPacket: ";
        for (uint8_t i = 0; i < ((buf->length) / 4); i++)
        {
            if (packet->parameter[i] == packet->parameter[i])// проверка на isnan
            {
                sprintf_s(str, "%f", (packet->parameter[i]));
                cout << "param = " << str << ' ';
            }
            else
            {
                cout << "param = NaN ";
            }

        }
        cout << endl;
        break;
    }
    // Примечание: в данном примере вывод значений некоторых параметров наборного пакета с пометкой int будет некорректен, поскольку данная программа  
    // не посылает запроса на получение номеров парамеров наборного пакета и выводит все параметры, как float.
    }
}