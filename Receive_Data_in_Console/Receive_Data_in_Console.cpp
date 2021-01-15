
#include <windows.h>
#include <iostream>
#include <stdio.h>
#include "LMP_Device.h"
using namespace std;

HANDLE hSerial;

char ReadCOM();
void WriteCOM(PacketBase* buf);
void RecognisePacket(PacketBase* buf);
void dataNewThreadReceiveFcn();

LMP_Device* GKV = new LMP_Device();


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
    GKV->SetReceivedPacketCallback(RecognisePacket);
    GKV->SetReceiveDataFunction(ReadCOM);
    GKV->SetSendDataFunction(WriteCOM);


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
    cout << "#start main loop\n";
    while (1)
    {
        Sleep(100);
        cout << "#do something\n";
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
    char iRet = 0;
    while (true)
    {

        iRet = ReadFile(hSerial, &sReceivedChar, 1, &iSize, 0);
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
            cout << "ADC Data Packet: ";
            sprintf_s(str, "%d", packet->sample_cnt);
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
            cout << "Raw Sensors Data Packet: ";
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
        case GKV_EULER_ANGLES_PACKET:
        {
            GyrovertData* packet;
            packet = (GyrovertData*)&buf->data;
            cout << "Gyrovert Data Packet: ";
            sprintf_s(str, "%d", packet->sample_cnt);
            cout << "Sample Counter = " << str << ' ';
            sprintf_s(str, "%f", packet->yaw);
            cout << "yaw = " << str << ' ';
            sprintf_s(str, "%f", packet->pitch);
            cout << "pitch = " << str << ' ';
            sprintf_s(str, "%f", packet->roll);
            cout << "roll = " << str << endl;
            break;
        }
        case GKV_INCLINOMETER_PACKET:
        {
            InclinometerData* packet;
            packet = (InclinometerData*)&buf->data;
            sprintf_s(str, "%d", packet->sample_cnt);
            cout << "Sample Counter = " << str << ' ';
            sprintf_s(str, "%f", packet->alfa);
            cout << "alfa = " << str << ' ';
            sprintf_s(str, "%f", packet->beta);
            cout << "beta = " << str << endl;
            break;
        }
        case GKV_BINS_PACKET:
        {
            BINSData* packet;
            packet = (BINSData*)&buf->data;
            cout << "BINS Data Packet: ";
            sprintf_s(str, "%d", packet->sample_cnt);
            cout << "Sample Counter = " << str << ' ';
            sprintf_s(str, "%f", packet->x);
            cout << "x = " << str << ' ';
            sprintf_s(str, "%f", packet->y);
            cout << "y = " << str << ' ';
            sprintf_s(str, "%f", packet->z);
            cout << "z = " << str << ' ';
            sprintf_s(str, "%f", packet->alfa);
            cout << "alfa = " << str << ' ';
            sprintf_s(str, "%f", packet->beta);
            cout << "beta = " << str << ' ';
            sprintf_s(str, "%f", packet->q[0]);
            cout << "q0 = " << str << ' ';
            sprintf_s(str, "%f", packet->q[1]);
            cout << "q1 = " << str << ' ';
            sprintf_s(str, "%f", packet->q[2]);
            cout << "q2 = " << str << ' ';
            sprintf_s(str, "%f", packet->q[3]);
            cout << "q3 = " << str << ' ';
            sprintf_s(str, "%f", packet->yaw);
            cout << "yaw = " << str << ' ';
            sprintf_s(str, "%f", packet->pitch);
            cout << "pitch = " << str << ' ';
            sprintf_s(str, "%f", packet->roll);
            cout << "roll = " << str << endl;
            break;
        }
        case GKV_GNSS_PACKET:
        {
            GpsData* packet;
            packet = (GpsData*)&buf->data;
            cout << "GNSS Data Packet: ";
            sprintf_s(str, "%f", packet->time);
            cout << "time = " << str << ' ';
            sprintf_s(str, "%f", packet->latitude);
            cout << "latitude = " << str << ' ';
            sprintf_s(str, "%f", packet->longitude);
            cout << "longitude = " << str << ' ';
            sprintf_s(str, "%f", packet->altitude);
            cout << "altitude = " << str << ' ';
            sprintf_s(str, "%d", packet->state_status);
            cout << "state_status = " << str << ' ';
            sprintf_s(str, "%f", packet->TDOP);
            cout << "TDOP = " << str << ' ';
            sprintf_s(str, "%f", packet->HDOP);
            cout << "HDOP = " << str << ' ';
            sprintf_s(str, "%f", packet->VDOP);
            cout << "VDOP = " << str << endl;
            break;
        }
        case GKV_CUSTOM_PACKET:
        {
            CustomData* packet;
            packet = (CustomData*)&buf->data;
            cout << "CustomPacket: ";
            for (uint8_t i = 0; i < ((buf->length)/4); i++)
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