﻿#ifdef _WIN32
#include <windows.h>
#include <iostream>
#include <stdio.h>
#include "LMP_Device.h"

using namespace Gyrovert;

using namespace std;

HANDLE hSerial;
char ReceivedData=0;

bool InitSerialPort(string port_name, int32_t baudrate);
char* ReadCOM();
void WriteCOM(GKV_PacketBase* buf);
void RecognisePacket(LMP_Device * GKV,GKV_PacketBase* buf);

int main()
{
    /* Select Serial Port */
    string com_port;
    cout << "Set Serial Port:";
    cin >> com_port;
    cout << "#start connecting to " << com_port << "\n";
    /* Create LMP Device Object GKV */
    LMP_Device* GKV = new LMP_Device();
    /* Serial Port Settings For Windows */
    com_port = "\\\\.\\" + com_port;
    if (!(InitSerialPort(com_port, 921600))) return 1;
    /* GKV Settings */
    GKV->SetReceivedPacketCallback(RecognisePacket);//Set User Callback for Each Parsed GKV Packet
    GKV->SetReceiveDataFunction(ReadCOM);//Set User Function That Receives Data From Serial Port And Returns Received Byte
    GKV->SetSendDataFunction(WriteCOM);//Set User Function That Sends Data to Serial Port connected to GKV
    GKV->RunDevice();//Run Thread For Receiving Data From GKV
    GKV->SetCANPortMsg(IfProtoConfig::IFACE_CAN1, IfCanMessage::INDEX_STATUS_CNT, 10, 0x10);
    cout << "#start main loop\n";
    while (1)
    {
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
    DWORD iSize;
    char iRet = 0;
    while (true)
    {
        iRet = ReadFile(hSerial, &ReceivedData, sizeof(ReceivedData), &iSize, 0);
        if (iSize > 0)
            return &ReceivedData;
    }
    return 0;
}

void RecognisePacket(LMP_Device *GKV, GKV_PacketBase* buf)
{
    char str[30];
    switch (buf->type)
    {
    case GKV_ADC_CODES_PACKET:
    {
        GKV_ADCData* packet;
        packet = (GKV_ADCData*)&buf->data;
        cout << "ADC Data Packet: ";
        sprintf(str, "%d", packet->sample_cnt);
        cout << "Sample Counter = " << str << ' ';
        sprintf(str, "%d", packet->a[0]);
        cout << "ax = " << str << ' ';
        sprintf(str, "%d", packet->a[1]);
        cout << "ay = " << str << ' ';
        sprintf(str, "%d", packet->a[2]);
        cout << "az = " << str << ' ';
        sprintf(str, "%d", packet->w[0]);
        cout << "wx = " << str << ' ';
        sprintf(str, "%d", packet->w[1]);
        cout << "wy = " << str << ' ';
        sprintf(str, "%d", packet->w[2]);
        cout << "wz = " << str << endl;
        break;
    }
    case GKV_RAW_DATA_PACKET:
    {
        GKV_RawData* packet;
        packet = (GKV_RawData*)&buf->data;
        cout << "Raw Sensors Data Packet: ";
        sprintf(str, "%d", packet->sample_cnt);
        cout << "Sample Counter = " << str << ' ';
        sprintf(str, "%f", packet->a[0]);
        cout << "ax = " << str << ' ';
        sprintf(str, "%f", packet->a[1]);
        cout << "ay = " << str << ' ';
        sprintf(str, "%f", packet->a[2]);
        cout << "az = " << str << ' ';
        sprintf(str, "%f", packet->w[0]);
        cout << "wx = " << str << ' ';
        sprintf(str, "%f", packet->w[1]);
        cout << "wy = " << str << ' ';
        sprintf(str, "%f", packet->w[2]);
        cout << "wz = " << str << endl;
        break;
    }
    case GKV_EULER_ANGLES_PACKET:
    {
        GKV_GyrovertData* packet;
        packet = (GKV_GyrovertData*)&buf->data;
        cout << "Gyrovert Data Packet: ";
        sprintf(str, "%d", packet->sample_cnt);
        cout << "Sample Counter = " << str << ' ';
        sprintf(str, "%f", packet->yaw);
        cout << "yaw = " << str << ' ';
        sprintf(str, "%f", packet->pitch);
        cout << "pitch = " << str << ' ';
        sprintf(str, "%f", packet->roll);
        cout << "roll = " << str << endl;
        break;
    }
    case GKV_INCLINOMETER_PACKET:
    {
        GKV_InclinometerData* packet;
        packet = (GKV_InclinometerData*)&buf->data;
        sprintf(str, "%d", packet->sample_cnt);
        cout << "Sample Counter = " << str << ' ';
        sprintf(str, "%f", packet->alfa);
        cout << "alfa = " << str << ' ';
        sprintf(str, "%f", packet->beta);
        cout << "beta = " << str << endl;
        break;
    }
    case GKV_BINS_PACKET:
    {
        GKV_BINSData* packet;
        packet = (GKV_BINSData*)&buf->data;
        cout << "BINS Data Packet: ";
        sprintf(str, "%d", packet->sample_cnt);
        cout << "Sample Counter = " << str << ' ';
        sprintf(str, "%f", packet->x);
        cout << "x = " << str << ' ';
        sprintf(str, "%f", packet->y);
        cout << "y = " << str << ' ';
        sprintf(str, "%f", packet->z);
        cout << "z = " << str << ' ';
        sprintf(str, "%f", packet->alfa);
        cout << "alfa = " << str << ' ';
        sprintf(str, "%f", packet->beta);
        cout << "beta = " << str << ' ';
        sprintf(str, "%f", packet->q[0]);
        cout << "q0 = " << str << ' ';
        sprintf(str, "%f", packet->q[1]);
        cout << "q1 = " << str << ' ';
        sprintf(str, "%f", packet->q[2]);
        cout << "q2 = " << str << ' ';
        sprintf(str, "%f", packet->q[3]);
        cout << "q3 = " << str << ' ';
        sprintf(str, "%f", packet->yaw);
        cout << "yaw = " << str << ' ';
        sprintf(str, "%f", packet->pitch);
        cout << "pitch = " << str << ' ';
        sprintf(str, "%f", packet->roll);
        cout << "roll = " << str << endl;
        break;
    }
    case GKV_GNSS_PACKET:
    {
        GKV_GpsData* packet;
        packet = (GKV_GpsData*)&buf->data;
        cout << "GNSS Data Packet: ";
        sprintf(str, "%d", packet->time);
        cout << "time = " << str << ' ';
        sprintf(str, "%f", packet->latitude);
        cout << "latitude = " << str << ' ';
        sprintf(str, "%f", packet->longitude);
        cout << "longitude = " << str << ' ';
        sprintf(str, "%f", packet->altitude);
        cout << "altitude = " << str << ' ';
        sprintf(str, "%d", packet->state_status);
        cout << "state_status = " << str << ' ';
        sprintf(str, "%f", packet->TDOP);
        cout << "TDOP = " << str << ' ';
        sprintf(str, "%f", packet->HDOP);
        cout << "HDOP = " << str << ' ';
        sprintf(str, "%f", packet->VDOP);
        cout << "VDOP = " << str << endl;
        break;
    }
    case GKV_EXTENDED_GNSS_PACKET:
    {
        GKV_GpsDataExt* packet;
        packet = (GKV_GpsDataExt*)&buf->data;
        cout << "Extended GNSS Data Packet: ";
        sprintf(str, "%f", packet->vlat);
        cout << "vlat = " << str << ' ';
        sprintf(str, "%f", packet->vlon);
        cout << "vlon = " << str << ' ';
        sprintf(str, "%f", packet->sig_lat);
        cout << "sig_lat = " << str << ' ';
        sprintf(str, "%f", packet->sig_lon);
        cout << "sig_lon = " << str << ' ';
        sprintf(str, "%f", packet->sig_alt);
        cout << "sig_alt = " << str << ' ';
        sprintf(str, "%f", packet->sig_vlat);
        cout << "sig_vlat = " << str << ' ';
        sprintf(str, "%f", packet->sig_vlon);
        cout << "sig_vlon = " << str << ' ';
        sprintf(str, "%f", packet->sig_valt);
        cout << "sig_valt = " << str << endl;
        break;
    }
    case GKV_CUSTOM_PACKET:
    {
        GKV_CustomData* packet;
        packet = (GKV_CustomData*)&buf->data;
        cout << "CustomPacket: ";
        for (uint8_t i = 0; i < ((buf->length) / 4); i++)
        {
            if (GKV->IsCustomPacketParamReceived())//if custom parameters list received
            {
                uint8_t CurrentParameterType = FloatParameter;
                for (uint8_t j = 0; j < sizeof(GKV->INT_PARAM_NUMBERS); j++)
                {
                    if (GKV->DeviceState.CurrentCustomPacketParameters.param[i] == GKV->INT_PARAM_NUMBERS[j])
                    {
                        CurrentParameterType = Int32Parameter;
                        break;
                    }
                }
                for (uint8_t j = 0; j < sizeof(GKV->UINT_PARAM_NUMBERS); j++)
                {
                    if (GKV->DeviceState.CurrentCustomPacketParameters.param[i] == GKV->UINT_PARAM_NUMBERS[j])
                    {
                        CurrentParameterType = Uint32Parameter;
                        break;
                    }
                }
                if (CurrentParameterType == FloatParameter)
                {
                    cout << "param = " << (packet->parameter[i]) << ' ';
                }
                else if (CurrentParameterType == Int32Parameter)
                {
                    cout << "param = " << *(int32_t*)&(packet->parameter[i]) << ' ';
                }
                else
                {
                    cout << "param = " << *(uint32_t*)&(packet->parameter[i]) << ' ';
                }
            }
        }
        cout << endl;
        break;
    }
    //Примечание: в данном примере вывод значений некоторых параметров наборного пакета с пометкой int будет некорректен, поскольку данная программа
    //не посылает запроса на получение номеров парамеров наборного пакета и выводит все параметры, как float.
    }
}
#else
int main()
{
}
#endif
