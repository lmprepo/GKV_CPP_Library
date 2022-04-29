#ifdef _WIN32
#include <windows.h>
#include <iostream>
#include <stdio.h>
#include "LMP_Device.h"

using namespace Gyrovert;

using namespace std;

HANDLE hSerial;
OVERLAPPED o;
DWORD dwEvtMask;

LMP_Device* GKV;

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
    GKV = new LMP_Device();
    /* Serial Port Settings For Windows */
    com_port = "\\\\.\\" + com_port;
    if (!(InitSerialPort(com_port, 921600))) return 1;
    /* GKV Settings */
    GKV->SetReceivedPacketCallback(RecognisePacket);//Set User Callback for Each Parsed GKV Packet
    GKV->SetReceiveDataFunction(ReadCOM);//Set User Function That Receives Data From Serial Port And Returns Received Byte
    GKV->SetSendDataFunction(WriteCOM);//Set User Function That Sends Data to Serial Port connected to GKV
    GKV->RunDevice();//Run Thread For Receiving Data From GKV
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
    static char ReceivedData[2048] = {0};
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

void RecognisePacket(LMP_Device *GKV, GKV_PacketBase* buf)
{
std::string output_str;
switch (buf->type)
    {
    case GKV_ADC_CODES_PACKET:
    {
        GKV_ADCData* packet;
        packet = (GKV_ADCData*)&buf->data;
        output_str.append("ADC Data Packet: ");
        output_str.append("Sample Counter = " + std::to_string(packet->sample_cnt));
        output_str.append(" ax = " + std::to_string((int32_t)packet->a[0]));
        output_str.append(" ay = " + std::to_string((int32_t)packet->a[1]));
        output_str.append(" az = " + std::to_string((int32_t)packet->a[2]));
        output_str.append(" wx = " + std::to_string((int32_t)packet->w[0]));
        output_str.append(" wy = " + std::to_string((int32_t)packet->w[1]));
        output_str.append(" wz = " + std::to_string((int32_t)packet->w[2]));
        cout << output_str << endl;
        break;
    }
    case GKV_RAW_DATA_PACKET:
    {
        GKV_RawData* packet;
        packet = (GKV_RawData*)&buf->data;
        output_str.append( "Raw Sensors Data Packet: ");
        output_str.append("Sample Counter = " + std::to_string(packet->sample_cnt));
        output_str.append(" ax = " + std::to_string(packet->a[0]));
        output_str.append(" ay = " + std::to_string(packet->a[1]));
        output_str.append(" az = " + std::to_string(packet->a[2]));
        output_str.append(" wx = " + std::to_string(packet->w[0]));
        output_str.append(" wy = " + std::to_string(packet->w[1]));
        output_str.append(" wz = " + std::to_string(packet->w[2]));
        cout   << output_str << endl;

        break;
    }
    case GKV_EULER_ANGLES_PACKET:
    {
        GKV_GyrovertData* packet;
        packet = (GKV_GyrovertData*)&buf->data;
        output_str.append("Gyrovert Data Packet: ");
        output_str.append("Sample Counter = " + std::to_string(packet->sample_cnt));
        output_str.append(" yaw = " + std::to_string(packet->yaw));
        output_str.append(" pitch = " + std::to_string(packet->pitch));
        output_str.append(" roll = " + std::to_string(packet->roll));
        cout << output_str << endl;
        break;
    }
    case GKV_INCLINOMETER_PACKET:
    {
        GKV_InclinometerData* packet;
        packet = (GKV_InclinometerData*)&buf->data;
        output_str.append("Inclinometer Data Packet: ");
        output_str.append("Sample Counter = " + std::to_string(packet->sample_cnt));
        output_str.append(" alfa = " + std::to_string(packet->alfa));
        output_str.append(" beta = " + std::to_string(packet->beta));
        cout << output_str << endl;
        break;
    }
    case GKV_BINS_PACKET:
    {
        GKV_BINSData* packet;
        packet = (GKV_BINSData*)&buf->data;
        output_str.append("BINS Data Packet: ");
        output_str.append("Sample Counter = " + std::to_string(packet->sample_cnt));
        output_str.append(" x = " + std::to_string(packet->x));
        output_str.append(" y = " + std::to_string(packet->y));
        output_str.append(" z = " + std::to_string(packet->z));
        output_str.append(" alfa = " + std::to_string(packet->alfa));
        output_str.append(" beta = " + std::to_string(packet->beta));
        output_str.append(" q0 = " + std::to_string(packet->q[0]));
        output_str.append(" q1 = " + std::to_string(packet->q[1]));
        output_str.append(" q2 = " + std::to_string(packet->q[2]));
        output_str.append(" q3 = " + std::to_string(packet->q[3]));
        output_str.append(" yaw = " + std::to_string(packet->yaw));
        output_str.append(" pitch = " + std::to_string(packet->pitch));
        output_str.append(" roll = " + std::to_string(packet->roll));
        cout << output_str << endl;
        break;
    }
    case GKV_GNSS_PACKET:
    {
        GKV_GpsData* packet;
        packet = (GKV_GpsData*)&buf->data;
        output_str.append("GNSS Data Packet: ");
        output_str.append("time = " + std::to_string(packet->time));
        output_str.append(" latitude = " + std::to_string(packet->latitude));
        output_str.append(" longitude = " + std::to_string(packet->longitude));
        output_str.append(" altitude = " + std::to_string(packet->altitude));
        output_str.append(" state_status = " + std::to_string(packet->state_status));
        output_str.append(" TDOP = " + std::to_string(packet->TDOP));
        output_str.append(" HDOP = " + std::to_string(packet->HDOP));
        output_str.append(" VDOP = " + std::to_string(packet->VDOP));
        cout << output_str << endl;
        break;
    }
    case GKV_EXTENDED_GNSS_PACKET:
    {
        GKV_GpsDataExt* packet;
        packet = (GKV_GpsDataExt*)&buf->data;
        output_str.append("Extended GNSS Data Packet: ");
        output_str.append("vlat = " + std::to_string(packet->vlat));
        output_str.append(" vlon = " + std::to_string(packet->vlon));
        output_str.append(" sig_lat = " + std::to_string(packet->sig_lat));
        output_str.append(" sig_lon = " + std::to_string(packet->sig_lon));
        output_str.append(" sig_alt = " + std::to_string(packet->sig_alt));
        output_str.append(" sig_vlat = " + std::to_string(packet->sig_vlat));
        output_str.append(" sig_vlon = " + std::to_string(packet->sig_vlon));
        output_str.append(" sig_valt = " + std::to_string(packet->sig_valt));
        cout << output_str << endl;
        break;
    }
    case GKV_CUSTOM_PACKET:
    {
        GKV_CustomData* packet;
        packet = (GKV_CustomData*)&buf->data;
        output_str.append("CustomPacket:");
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
                    output_str.append(" param = " + std::to_string(packet->parameter[i]));
                }
                else if (CurrentParameterType == Int32Parameter)
                {
                    output_str.append(" param = " + std::to_string((int32_t) * (int32_t*)&(packet->parameter[i])));
                }
                else
                {
                    output_str.append(" param = " + std::to_string((uint32_t) * (uint32_t*)&(packet->parameter[i])));
                }
            }
        }
        cout << output_str << endl;
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
