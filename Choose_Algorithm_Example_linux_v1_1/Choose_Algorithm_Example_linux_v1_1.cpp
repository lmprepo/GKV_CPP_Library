#ifdef __linux
#include <iostream>
#include <stdio.h>
#include "GKV_Device.h"
using namespace Gyrovert;
using namespace std;

string input;
uint8_t algorithm = GKV_ADC_CODES_ALGORITHM;
uint8_t algorithm_packet = 0;
uint8_t algorithm_selected = 0;

void RecognisePacket(GKV_PacketBase* buf);
uint8_t check_input(string str);
uint8_t ChooseAlgorithmPacket(uint8_t algorithm);

int main()
{
    /* Select Serial Port */
    string com_port;
    cout << "Set Serial Port:";
    cin >> com_port;
    cout << "#start connecting to " << com_port << "\n";
    /* Create LMP Device Object GKV */
    GKV_Device* GKV = new GKV_Device(com_port, B921600);
    if (!(GKV->GetSerialConnectionState())) return 0;
    /* GKV Settings */
    GKV->SetReceivedPacketCallback(RecognisePacket);//Set User Callback for Each Parsed GKV Packet
    GKV->RunDevice();//Run Thread For Receiving Data From GKV  

    printf("Choose GKV algorithm:\n");
    printf("0 - ADC Codes Data from Sensors\n");
    printf("1 - Calibrated Raw Sensor Data\n");
    printf("2 - Orientation Kalman Filtered Data\n");
    printf("4 - Inclinometer Data\n");
    printf("5 - Orientation Mahony Filtered Data\n");
    printf("6 - BINS Navigation Data\n");
    printf("7 - Custom Algorithm\n");
    printf("8 - Navigation Data GNSS+BINS\n");
    printf("9 - Navigation Data GNSS+BINS type 2\n");
    /* Select Algorithm Number and Check It */
    cout << "Selected Algorithm = ";
    do
    {
        cin >> input;
        if (check_input(input))
        {
            cout << "Selected Algorithm = " << input;
            algorithm = atoi(input.c_str());
        }
        else
        {
            printf("Wrong value. Try Again. Selected Algorithm = ");
            algorithm = 255;
        }
    } while (!(check_input(input)));
    algorithm_packet = ChooseAlgorithmPacket(algorithm);

    while (!(algorithm_selected))
    {
        GKV->SetDefaultAlgorithmPacket();
        GKV->SetAlgorithm(algorithm);
    }
    GKV->RequestDeviceID();

    while (1)
    {
        //do something
    }
    return 0;
}

uint8_t check_input(string str)
{
    if (!(str.length() == 1))
    {
        return 0;
    }
    char char_array[2];
    strcpy(char_array, str.c_str());
    if (!(isdigit(char_array[0])))
    {
        return 0;
    }
    if ((atoi(str.c_str()) > 9) || (atoi(str.c_str()) == 3))
    {
        return 0;
    }
    return 1;
}

uint8_t ChooseAlgorithmPacket(uint8_t algorithm)
{
    switch (algorithm)
    {
    case GKV_ADC_CODES_ALGORITHM:
    {
        return GKV_ADC_CODES_PACKET;
        break;
    }
    case GKV_SENSORS_DATA_ALGORITHM:
    {
        return GKV_RAW_DATA_PACKET;
        break;
    }
    case GKV_ORIENTATION_KALMAN_ALGORITHM:
    {
        return GKV_EULER_ANGLES_PACKET;
        break;
    }
    case GKV_INCLINOMETER_ALGORITHM:
    {
        return GKV_INCLINOMETER_PACKET;
        break;
    }
    case GKV_ORIENTATION_MAHONY_ALGORITHM:
    {
        return GKV_EULER_ANGLES_PACKET;
        break;
    }
    case GKV_BINS_NAVIGATON_ALGORITHM:
    {
        return GKV_BINS_PACKET;
        break;
    }
    case GKV_CUSTOM_ALGORITHM:
    {
        return GKV_ADC_CODES_PACKET;
        break;
    }
    case GKV_KALMAN_GNSS_NAVIGATON_ALGORITHM:
    {
        return GKV_GNSS_PACKET;
        break;
    }
    case GKV_ESKF5_NAVIGATON_ALGORITHM:
    {
        return GKV_GNSS_PACKET;
        break;
    }
    default:
        return GKV_ADC_CODES_PACKET;
        break;
    }
}

void RecognisePacket(GKV_PacketBase* buf)
{
    char str[30];
    if (algorithm_selected)
    {
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
            sprintf(str, "%f", packet->time);
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
                if (packet->parameter[i] == packet->parameter[i])// проверка на isnan
                {
                    sprintf(str, "%f", (packet->parameter[i]));
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
        case GKV_DEV_ID_PACKET:
        {
            cout << "ID Packet: " << endl;
            break;
        }
        //Примечание: в данном примере вывод значений некоторых параметров наборного пакета с пометкой int будет некорректен, поскольку данная программа
        //не посылает запроса на получение номеров парамеров наборного пакета и выводит все параметры, как float.
        }
    }
    else
    {
        if (buf->type == algorithm_packet) algorithm_selected = 1;
    }
}
#else
int main()
{
}
#endif
