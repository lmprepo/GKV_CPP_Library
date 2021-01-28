#ifdef __linux
#include <stdlib.h>
#include <unistd.h>     
#include <fcntl.h>      
#include <termios.h>    
#include <iostream>
#include <stdio.h>
#include "LMP_Device.h"
using namespace Gyrovert;
using namespace std;

int SerialPortHandle;
string input;
uint8_t algorithm = ADC_CODES_ALGORITHM;
uint8_t algorithm_packet = 0;
uint8_t algorithm_selected = 0;

bool InitSerialPort(string port_name, int32_t baudrate);
char ReadCOM();
void WriteCOM(PacketBase* buf);
void RecognisePacket(PacketBase* buf);
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
    LMP_Device* GKV = new LMP_Device();
    /* Serial Port Settings For Linux */
    if (!(InitSerialPort(com_port, B921600))) return 1;
    /* GKV Settings */
    GKV->SetReceivedPacketCallback(RecognisePacket);//Set User Callback for Each Parsed GKV Packet
    GKV->SetReceiveDataFunction(ReadCOM);//Set User Function That Receives Data From Serial Port And Returns Received Byte
    GKV->SetSendDataFunction(WriteCOM);//Set User Function That Sends Data to Serial Port connected to GKV
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
        GKV->SetAlgorithm(algorithm);
    }
    while (1)
    {
        //do something
    }
    return 0;
}

bool InitSerialPort(string port_name, int32_t baudrate)
{
    SerialPortHandle = open(port_name.c_str(), O_RDWR | O_NOCTTY);
    if (SerialPortHandle < 0) {
        printf("Error opening port\n");
        return 0;
    }
    struct termios tty;
    struct termios tty_old;
    memset(&tty, 0, sizeof tty);
    /* Error Handling */
    if (tcgetattr(SerialPortHandle, &tty) != 0) {
        cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << endl;
        return 0;
    }
    /* Save old tty parameters */
    tty_old = tty;
    /* Set Baud Rate */
    cfsetospeed(&tty, (speed_t)baudrate);
    cfsetispeed(&tty, (speed_t)baudrate);
    /* Setting other Port Stuff */
    tty.c_cflag &= ~PARENB;            // Make 8n1
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cflag &= ~CRTSCTS;           // no flow control
    tty.c_cc[VMIN] = 1;                  // read doesn't block
    tty.c_cc[VTIME] = 5;                  // 0.5 seconds read timeout
    tty.c_cflag |= CREAD | CLOCAL;     // turn on READ & ignore ctrl lines
    /* Make raw */
    cfmakeraw(&tty);
    /* Flush Port, then applies attributes */
    tcflush(SerialPortHandle, TCIFLUSH);
    if (tcsetattr(SerialPortHandle, TCSANOW, &tty) != 0) {
        cout << "Error " << errno << " from tcsetattr" << endl;
        return 0;
    }
    return 1;
}

void WriteCOM(PacketBase* buf)
{
    int iOut = write(SerialPortHandle, buf, buf->length + 8);
    usleep(1000);
}

char ReadCOM()
{
    char sReceivedChar;
    while (true)
    {
        int iOut = read(SerialPortHandle, &sReceivedChar, 1);
        return sReceivedChar;
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
    case ADC_CODES_ALGORITHM:
    {
        return GKV_ADC_CODES_PACKET;
        break;
    }
    case SENSORS_DATA_ALGORITHM:
    {
        return GKV_RAW_DATA_PACKET;
        break;
    }
    case ORIENTATION_KALMAN_ALGORITHM:
    {
        return GKV_EULER_ANGLES_PACKET;
        break;
    }
    case INCLINOMETER_ALGORITHM:
    {
        return GKV_INCLINOMETER_PACKET;
        break;
    }
    case ORIENTATION_MAHONY_ALGORITHM:
    {
        return GKV_EULER_ANGLES_PACKET;
        break;
    }
    case BINS_NAVIGATON_ALGORITHM:
    {
        return GKV_BINS_PACKET;
        break;
    }
    case CUSTOM_ALGORITHM:
    {
        return GKV_ADC_CODES_PACKET;
        break;
    }
    case KALMAN_GNSS_NAVIGATON_ALGORITHM:
    {
        return GKV_GNSS_PACKET;
        break;
    }
    case ESKF5_NAVIGATON_ALGORITHM:
    {
        return GKV_GNSS_PACKET;
        break;
    }
    default:
        return GKV_ADC_CODES_PACKET;
        break;
    }
}

void RecognisePacket(PacketBase* buf)
{
    char str[30];
    if (algorithm_selected)
    {
        switch (buf->type)
        {
        case GKV_ADC_CODES_PACKET:
        {
            ADCData* packet;
            packet = (ADCData*)&buf->data;
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
            RawData* packet;
            packet = (RawData*)&buf->data;
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
            GyrovertData* packet;
            packet = (GyrovertData*)&buf->data;
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
            InclinometerData* packet;
            packet = (InclinometerData*)&buf->data;
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
            BINSData* packet;
            packet = (BINSData*)&buf->data;
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
            GpsData* packet;
            packet = (GpsData*)&buf->data;
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
        case GKV_CUSTOM_PACKET:
        {
            CustomData* packet;
            packet = (CustomData*)&buf->data;
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
