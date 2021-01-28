#ifdef __linux
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <iostream>
#include <stdio.h>
#include "LMP_Device.h"
using namespace std;

int SerialPortHandle;
uint8_t algorithm_selected = 0;

bool InitSerialPort(string port_name, int32_t baudrate);
char ReadCOM();
void WriteCOM(PacketBase* buf);
void ShowPacketData(ADCData* buf);

int main()
{
    string com_port;
    cout << "Set Serial Port:";
    cin >> com_port;
    cout << "#start connecting to " << com_port << "\n";
    /* Create LMP Device Object GKV */
    LMP_Device *GKV = new LMP_Device();
    /* Serial Port Settings For Linux */
    if (!(InitSerialPort(com_port, B921600))) return 1;
    /* GKV Settings */
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
        std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
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
        std::cout << "Error " << errno << " from tcsetattr" << std::endl;
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

void ShowPacketData(ADCData* packet)
{
        char str[30];
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
        algorithm_selected = 1;
}
#else
int main()
{
}
#endif
