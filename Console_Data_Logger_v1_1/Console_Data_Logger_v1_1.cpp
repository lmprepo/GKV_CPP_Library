#ifdef _WIN32  
#include <string>
#include <iostream>
#include <stdio.h>
#include "GKV_Device.h"
#include <filesystem>
using namespace Gyrovert;
using namespace std;


int main()
{
    string com_port;
    cout << "Set Serial Port:";
    cin >> com_port;
    cout << "#start connecting to " << com_port << "\n";
    /* Create GKV Device Object GKV */
    GKV_Device* GKV = new GKV_Device(com_port, 921600);
    if (!(GKV->GetSerialConnectionState())) return 0;
    /* GKV Settings */
    GKV->RunDevice();//Run Thread For Receiving Data From GKV
    GKV->StartWriteBinaryData();
    cout << "#start main loop\n";
    cout << "Writing data to " << std::filesystem::current_path() << '\n';    /* Show data writing path */

    while (1)
    {
        //do something
    }
    GKV->StopWriteBinaryData();
    return 0;
}

#else
int main()
{
}
#endif
