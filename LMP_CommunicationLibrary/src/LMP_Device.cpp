#include "LMP_Device.h"
#include <string.h>

/**
  * @name	LMP_Device
  * @brief  Default Constructor
  * @retval no return value.
  */
LMP_Device::LMP_Device()
{
}

/**
  * @name	~LMP_Device
  * @brief  Default Destructor
  * @retval no return value.
  */
LMP_Device::~LMP_Device()
{
	gkv_open = false;
}

/**
  * @name	SetSendDataFunction
  * @brief  Function sets pointer on user function for seril data transmition from PC to LMP Device
  * @param  ptrSendPacketFun - pointer on void-type callback function that gets pointer on PacketBase structure and sends "length" fiels + 8 bytes
  * @retval no return value.
  */
void  LMP_Device::SetSendDataFunction(void(*ptrSendPacketFun)(PacketBase* Output_Packet_Ptr))
{
	ptrSendFun = ptrSendPacketFun;
}

/**
  * @name	SetReceiveDataFunction
  * @brief  Function sets pointer on user function for serial data receive to PC
  * @param  ptrRecPacketFun - pointer on char-type callback function that returns received byte from serial port
  * @retval no return value.
  */
void  LMP_Device::SetReceiveDataFunction(char(*ptrRecPacketFun)())
{
	ptrRecFcn = ptrRecPacketFun;
}

/**
  * @name	SetReceivedPacketCallback
  * @brief  Function sets pointer on user function for processing every received and parsed packet from LMP Device
  * @param  ptrReceivedPacketProcessingFun - pointer on void-type user callback function that gets pointer on received PacketBase structure
  * @retval no return value.
  */
void  LMP_Device::SetReceivedPacketCallback(void(*ptrReceivedPacketProcessingFun)(PacketBase* Input_Packet_Ptr))
{
	ptrPacketProcessingFun = ptrReceivedPacketProcessingFun;
}

/**
  * @name	SetSettingsReceivedCallback
  * @brief  Function sets pointer on user function for processing received and parsed settings packet (type 0x07) from LMP Device
  * @param  ptrSendPacketFun - pointer on void-type user callback function that gets pointer on received and parsed Settings structure
  * @retval no return value.
  */
void LMP_Device::SetSettingsReceivedCallback(void(*ptrReceivedPacketProcessingFun)(Settings* settings))
{
	ptrSettingsPacketCallback = ptrReceivedPacketProcessingFun;
}

/**
  * @name	SetCustomPacketParamReceivedCallback
  * @brief  Function sets pointer on user function for processing received and parsed custom parameters packet (type 0x27) from LMP Device
  * @param  ptrReceivedPacketProcessingFun - pointer on void-type user callback function that gets pointer on received and parsed custom data parameters structure
  * @retval no return value.
  */
void LMP_Device::SetCustomPacketParamReceivedCallback(void(*ptrReceivedPacketProcessingFun)(CustomDataParam* param))
{
	ptrCustomPacketParamCallback = ptrReceivedPacketProcessingFun;
}

/**
  * @name	SetCustomPacketParamReceivedCallback
  * @brief  Function sets pointer on user function for processing received and parsed custom packet (type 0x13) from LMP Device
  * @param  ptrReceivedPacketProcessingFun - pointer on void-type user callback function that gets pointer on received and parsed custom packet structure
  * @retval no return value.
  */
void LMP_Device::SetCustomPacketReceivedCallback(void(*ptrReceivedPacketProcessingFun)(CustomData* data))
{
	ptrCustomDataPacketCallback = ptrReceivedPacketProcessingFun;
}

/**
  * @name	SetADCDataReceivedCallback
  * @brief  Function sets pointer on user function for processing received and parsed ADC Codes packet (type 0x0A) from LMP Device
  * @param  ptrReceivedPacketProcessingFun - pointer on void-type user callback function that gets pointer on received and parsed ADC Codes structure
  * @retval no return value.
  */
void LMP_Device::SetADCDataReceivedCallback(void(*ptrReceivedPacketProcessingFun)(ADCData* data))
{
	ptrADCPacketCallback = ptrReceivedPacketProcessingFun;
}

/**
  * @name	SetRawDataReceivedCallback
  * @brief  Function sets pointer on user function for processing received and parsed Calibrated Sensors Data packet (type 0x0B) from LMP Device
  * @param  ptrReceivedPacketProcessingFun - pointer on void-type user callback function that gets pointer on received and parsed Calibrated Sensors Data structure
  * @retval no return value.
  */
void LMP_Device::SetRawDataReceivedCallback(void(*ptrReceivedPacketProcessingFun)(RawData* data))
{
	ptrRawDataPacketCallback = ptrReceivedPacketProcessingFun;
}

/**
  * @name	SetGyrovertDataReceivedCallback
  * @brief  Function sets pointer on user function for processing received and parsed Orientation Data packet (type 0x0C) from LMP Device
  * @param  ptrReceivedPacketProcessingFun - pointer on void-type user callback function that gets pointer on received and parsed Orientation Data structure
  * @retval no return value.
  */
void LMP_Device::SetGyrovertDataReceivedCallback(void(*ptrReceivedPacketProcessingFun)(GyrovertData* data))
{
	ptrGyrovertDataPacketCallback = ptrReceivedPacketProcessingFun;
}

/**
  * @name	SetInclinometerDataReceivedCallback
  * @brief  Function sets pointer on user function for processing received and parsed Inclinometer Data packet (type 0x0D) from LMP Device
  * @param  ptrReceivedPacketProcessingFun - pointer on void-type user callback function that gets pointer on received and parsed Inclinometer Data structure
  * @retval no return value.
  */
void LMP_Device::SetInclinometerDataReceivedCallback(void(*ptrReceivedPacketProcessingFun)(InclinometerData* data))
{
	ptrInclinometerDataPacketCallback = ptrReceivedPacketProcessingFun;
}

/**
  * @name	SetBINSDataReceivedCallback
  * @brief  Function sets pointer on user function for processing received and parsed BINS Data packet (type 0x12) from LMP Device
  * @param  ptrReceivedPacketProcessingFun - pointer on void-type user callback function that gets pointer on received and parsed BINS Data structure
  * @retval no return value.
  */
void LMP_Device::SetBINSDataReceivedCallback(void(*ptrReceivedPacketProcessingFun)(BINSData* data))
{
	ptrBINSDataPacketCallback = ptrReceivedPacketProcessingFun;
}

/**
  * @name	SetBINS2DataReceivedCallback
  * @brief  Function sets pointer on user function for processing received and parsed Extended BINS Data packet (type 0x14) from LMP Device
  * @param  ptrReceivedPacketProcessingFun - pointer on void-type user callback function that gets pointer on received and parsed Extended BINS Data structure
  * @retval no return value.
  */
void LMP_Device::SetBINS2DataReceivedCallback(void(*ptrReceivedPacketProcessingFun)(BINS2Data* data))
{
	ptrBINS2DataPacketCallback = ptrReceivedPacketProcessingFun;
}

/**
  * @name	SetGNSSDataReceivedCallback
  * @brief  Function sets pointer on user function for processing received and parsed GNSS Data packet (type 0x0E) from LMP Device
  * @param  ptrReceivedPacketProcessingFun - pointer on void-type user callback function that gets pointer on received and parsed GNSS Data structure
  * @retval no return value.
  */
void LMP_Device::SetGNSSDataReceivedCallback(void(*ptrReceivedPacketProcessingFun)(GpsData* data))
{
	ptrGNSSDataPacketCallback = ptrReceivedPacketProcessingFun;
}

/**
  * @name	SetExtGNSSDataReceivedCallback
  * @brief  Function sets pointer on user function for processing received and parsed Extended GNSS Data packet (type 0x0F) from LMP Device
  * @param  ptrReceivedPacketProcessingFun - pointer on void-type user callback function that gets pointer on received and parsed Extended GNSS Data structure
  * @retval no return value.
  */
void LMP_Device::SetExtGNSSDataReceivedCallback(void(*ptrReceivedPacketProcessingFun)(GpsDataExt* data))
{
	ptrExtGNSSDataPacketCallback = ptrReceivedPacketProcessingFun;
}

/**
  * @name	Configure_Output_Packet
  * @brief  Function inserts selected packet structure into base packet structure, sets values for basic fields and computes crc32.
  * @param  type - unsigned char value for type of transmitting packet
  * @param  data_ptr - pointer on beginning of packet structure that should be inserted into "data" field of transmitting packet. Packet can be empty
  * @param	size - length of data that should be copied from "data_ptr" into "data" field of transmitting packet
  * @retval no return value.
  */
void LMP_Device::Configure_Output_Packet(uint8_t type, void* data_ptr, uint8_t size)
{
	Output_Packet->preamble = 0xFF;
	Output_Packet->address = 0x01;
	Output_Packet->type = type;
	Output_Packet->length = size;
	if (size)
	{
		memcpy(Output_Packet->data, data_ptr, size);
	}
	*((uint32_t*)&Output_Packet->data[size]) = crc32_compute(Output_Packet, Output_Packet->length + 4);
}

/**
  * @name	Send_Data
  * @brief  Function run void callback function that sending data to serial interface connected to GKV
  * @retval no return value.
  */
void LMP_Device::Send_Data()
{
	if (ptrSendFun)
	{
		ptrSendFun(Output_Packet);
	}
}

/**
  * @name	SendEmptyPacket
  * @brief  Function Configures and Sends Packet with length = 0 for different types of requests
  * @param  type - type of empty packet
  * @retval no return value.
  */
void LMP_Device::SendEmptyPacket(uint8_t type)
{
	Configure_Output_Packet(type, 0, 0);
	Send_Data();
}

/**
  * @name	SetAlgorithm
  * @brief  Function configures and sends Settings Packet (type=0x07) with selected algorithm number
  * @param  algorithm_register_value - number of selected algorithm
  * @retval no return value.
  */
void LMP_Device::SetAlgorithm(uint8_t algorithm_register_value)
{
	Settings GKV_Settings;
	memset(&GKV_Settings, 0, sizeof(GKV_Settings));
	uint8_t type = GKV_DEV_SETTINGS_PACKET;

	if (algorithm_register_value <= ESKF5_NAVIGATON_ALGORITHM)
	{
		GKV_Settings.param_mask |= CHANGE_ALGORITHM;
		GKV_Settings.algorithm = algorithm_register_value;
	}
	Configure_Output_Packet(type, &GKV_Settings, sizeof(GKV_Settings));
	Send_Data();
}

/**
  * @name	SetBaudrate
  * @brief  Function configures and sends Settings Packet (type=0x07) with selected baudrate number
  * @param  baudrate_register_value - number of selected baudrate of main RS-422 interface
  * @retval no return value.
  */
void LMP_Device::SetBaudrate(uint8_t baudrate_register_value)
{
	Settings GKV_Settings;
	memset(&GKV_Settings, 0, sizeof(GKV_Settings));
	uint8_t type = GKV_DEV_SETTINGS_PACKET;

	if (baudrate_register_value <= BAUDRATE_9600)
	{
		GKV_Settings.param_mask |= CHANGE_BAUDRATE;
		GKV_Settings.uart_baud_rate = baudrate_register_value;
	}
	Configure_Output_Packet(type, &GKV_Settings, sizeof(GKV_Settings));
	Send_Data();
}

/**
  * @name	SetDefaultAlgorithmPacket
  * @brief  Function configures and sends Settings Packet (type=0x07) with set sending mode as default packet for current algorithm.
  * @retval no return value.
  */
void LMP_Device::SetDefaultAlgorithmPacket()
{
	Settings GKV_Settings;
	memset(&GKV_Settings, 0, sizeof(GKV_Settings));
	uint8_t type = GKV_DEV_SETTINGS_PACKET;

	GKV_Settings.mode = SET_DEFAULT_ALGORITHM_PACKET;
	GKV_Settings.mode_mask = ALLOW_CHANGE_SELECTED_PACKET;
	Configure_Output_Packet(type, &GKV_Settings, sizeof(GKV_Settings));
	Send_Data();
}

/**
  * @name	SetCustomAlgorithmPacket
  * @brief  Function configures and sends Settings Packet (type=0x07) with set sending mode as custom packet for current algorithm.
  * @retval no return value.
  */
void LMP_Device::SetCustomAlgorithmPacket()
{
	Settings GKV_Settings;
	memset(&GKV_Settings, 0, sizeof(GKV_Settings));
	uint8_t type = GKV_DEV_SETTINGS_PACKET;
	GKV_Settings.mode = SET_CUSTOM_PACKET;
	GKV_Settings.mode_mask = ALLOW_CHANGE_SELECTED_PACKET;
	Configure_Output_Packet(type, &GKV_Settings, sizeof(GKV_Settings));
	Send_Data();
}


/**
  * @name	SetCustomPacketParam
  * @brief  Function configures and sends Custom Parameters Packet (type=0x27) with selected quantity and numbers of selected parameters
  * @param  param_array_ptr - pointer on array of bytes with numbers of custom data parameters
  * @param  quantity_of_params - quantity of selected parameters in byte array
  * @retval no return value.
  */
void LMP_Device::SetCustomPacketParam(uint8_t *param_array_ptr,uint8_t quantity_of_params)
{
	CustomDataParam GKV_CustomDataParam;
	memset(&GKV_CustomDataParam, 0, sizeof(GKV_CustomDataParam));
	uint8_t type = GKV_CUSTOM_DATA_PARAM_PACKET;
	GKV_CustomDataParam.num = quantity_of_params;
	memcpy(&(GKV_CustomDataParam.param), &param_array_ptr, quantity_of_params);
	Configure_Output_Packet(type, &GKV_CustomDataParam, sizeof(GKV_CustomDataParam));
	Send_Data();
}

/**
  * @name	RequestSettings
  * @brief  Function Configures and sends Empty Packet with Settings Request Type (type=0x06)
  * @retval no return value.
  */
void LMP_Device::RequestSettings()
{
	SendEmptyPacket(GKV_DEV_SETTINGS_REQUEST);
}

/**
  * @name	RequestDeviceID
  * @brief  Function Configures and sends Empty Packet with ID Request Type (type=0x04)
  * @retval no return value.
  */
void LMP_Device::RequestDeviceID()
{
	SendEmptyPacket(GKV_DEV_ID_REQUEST);
}

/**
  * @name	RequestData
  * @brief  Function Configures and sends Empty Packet with Data Request Type (type=0x17)
  * @retval no return value.
  */
void LMP_Device::RequestData()
{
	SendEmptyPacket(GKV_DATA_REQUEST);
}


/**
  * @name	CheckConnection
  * @brief  Function Configures and sends Empty Packet with Check Connection Type (type=0x00)
  * @retval no return value.
  */
void LMP_Device::CheckConnection()
{
	SendEmptyPacket(GKV_CHECK_PACKET);
}

/**
  * @name	CheckConnection
  * @brief  Function Configures and sends Empty Packet with Request of Custom Parameters List Type (type=0x26)
  * @retval no return value.
  */
void LMP_Device::RequestCustomPacketParams()
{
	SendEmptyPacket(GKV_CUSTOM_PACKET_PARAM_REQUEST);
}

/**
  * @name	crc32_compute
  * @brief  CRC32 checksum calculation
  * @param  buf - pointer on beginning of packet
  * @param  size - length of packet without checksum
  * @retval function returns result crc32 calculation.
  */
uint32_t LMP_Device::crc32_compute(const void* buf, unsigned long size)
{
	uint32_t crc = 0;
	const uint8_t* p = (const uint8_t*)buf;
	crc = crc ^ 0xFFFFFFFFUL;

	while (size--)
		crc = crc32_tabl[(crc ^ *p++) & 0xFF] ^ (crc >> 8);

	return crc ^ 0xFFFFFFFFUL;
}

/**
  * @name	check
  * @brief  chcecking input packet buffer for crc32
  * @param  pack - pointer on beginning of input packet buffer
  * @retval function returns result of checking crc32. 0x00 - checksum is incorrect, 0x01 - checksum is correct
  */
uint8_t LMP_Device::check(PacketBase* pack)
{
	if (pack->preamble != 0xFF)
		return false;
	if (pack->length > 255)
		return false;

	uint32_t crc = crc32_compute(pack, pack->length + 4);
	uint8_t* p = &pack->data[pack->length];
	uint8_t* t = (uint8_t*)&crc;
	if (*p++ != *t++) return false;
	if (*p++ != *t++) return false;
	if (*p++ != *t++) return false;
	if (*p++ != *t++) return false;

	return true;
	return false;


}

/**
  * @name	put
  * @brief  Function checks current received byte, searching preamble and after finding it puts it into input packet buffer and increments counter
  * @param  b - byte received from serial port connected to GKV
  * @retval function returns result of searching preamble and returns zero until it found.
  */
uint8_t LMP_Device::put(uint8_t b)//проверка на преамбулу
{
	if (CTR == 0)
	{
		if (b != 0xFF)
		{
			return 0;
		}
	}

	*((uint8_t*)(&InputPacket)+CTR) = b;
	CTR++;
	return 1;
}


/**
  * @name	Receive_Process
  * @brief  Main function of received data processing. It can be inserted into main cycle and calls when byte received. function forms packet with received bytes and runs callback fucntion when it formed.
  * @retval function returns result of searching correct packet. 0x00 - not enough bytes received, 0x01 - checksum is incorrect, 0x02 - packet checked
  */
uint8_t LMP_Device::Receive_Process()
{
	char buffer_byte=0;
	if (ptrRecFcn)
	{
		 buffer_byte = (*ptrRecFcn)();
	}
	if (put(buffer_byte))
	{
		return parseCycle();
	}
	return 0;
}




/**
  * @name	parseCycle
  * @brief  Parcing cycle function. When new byte added to input packet buffer. Function checks number of received bytes and checksum result.
  * @retval function returns result of searching correct packet. 0x00 - not enough bytes received, 0x01 - checksum is incorrect, 0x02 - packet checked
  */
uint8_t LMP_Device::parseCycle()
{
	uint8_t status = 0;
	while (1)
	{
		status = parse();
		if (status == NOT_ENOUGH)
		{
			break;
		}
		else if (status == REFIND_PREAMBLE)
		{
			if (!refind_preamble(1))
				break;
		}
		else if (status == CHECK_OK)
		{
			if(ptrPacketProcessingFun)
			{
				ptrPacketProcessingFun(((PacketBase*)&InputPacket));
			}
			RecognisePacket(((PacketBase*)&InputPacket));
			if (!refind_preamble(((PacketBase*)&InputPacket)->length + 8))
				break;
		}
	}
	if (status < CHECK_OK)
	{
		status = 0;
	}
	return status;
}



/**
  * @name	refind_preamble
  * @brief  Moving memory function when checksum is incorrect to check next 0xFF as preamble or moving memory when correct packet processed.
  * @param  start - byte with number of start byte to move memory when checksum is incorrect (1) or when received packet is correct (buf->length) + 8 .
  * @retval Function returns 1 when 0xFF found after memory moving and 0, when it wasn't found.
  */
uint8_t LMP_Device::refind_preamble(int start)
{
	uint8_t* in_buf = (uint8_t*)(InputPacket);
	for (int i = start; i < CTR; i++)
	{
		if (*(in_buf + i) == 0xFF)
		{
			CTR -= i;
			memmove(in_buf, (in_buf + i), CTR);
			return 1;
		}
	}
	CTR = 0;
	return 0;
}



/**
  * @name	parse
  * @brief  Parcing step function. Trying to find correct packet in current quantity of received bytes
  * @retval function returns result of searching correct packet. 0x00 - not enough bytes received, 0x01 - checksum is incorrect, 0x02 - packet checked
  */
uint8_t LMP_Device::parse()
{
	if (CTR >= 4)
	{
		if (((PacketBase *)&InputPacket)->length > DATA_LENGTH)
		{
			return REFIND_PREAMBLE;
		}
	}
	if (CTR < (((PacketBase*)&InputPacket)->length) + 8)
	{
		return NOT_ENOUGH;
	}
	if (!check((PacketBase*)&InputPacket))
	{
		return REFIND_PREAMBLE;
	}
	return CHECK_OK;
}

/**
  * @name	GetInputPacketType
  * @brief  Function sets pointer on user function for processing received and parsed custom parameters packet (type 0x27) from LMP Device
  * @retval function returns value of last received packet type.
  */
uint8_t LMP_Device::GetInputPacketType()
{
	return (((PacketBase*)&InputPacket)->type);
}

/**
  * @name	RecognisePacket
  * @brief  Default callback for every received packet
  * @param  buf - pointer on received packet
  * @retval no return value.
  */
void LMP_Device::RecognisePacket(PacketBase* buf)
{
    switch (buf->type)
    {
    case GKV_ADC_CODES_PACKET:
    {
		ADCData data;
		memcpy(&(data), &(buf->data), sizeof(data));
		if (ptrADCPacketCallback)
		{
			ptrADCPacketCallback(&data);
		}
        break;
    }
    case GKV_RAW_DATA_PACKET:
    {
		RawData data;
		memcpy(&(data), &(buf->data), sizeof(data));
		if (ptrRawDataPacketCallback)
		{
			ptrRawDataPacketCallback(&data);
		}
    }
    case GKV_EULER_ANGLES_PACKET:
    {
		GyrovertData data;
		memcpy(&(data), &(buf->data), sizeof(data));
		if (ptrGyrovertDataPacketCallback)
		{
			ptrGyrovertDataPacketCallback(&data);
		}
        break;
    }
    case GKV_INCLINOMETER_PACKET:
    {
		InclinometerData data;
		memcpy(&(data), &(buf->data), sizeof(data));
		if (ptrInclinometerDataPacketCallback)
		{
			ptrInclinometerDataPacketCallback(&data);
		}
        break;
    }
    case GKV_BINS_PACKET:
    {
		BINSData data;
		memcpy(&(data), &(buf->data), sizeof(data));
		if (ptrBINSDataPacketCallback)
		{
			ptrBINSDataPacketCallback(&data);
		}
        break;
    }
	case GKV_BINS2_PACKET:
	{
		BINS2Data data;
		memcpy(&(data), &(buf->data), sizeof(data));
		if (ptrBINS2DataPacketCallback)
		{
			ptrBINS2DataPacketCallback(&data);
		}
		break;
	}
    case GKV_GNSS_PACKET:
    {
		GpsData data;
		memcpy(&(data), &(buf->data), sizeof(data));

		if (ptrGNSSDataPacketCallback)
		{
			ptrGNSSDataPacketCallback(&data);
		}
		break;
    }
    case GKV_CUSTOM_PACKET:
    {
        CustomData data;
		memcpy(&(data), &(buf->data),buf->length);
		if (ptrCustomDataPacketCallback)
		{
			ptrCustomDataPacketCallback(&data);
		}        break;
    }
	case GKV_DEV_ID_PACKET:
	{
		memcpy(&(DeviceState.GeneralDeviceParameters), &buf->data, sizeof(Test));
		DeviceIDRequestedFlag = false;
		if (ptrDeviceIDCallback)
		{
			ptrDeviceIDCallback(&(DeviceState.GeneralDeviceParameters));
		}
		break;
	}
	case GKV_DEV_SETTINGS_PACKET:
	{
		memcpy(&(DeviceState.CurrentSettings), &buf->data, sizeof(Settings));
		SettingsRequestedFlag = false;
		if (ptrSettingsPacketCallback)
		{
			ptrSettingsPacketCallback(&(DeviceState.CurrentSettings));
		}
		break;
	}
	case GKV_CUSTOM_DATA_PARAM_PACKET:
	{
		memcpy(&(DeviceState.CurrentCustomPacketParameters), &buf->data, sizeof(CustomDataParam));
		CustomPacketParamRequestedFlag = false;
		if (ptrCustomPacketParamCallback)
		{
			ptrCustomPacketParamCallback(&(DeviceState.CurrentCustomPacketParameters));
		}
		break;
	}
	case GKV_CONFIRM_PACKET:
	{
		if (CustomPacketParamSentFlag)
		{
			CustomPacketParamSentFlag = false;
		}
		if (SettingsSentFlag)
		{
			SettingsSentFlag = false;
		}
		break;
	}
    }
}

/**
  * @name	dataNewThreadReceiveFcn
  * @brief  Function of GKV data receiving thread main cycle
  * @retval no return value.
  */
void LMP_Device::dataNewThreadReceiveFcn()
{
	while (gkv_open)
	{
		Receive_Process();
	}
}

/**
  * @name	RunDevice
  * @brief  Function creates and runs new thread for receiving and parsing GKV Data
  * @retval no return value.
  */
void LMP_Device::RunDevice()
{
	Receive_Process();
	std::thread Receiver(&LMP_Device::dataNewThreadReceiveFcn, this);
	Receiver.detach();
	RequestSettings();
	RequestDeviceID();
	RequestCustomPacketParams();
}