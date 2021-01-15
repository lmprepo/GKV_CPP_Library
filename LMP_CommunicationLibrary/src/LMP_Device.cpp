#include "LMP_Device.h"
#include <string.h>

LMP_Device::LMP_Device()
{
}


LMP_Device::~LMP_Device()
{
	gkv_open = false;
}


void  LMP_Device::SetSendDataFunction(void(*ptrSendPacketFun)(PacketBase* Output_Packet_Ptr))
{
	ptrSendFun = ptrSendPacketFun;
}

void  LMP_Device::SetReceiveDataFunction(char(*ptrRecPacketFun)())
{
	ptrRecFcn = ptrRecPacketFun;
}

void  LMP_Device::SetReceivedPacketCallback(void(*ptrReceivedPacketProcessingFun)(PacketBase* Input_Packet_Ptr))
{
	ptrPacketProcessingFun = ptrReceivedPacketProcessingFun;
}

void LMP_Device::SetSettingsReceivedCallback(void(*ptrReceivedPacketProcessingFun)(Settings* settings))
{
	ptrSettingsPacketCallback = ptrReceivedPacketProcessingFun;
}

void LMP_Device::SetCustomPacketParamReceivedCallback(void(*ptrReceivedPacketProcessingFun)(CustomDataParam* param))
{
	ptrCustomPacketParamCallback = ptrReceivedPacketProcessingFun;
}

void LMP_Device::SetCustomPacketReceivedCallback(void(*ptrReceivedPacketProcessingFun)(CustomData* data))
{
	ptrCustomDataPacketCallback = ptrReceivedPacketProcessingFun;
}

void LMP_Device::SetADCDataReceivedCallback(void(*ptrADCPacketRecCallback)(ADCData* data))
{
	ptrADCPacketCallback = ptrADCPacketRecCallback;
}

void LMP_Device::SetRawDataReceivedCallback(void(*ptrReceivedPacketProcessingFun)(RawData* data))
{
	ptrRawDataPacketCallback = ptrReceivedPacketProcessingFun;
}

void LMP_Device::SetGyrovertDataReceivedCallback(void(*ptrReceivedPacketProcessingFun)(GyrovertData* data))
{
	ptrGyrovertDataPacketCallback = ptrReceivedPacketProcessingFun;
}

void LMP_Device::SetInclinometerDataReceivedCallback(void(*ptrReceivedPacketProcessingFun)(InclinometerData* data))
{
	ptrInclinometerDataPacketCallback = ptrReceivedPacketProcessingFun;
}

void LMP_Device::SetBINSDataReceivedCallback(void(*ptrReceivedPacketProcessingFun)(BINSData* data))
{
	ptrBINSDataPacketCallback = ptrReceivedPacketProcessingFun;
}

void LMP_Device::SetBINS2DataReceivedCallback(void(*ptrReceivedPacketProcessingFun)(BINS2Data* data))
{
	ptrBINS2DataPacketCallback = ptrReceivedPacketProcessingFun;
}

void LMP_Device::SetGNSSDataReceivedCallback(void(*ptrReceivedPacketProcessingFun)(GpsData* data))
{
	ptrGNSSDataPacketCallback = ptrReceivedPacketProcessingFun;
}

void LMP_Device::SetExtGNSSDataReceivedCallback(void(*ptrReceivedPacketProcessingFun)(GpsDataExt* data))
{
	ptrExtGNSSDataPacketCallback = ptrReceivedPacketProcessingFun;
}
/**
  * @name	Configure_Output_Packet
  * @brief  Function inserts selected packet structure into base packet structure, sets values for basic fields and computes crc32.
  * @param  Output_Packet_Ptr - pointer on beginning of base packet structure that should be transmitted to GKV (pointer on full transmitting packet)
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
  * @param  ptrSendPacketFun - pointer on void-type callback function that gets pointer on PacketBase structure and sends "length" fiels + 8 bytes
  * @param  Output_Packet_Ptr - pointer on beginning of base packet structure that should be transmitted to GKV (pointer on full transmitting packet)
  * @retval no return value.
  */
void LMP_Device::Send_Data()
{
	if (ptrSendFun)
	{
		ptrSendFun(Output_Packet);
	}
}

void LMP_Device::SendEmptyPacket(uint8_t type)
{
	Configure_Output_Packet(type, 0, 0);
	Send_Data();
}

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
  * @brief  Function configures base packet of 0x06 type and sends it to GKV to set sending mode as default packet for current algorithm. Sending via callback function SendPacketFun
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
  * @brief  Function configures base packet of 0x06 type and sends it to GKV to set sending mode as custom packet for current algorithm. Sending via callback function SendPacketFun
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

void LMP_Device::RequestSettings()
{
	SendEmptyPacket(GKV_DEV_SETTINGS_REQUEST);
}

void LMP_Device::RequestDeviceID()
{
	SendEmptyPacket(GKV_DEV_ID_REQUEST);
}

void LMP_Device::RequestData()
{
	SendEmptyPacket(GKV_DATA_REQUEST);
}

void LMP_Device::CheckConnection()
{
	SendEmptyPacket(GKV_CHECK_PACKET);
}

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

//
///*------------------------------Receiving_Data_Parser----------------------------------------------------------------------*/
//
//#define NOT_ENOUGH				0x00
//#define REFIND_PREAMBLE			0x01
//#define CHECK_OK				0x02



/**
  * @name	put
  * @brief  Function checks current received byte, searching preamble and after finding it puts it into input packet buffer and increments counter
  * @param  b - byte received from UART/COM-port connected to GKV
  * @param  ptr_cnt - pointer on byte counter in input packet buffer
  * @param  buf - pointer on beginning of input packet buffer
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
  * @brief  Main fuction of received data processing. It can be inserted into main cycle and calls when byte received. function forms packet with received bytes and runs callback fucntion when it formed.
  * @param  ptrRecognisePacket - pointer on user callback function that should process packet when correct packet receive
  * @param  ptrInputStructure - pointer on structure includes current byte value, byte counter and full structure of receiving packet
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
  * @param  ptrRecognisePacket - pointer on user callback function that should process packet when correct packet receive
  * @param  ptr_cnt - pointer on byte counter in input packet buffer
  * @param  buf - pointer on beginning of input packet buffer
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
  * @param  ptr_cnt - pointer on byte counter in input packet buffer
  * @param  buf - pointer on beginning of input packet buffer
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
  * @brief  Parcing step function. Trying to find correct packet in current number of received bytes
  * @param  ptr_cnt - pointer on byte counter in input packet buffer
  * @param  buf - pointer on beginning of input packet buffer
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

uint8_t LMP_Device::GetInputPacketType()
{
	return (((PacketBase*)&InputPacket)->type);
}

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

void LMP_Device::dataNewThreadReceiveFcn()
{
	while (gkv_open)
	{
		Receive_Process();
	}
}

void LMP_Device::RunDevice()
{
	Receive_Process();
	std::thread Receiver(&LMP_Device::dataNewThreadReceiveFcn, this);
	Receiver.detach();
	RequestSettings();
	RequestDeviceID();
	RequestCustomPacketParams();
}