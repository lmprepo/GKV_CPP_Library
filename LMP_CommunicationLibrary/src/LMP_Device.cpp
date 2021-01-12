#include "LMP_Device.h"
#include <string.h>

LMP_Device::LMP_Device()
{
}


LMP_Device::~LMP_Device()
{

}


void  LMP_Device::SetSendDataFunction(void(*ptrSendPacketFun)(PacketBase* Output_Packet_Ptr))
{
	ptrSendFun = ptrSendPacketFun;
}

void  LMP_Device::SetReceiveDataFunction(char(*ptrRecPacketFun)())
{
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
	memcpy(Output_Packet->data, data_ptr, size);
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
	ptrSendFun(Output_Packet);
}

void LMP_Device::Set_Algorithm(uint8_t algorithm_register_value)
{
	PacketBase Output_Packet;
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


/*------------------------------Receiving_Data_Parser----------------------------------------------------------------------*/

#define NOT_ENOUGH				0x00
#define REFIND_PREAMBLE			0x01
#define CHECK_OK				0x02



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
uint8_t LMP_Device::Receive_Process(void (*ptrRecognisePacket)(PacketBase* buf), char inputBufferByte)
{
	if (put(inputBufferByte))
	{
		return parseCycle(ptrRecognisePacket);
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
uint8_t LMP_Device::parseCycle(void (*ptrRecognisePacket)(PacketBase* buf))
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
			ptrRecognisePacket(((PacketBase*)&InputPacket));
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
