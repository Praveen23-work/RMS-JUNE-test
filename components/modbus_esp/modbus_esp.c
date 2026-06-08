#include <stdio.h>
#include "modbus_esp.h"
#include <math.h>

uint8_t len_of_response_num = 0;

uint16_t modbus_crc16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t pos = 0; pos < len; pos++)
    {
        crc ^= (uint16_t)buf[pos];

        for (uint8_t i = 0; i < 8; i++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

static inline void modbus_read_req(uint8_t *buf, uint8_t slave, uint8_t read_write,
                                   uint16_t addr, uint16_t qty)
{
    buf[0] = slave;
    buf[1] = read_write;
    buf[2] = addr >> 8;
    buf[3] = addr & 0xFF;
    buf[4] = qty >> 8;   // Value to write or number of registers to read
    buf[5] = qty & 0xFF; // Value to write or number of registers to read
    *(uint16_t *)&buf[6] = modbus_crc16(buf, 6);
}

void MODBUS_handler(uint8_t SensorSelect,
                    uint8_t *modbusRequest,
                    uint8_t read_write,
                    uint16_t qty_or_val)
{
    uint8_t  slave_id = 0x01;   // Default slave (RMS Board)
    uint16_t reg_addr = 0xFFFF;

    /* READ → always 1 register */
    if (read_write != 1)  // not write
        qty_or_val = 1;

    switch (SensorSelect)
    {
        /* -------- INPUT REGISTERS (FC04) -------- */
        case DigitalInputs: reg_addr = 0x0000; break;

        case Pressure1:     reg_addr = 0x0001; break;
        case Pressure2:     reg_addr = 0x0002; break;

        case Flow1:         reg_addr = 0x000B; break;
        case Flow2:         reg_addr = 0x000C; break;

        /* -------- HOLDING REGISTERS (FC03 / FC06) -------- */
        case Rly_SV1:       reg_addr = 0x0000; break;
        case Rly_SV2:       reg_addr = 0x0001; break;
        case Rly_SV3:       reg_addr = 0x0002; break;
        case Rly_SV4:       reg_addr = 0x0003; break;
        case Rly_HP:        reg_addr = 0x0004; break;
        case Rly_UV:        reg_addr = 0x0005; break;
        case Rly_BackW:     reg_addr = 0x0006; break;
        case Rly_RW:        reg_addr = 0x0007; break;

        case HMI_TDS_IN:    reg_addr = 0x000C; break;
        case HMI_TEMPW_IN:  reg_addr = 0x000D; break;
        case HMI_TDS_OUT:   reg_addr = 0x000E; break;
        case HMI_TEMPW_OUT: reg_addr = 0x000F; break;

        case SYS_en:        reg_addr = 0x0010; break;
        case DDPS_en:       reg_addr = 0x0011; break;
        case BACKW_FREQ:    reg_addr = 0x0012; break;
        case PS_DELTA:      reg_addr = 0x0013; break;
        case HMI_BACKW_BTN: reg_addr = 0x0014; break;
        case RMS_SENSOR_CFG:reg_addr = 0x0015; break;// added for 2 level sensor config

        /* -------- TDS MODULE Board  (INPUT REGISTERS) -------- */
        case TDS_IN:        reg_addr = 0x0001; slave_id = 0x05; break;
        case TEMPW_IN:      reg_addr = 0x0002; slave_id = 0x05; break;
        case TDS_OUT:       reg_addr = 0x0003; slave_id = 0x05; break;
        case TEMPW_OUT:     reg_addr = 0x0004; slave_id = 0x05; break;

        default:
            len_of_response_num = 0;
            memset(modbusRequest, 0, 8);
            return;
    }

    /* -------- FUNCTION CODE SELECTION -------- */

    if (read_write == 1)
    {
        /* WRITE → only relays and MOSFETs allowed */
        read_write = 0x06;
    }
    else
    {
        /* READ */
        if (SensorSelect <= Flow2)
        {
            // printf("READ INPUT REGISTERS\n");
            read_write = 0x04;   // DigitalInputs → Flow2 (Input Registers)
        }
        else
        {
            // printf("READ HOLDING REGISTERS\n");
            read_write = 0x03;
        }
               // Others
    }

    /* -------- BUILD MODBUS FRAME -------- */
    modbus_read_req(modbusRequest,
                    slave_id,
                    read_write,
                    reg_addr,
                    qty_or_val);

    /* -------- EXPECTED RESPONSE LENGTH -------- */
    if (read_write == 0x03 || read_write == 0x04)
        len_of_response_num = 5 + (qty_or_val * 2);
    else    // 0x06
        len_of_response_num = 8;
        
}



uint8_t MODBUS_Sensor_Count()
{
    return SENSOR_TYPE_COUNT;
}


uint8_t len_of_modbus_response()
{
    return len_of_response_num;
}



// Standard MODBUS RTU CRC16
uint16_t modbus_crc(uint8_t *data, uint8_t len) {
    uint16_t crc = 0xFFFF;

    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else         crc >>= 1;
        }
    }
    return crc;
}



float decodeModbusResponse_TDS(uint8_t *receivedData)
{
  uint16_t data = (receivedData[3] << 8) | receivedData[4];
  float tds = (float) data * 0.7;
  return tds;
}



float  decodeModbusResponse_Chl(uint8_t *receivedData)
{
    unsigned int rawData = (receivedData[3] << 8) | receivedData[4];  // Combine bytes
    unsigned int decimalPoints = receivedData[5];  // Number of decimal places

    // Convert raw data to floating-point value
    float sensorValue = rawData / pow(10, decimalPoints);  // Apply decimal shift

    // Assign to global variable
    return sensorValue;
}

float decodeModbusResponse_UFM( uint8_t *floatBytes)
{
	 union {
        uint8_t b[4];
        float f;
    } data_f;

    // Copy bytes in correct order for your platform (little-endian)
    data_f.b[0] = floatBytes[6];
    data_f.b[1] = floatBytes[5];
    data_f.b[2] = floatBytes[4];
    data_f.b[3] = floatBytes[3];

	return  data_f.f;
}
