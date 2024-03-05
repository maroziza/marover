# ExpressLRS / CRSF Protocol 解析メモ

https://github.com/ExpressLRS/ExpressLRS/wiki/CRSF-Protocol

↑まずはコレを読む



- RX側: 全二重(Full Duplex) 42000 Baud / N81
- TX側: 半二重（Half Duplex)


```C
 * CRSF protocol
 *
 * CRSF protocol uses a single wire half duplex uart connection.
 * The master sends one frame every 4ms and the slave replies between two frames from the master.
 *
 * 420000 baud
 * not inverted
 * 8 Bit
 * 1 Stop bit
 * Big endian
 * 420000 bit/s = 46667 byte/s (including stop bit) = 21.43us per byte
 * Max frame size is 64 bytes
 * A 64 byte frame plus 1 sync byte can be transmitted in 1393 microseconds.
 *
 * CRSF_TIME_NEEDED_PER_FRAME_US is set conservatively at 1500 microseconds
 *
 * Every frame has the structure:
 * <Device address><Frame length><Type><Payload><CRC>
 *
 * Device address: (uint8_t)
 * Frame length:   length in  bytes including Type (uint8_t)
 * Type:           (uint8_t)
 * CRC:            (uint8_t)
 ```

## Packet 構造

|device_addr|frame_size|type|Payload..|..|CRC|
|--|--|---|--|--|--|
|C8|xx|xx|..|..|00|

## ヘッダ

```C
/**
 * Define the shape of a standard header
 */
typedef struct crsf_header_s
{
    uint8_t device_addr; // from crsf_addr_e
    uint8_t frame_size;  // counts size after this byte, so it must be the payload size + 2 (type and crc)
    uint8_t type;        // from crsf_frame_type_e
} PACKED crsf_header_t;

#define CRSF_MK_FRAME_T(payload) struct payload##_frame_s { crsf_header_t h; payload p; uint8_t crc; } PACKED
```
```C
// Used by extended header frames (type in range 0x28 to 0x96)
typedef struct crsf_ext_header_s
{
    // Common header fields, see crsf_header_t
    uint8_t device_addr;
    uint8_t frame_size;
    uint8_t type;
    // Extended fields
    uint8_t dest_addr;
    uint8_t orig_addr;
} PACKED crsf_ext_header_t;
```

### device_addr

[device_addr](https://github.com/ExpressLRS/ExpressLRS/blob/426cb69ee037d2b59ead07fd22725a3fbadeea20/src/lib/CrsfProtocol/crsf_protocol.h#LL123C1-L140C1)に送信先を指定する。

|Device|device_addr|
|------|-----------|
|RX→FCの場合　|0xC8|
|プロポ→TXの場合|0xEE|
```C
  CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8,
  CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE,
```

### frame_size

### type

```C
typedef enum
{
    CRSF_FRAMETYPE_GPS = 0x02,
    CRSF_FRAMETYPE_VARIO = 0x07,
    CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
    CRSF_FRAMETYPE_BARO_ALTITUDE = 0x09,
    CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,
    CRSF_FRAMETYPE_OPENTX_SYNC = 0x10,
    CRSF_FRAMETYPE_RADIO_ID = 0x3A,
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
    CRSF_FRAMETYPE_ATTITUDE = 0x1E,
    CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,
    // Extended Header Frames, range: 0x28 to 0x96
    CRSF_FRAMETYPE_DEVICE_PING = 0x28,
    CRSF_FRAMETYPE_DEVICE_INFO = 0x29,
    CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
    CRSF_FRAMETYPE_PARAMETER_READ = 0x2C,
    CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D,

    //CRSF_FRAMETYPE_ELRS_STATUS = 0x2E, ELRS good/bad packet count and status flags

    CRSF_FRAMETYPE_COMMAND = 0x32,
    // KISS frames
    CRSF_FRAMETYPE_KISS_REQ  = 0x78,
    CRSF_FRAMETYPE_KISS_RESP = 0x79,
    // MSP commands
    CRSF_FRAMETYPE_MSP_REQ = 0x7A,   // response request using msp sequence as command
    CRSF_FRAMETYPE_MSP_RESP = 0x7B,  // reply with 58 byte chunked binary
    CRSF_FRAMETYPE_MSP_WRITE = 0x7C, // write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
    // Ardupilot frames
    CRSF_FRAMETYPE_ARDUPILOT_RESP = 0x80,
} crsf_frame_type_e;
```

|Frame type|Byte|
|----------|----|
|CRSF_FRAMETYPE_RC_CHANNELS_PACKED|0x16|

## Type別

### 0x16: CRSF_FRAMETYPE_RC_CHANNELS_PACKED (チャンネル情報）

プロポ操作構造体（11bits)

```C
/**
 * Crossfire packed channel structure, each channel is 11 bits
 */
typedef struct crsf_channels_s
{
    unsigned ch0 : 11;
    unsigned ch1 : 11;
    unsigned ch2 : 11;
    unsigned ch3 : 11;
    unsigned ch4 : 11;
    unsigned ch5 : 11;
    unsigned ch6 : 11;
    unsigned ch7 : 11;
    unsigned ch8 : 11;
    unsigned ch9 : 11;
    unsigned ch10 : 11;
    unsigned ch11 : 11;
    unsigned ch12 : 11;
    unsigned ch13 : 11;
    unsigned ch14 : 11;
    unsigned ch15 : 11;
} PACKED crsf_channels_t;
```

```C
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
```

```C
  constexpr uint8_t outBuffer[] = {
        // No need for length prefix as we aren't using the FIFO
        CRSF_ADDRESS_FLIGHT_CONTROLLER,
        RCframeLength + 2,
        CRSF_FRAMETYPE_RC_CHANNELS_PACKED
    };
```

## ELRS

```C
// ELRS command
#define ELRS_ADDRESS                    0xEE
#define ELRS_BIND_COMMAND               0xFF
#define ELRS_WIFI_COMMAND               0xFE
#define ELRS_PKT_RATE_COMMAND           0x01
#define ELRS_TLM_RATIO_COMMAND          0x02
#define ELRS_SWITCH_MODE_COMMAND        0x03
#define ELRS_MODEL_MATCH_COMMAND        0x04
#define ELRS_POWER_COMMAND              0x06
#define ELRS_BLE_JOYSTIC_COMMAND        17
#define TYPE_SETTINGS_WRITE             0x2D
#define ADDR_RADIO                      0xEA //  Radio Transmitter
```
## コマンドパケット

```C
// prepare elrs setup packet (power, packet rate...)
void CRSF::crsfPrepareCmdPacket(uint8_t packetCmd[], uint8_t command, uint8_t value) {
    packetCmd[0] = ELRS_ADDRESS;
    packetCmd[1] = 6; // length of Command (4) + payload + crc
    packetCmd[2] = TYPE_SETTINGS_WRITE;
    packetCmd[3] = ELRS_ADDRESS;
    packetCmd[4] = ADDR_RADIO;
    packetCmd[5] = command;
    packetCmd[6] = value;
    packetCmd[7] = crsf_crc8(&packetCmd[2], packetCmd[1] - 1); // CRC
}

void CRSF::CrsfWritePacket(uint8_t packet[], uint8_t packetLength) {
    port.write(packet, packetLength);
}
```
https://github.com/kkbin505/Arduino-Transmitter-for-ELRS/blob/ffc1374f5eaca39e906192b0064e159adcb996f9/SimpleTX/crsf.cpp#LL120C1-L134C2

### ELRS_PKT_RATE_COMMAND:Packet Rate (送信レート) 変更コマンド
```C
 
#define PktRate_150 1 // 150Hz
#define PktRate_250 2 // 250Hz
#define PktRate_500 3 // 500Hz

#define Power_25 1   // 25mW
#define Power_100 3   // 100mW

 
crsfClass.crsfPrepareCmdPacket(crsfCmdPacket, ELRS_PKT_RATE_COMMAND, currentPktRate);

crsfClass.CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
```
## CRC算出

```C
#define CRSF_CRC_POLY 0xd5

// CRC8 計算
static byte getCRC8( byte *buf, size_t size)
{
    byte crc8 = 0x00;

    for (int i=0; i<size; i++)
    {
        crc8 ^= buf[i];

        for (int j = 0; j < 8; j++)
        {
            if (crc8 & 0x80)
            {
                crc8 <<= 1;
                crc8 ^= CRSF_CRC_POLY;
            }
            else
            {
                crc8 <<= 1;
            }
        }
    }
    return crc8;
}
```

## 参考資料

- https://github.com/ExpressLRS/ExpressLRS/blob/master/src/lib/CrsfProtocol/crsf_protocol.h
- https://github.com/kkbin505/Arduino-Transmitter-for-ELRS
- https://github.com/danxdz/simpleTx_esp32/
 