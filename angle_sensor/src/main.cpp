#include <Arduino.h>
#include <RS485.h>
#include <wit_c_sdk.h>
#include <REG.h>


#define ACC_UPDATE    0x01
#define GYRO_UPDATE   0x02
#define ANGLE_UPDATE  0x04
#define MAG_UPDATE    0x08
#define READ_UPDATE   0x80

static volatile char s_cDataUpdate = 0;


float fAcc[3], fGyro[3], fAngle[3];


static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum);
static void SensorSend(uint8_t *p_data, uint32_t uiSize);
static void Delayms(uint16_t ms);


void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n ESP32 PLC14 — WT901C RS485 IMU Reader");
  RS485.begin(115200);
  delay(300);

  WitInit(WIT_PROTOCOL_MODBUS, 0x51);  
  WitSerialWriteRegister(SensorSend);  
  WitRegisterCallBack(CopeSensorData); 
  WitDelayMsRegister(Delayms);         

  Serial.println(" SDK Initialized. Requesting sensor data...");
}

void loop() {
  
  WitReadReg(AX, 12);
  delay(100);

  
  while (RS485.available()) {
    WitSerialDataIn(RS485.read());
  }

  if (s_cDataUpdate & ANGLE_UPDATE) {
    s_cDataUpdate &= ~ANGLE_UPDATE;

    Serial.printf("X: %.2f°  Y: %.2f°  Z: %.2f°\n",
                  fAngle[0], fAngle[1], fAngle[2]);
  }

  delay(300);
}


static void SensorSend(uint8_t *p_data, uint32_t uiSize) {
  RS485.write(p_data, uiSize);
  RS485.flush();
}


static void Delayms(uint16_t ms) {
  delay(ms);
}


static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum) {
  for (uint32_t i = 0; i < uiRegNum; i++) {
    switch (uiReg) {
      case AZ:
        s_cDataUpdate |= ACC_UPDATE;
        break;
      case GZ:
        s_cDataUpdate |= GYRO_UPDATE;
        break;
      case HZ:
        s_cDataUpdate |= MAG_UPDATE;
        break;
      case Yaw:
        s_cDataUpdate |= ANGLE_UPDATE;
        fAngle[0] = sReg[Roll] / 32768.0f * 180.0f;
        fAngle[1] = sReg[Pitch] / 32768.0f * 180.0f;
        fAngle[2] = sReg[Yaw] / 32768.0f * 180.0f;
        break;
      default:
        s_cDataUpdate |= READ_UPDATE;
        break;
    }
    uiReg++;
  }
}
