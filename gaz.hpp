

#ifndef __MICS_VZ_89TE_H__

#define __MICS_VZ_89TE_H__



  #define MICS_VZ_89TE_PORT    0x00

  #define MICS_VZ_89TE_ADDR      0x70 //0x70 default I2C address
  //registers
  #define MICS_VZ_89TE_ADDR_CMD_GETSTATUS 0x0C  // This command is used to read the VZ89 status coded with 6 bytes:
  #define MICS_VZ_89TE_DATE_CODE 0x0D

  #define MICS_VZ_89TE_ADDR_CMD_GETSTATUS_CRC 0xF3  // This command is used to read the VZ89 status coded with 6 bytes:
  #define MICS_VZ_89TE_DATE_CODE_CRC 0xF2

  #include "Arduino.h"
  
   class MICS_VZ_89TE {
      
  public:
      MICS_VZ_89TE(void);
      // To get the data
      float getCO2(void);
      float getVOC(void);
      float getYear(void);
      float getMonth(void);
      float getDay(void);
      float getRev(void);
      float getCrc(void);
      float getStatus(void);
      
      float co2;
      float voc;
      float status;
      
      float year ;
      float month;
      float day;
      float rev;
      float crc;
      
      bool begin(void);
      void readSensor(void);
      void getVersion(void);
      
      
  private:
      
      uint8_t   _i2caddr;
      void readData(byte reg, byte crc, uint8_t data[]);

  };

#endif
