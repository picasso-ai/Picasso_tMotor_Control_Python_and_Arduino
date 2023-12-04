#include <Arduino.h>

#define SERIAL_Isra (Serial6)

class Serial_Isra
{
  public:
    void INIT();
    void READ();
    int float_to_uint(float x, float x_min, float x_max, uint8_t nbits);
    float uint_to_float(int x_int, float x_min, float x_max, uint8_t nbits);

    void READ2();
    void WRITE(char Msn[], int Msn_Length);
    void Packet_Decode(uint8_t c);
    void Print_serial();
    char Header[3] = {0xff, 0xee, 0xdd};
    uint8_t SerialData[6];
    uint8_t SerialData2[6];
    int SerialData_Length = 6; //Tommy Edited
    uint8_t ch;
    int count1 = 0;
    uint8_t st = 0;
    uint8_t Datain[5];
    int read_count = 0;

  private:

};
