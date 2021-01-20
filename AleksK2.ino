#include <SPI.h>

#define MOSI_PIN 11
#define MISO_PIN 12
#define SCK_PIN  13
#define SS_PIN 10

#define PTDinHigh PORTD |= (1 << 7)
#define PTDinLow PORTD &= ~(1 << 7)
#define PTClkHigh PORTD |= (1 << 6)
#define PTClkLow PORTD &= ~(1 << 6)
#define PTStbHigh PORTD |= (1 << 5)
#define PTStbLow PORTD &= ~(1 << 5)

#define CMD1_DMSC_08_20 0x00
#define CMD1_DMSC_09_19 0x08
#define CMD1_DMSC_10_18 0x09
#define CMD1_DMSC_11_17 0x0A
#define CMD1_DMSC_12_16 0x0B
#define CMD1_DMSC_13_15 0x0C
#define CMD1_DMSC_14_14 0x0D
#define CMD1_DMSC_15_13 0x0E
#define CMD1_DMSC_16_12 0x0F

#define CMD2_DSC_WDDM_INC (0x40 | 0x00)
#define CMD2_DSC_WDDM_FIX (0x40 | 0x04)
#define CMD2_DSC_WDLP (0x40 | 0x01)
#define CMD2_DSC_RKD (0x40 | 0x02)
#define CMD2_DSC_RSD (0x40 | 0x03)

#define CMD3_ASC 0xC0

#define CMD3_DCC_01_16 (0x88 | 0 )
#define CMD3_DCC_02_16 (0x88 | 1 )
#define CMD3_DCC_04_16 (0x88 | 2 )
#define CMD3_DCC_10_16 (0x88 | 3 )
#define CMD3_DCC_11_16 (0x88 | 4 )
#define CMD3_DCC_12_16 (0x88 | 5 )
#define CMD3_DCC_13_16 (0x88 | 6 )
#define CMD3_DCC_14_16 (0x88 | 7 )
#define CMD3_DCC_OFF 0x80

bool buttons[48];
volatile bool pool = false;
volatile bool flag_data = false;
volatile byte nn = 0;
volatile byte buff[39];
byte _buff[39];
byte sck = 0;
byte old_sck = 2;
bool live = false;
bool rock = false;
bool disco = false;
bool simul = false;
bool matr = false;
bool clas = false;
bool pops = false;
bool hall = false;

static const uint16_t VFD_SYMB[] = {
  0b0001000110110001, // 48: 0
  0b0000000100100000, // 49: 1
  0b0001000011100001, // 50: 2
  0b0001000101100001, // 51: 3
  0b0000000101110000, // 52: 4
  0b0001000101010001, // 53: 5
  0b0001000111010001, // 54: 6
  0b0000000100100001, // 55: 7
  0b0001000111110001, // 56: 8
  0b0001000101110001, // 57: 9
  0b0000000001000000, // 45: - //10
  0b0000010001000100, // 43: + //11

  0b0001000111000000, // 32: o //12
  0b0000000011010000, // 33: !- //13
  0b0000000101100000, // 34: -! //14
  0b0000010000000000, // 35: ! //15
  0b0101000111100011, // 36: $
  0b0100010100101000, // 37: ugly %
  0b0101101010001101, // 38: ugly &
  0b0000000000000000, // 39: ' (not representable)
  0b0001001000100001, // 40: (
  0b0001000100010001, // 41: )
  0b0100110000001110, // 42: *
  0b0000010000000000, // 44: ,
  0b0000100000000000, // 46: .
  0b0100010000001000, // 47: / //25

  0b0000000000000010, // 58: :
  0b0000000000000010, // 59: ; (not representable)
  0b0100100000001000, // 60: <
  0b0101000011000000, // 61: =
  0b0100010000000100, // 62: >
  0b0101010000101001, // 63: ugly ?
  0b0101111111111111, // 64: ugly @

  0b0000000111110001, // 65: A //33
  0b0001010101100101, // 66: B //34
  0b0001000010010001, // 67: C //35
  0b0001010100100101, // 68: D //36
  0b0001000011010001, // 69: E //37
  0b0000000011010001, // 70: F //38
  0b0001001101100001, // 71: G
  0b0000000111110000, // 72: H //40
  0b0001010000000101, // 73: I //41
  0b0001000110100000, // 74: J //42
  0b0000100010011000, // 75: K //43
  0b0001000010010000, // 76: L //44
  0b0000000110111010, // 77: M //45
  0b0000100110110010, // 78: N //46
  0b0001001100110001, // 79: O
  0b0000000011110001, // 80: P //48
  0b0100000111110001, // 81: Q
  0b0000100011110001, // 82: R //50
  0b0001000101010001, // 83: S //51
  0b0000010000000101, // 84: T //52
  0b0001000110110000, // 85: U //53
  0b0000001010011000, // 86: V //54
  0b0000101110110000, // 87: W //55
  0b0000101000001010, // 88: X //56
  0b0000010001110000, // 89: Y //57
  0b0101010000001001  // 90: Z
};

int getSymb(byte sym1, byte sym2) {
  int n = ((sym2 << 0) & 0xFF) + ((sym1 << 8) & 0xFFFF);
  switch (n) {
    case 0x1144:
      n = VFD_SYMB[12]; // o
      break;
    case 0x6104:
      n = VFD_SYMB[13]; // !
      break;
    case 0x7040:
      n = VFD_SYMB[14]; // !
      break;
    case 0x11C6:
      n = VFD_SYMB[0]; // 0
      break;
    case 0x1040:
      n = VFD_SYMB[1]; // 1
      break;
    case 0x7086:
      n = VFD_SYMB[2]; // 2
      break;
    case 0x70C2:
      n = VFD_SYMB[3]; // 3
      break;
    case 0x7140:
      n = VFD_SYMB[4]; // 4
      break;
    case 0x61C2:
      n = VFD_SYMB[5]; // 5 S
      break;
    case 0x61C6:
      n = VFD_SYMB[6]; // 6
      break;
    case 0x10C0:
      n = VFD_SYMB[7]; // 7
      break;
    case 0x71C6:
      n = VFD_SYMB[8]; // 8
      break;
    case 0x71C2:
      n = VFD_SYMB[9]; // 9
      break;
    case 0x6000:
      n = VFD_SYMB[10]; // -
      break;
    case 0x6208:
      n = VFD_SYMB[11]; // +
      break;
    case 0x71C4:
      n = VFD_SYMB[33]; // A
      break;
    case 0x32CA:
      n = VFD_SYMB[34]; // B
      break;
    case 0x6186:
      n = VFD_SYMB[37]; // E
      break;
    case 0x12CA:
      n = VFD_SYMB[36]; // D
      break;
    case 0x7144:
      n = VFD_SYMB[40]; // H
      break;
    case 0x4514:
      n = VFD_SYMB[43]; // K
      break;
    case 0x7184:
      n = VFD_SYMB[48]; // P
      break;
    case 0x7194:
      n = VFD_SYMB[50]; // R
      break;
    case 0x0106:
      n = VFD_SYMB[44]; // L
      break;
    case 0x0186:
      n = VFD_SYMB[35]; // C
      break;
    case 0x028A:
      n = VFD_SYMB[41]; // I
      break;
    case 0x1146:
      n = VFD_SYMB[53]; // U
      break;
    case 0x0C30:
      n = VFD_SYMB[56]; // X
      break;
    case 0x1D44:
      n = VFD_SYMB[45]; // M
      break;
    case 0x1954:
      n = VFD_SYMB[46]; // N
      break;
    case 0x6184:
      n = VFD_SYMB[38]; // F
      break;
    case 0x0288:
      n = VFD_SYMB[52]; // T
      break;
    case 0x7284:
      n = VFD_SYMB[42]; // J
      break;
    case 0x0524:
      n = VFD_SYMB[54]; // M
      break;
    case 0x1174:
      n = VFD_SYMB[55]; // W
      break;
    case 0x7142:
      n = VFD_SYMB[57]; // Y
      break;
      //default:
  }
  return n;
}

void pt6331_Write (uint8_t data) {
  for (uint8_t i = 0; i < 8; i++) {
    PTClkLow;
    (data & 0x01) ? PTDinHigh : PTDinLow;
    data >>= 1;
    _delay_us(2);
    PTClkHigh;
    _delay_us(2);
  }
}

void pt6331_Cmd (uint8_t cmd) {
  PTStbLow;
  //_delay_us(2);
  pt6331_Write(cmd);
  PTStbHigh;
  _delay_us(2);
}

void pt6331_SetMem(uint8_t data) {
  pt6331_Cmd(CMD2_DSC_WDDM_INC);
  PTStbLow;
  pt6331_Write(CMD3_ASC);
  for (uint8_t i = 0; i <= 0x2f; i++)  {
    pt6331_Write(data);
  }
  PTStbHigh;
  _delay_us(2);
}

void pt6331_WriteDigit(unsigned char digit, unsigned char s1, unsigned char s2, unsigned char s3) {
  pt6331_Cmd(CMD2_DSC_WDDM_INC);
  PTStbLow;
  pt6331_Write(CMD3_ASC | (digit * 3));
  pt6331_Write(s1);
  pt6331_Write(s2);
  pt6331_Write(s3 & 0xff);
  PTStbHigh;
  _delay_us(2);
}

void pt6311_SetData(uint8_t digit_pos, uint32_t data) {
  pt6331_Cmd(CMD2_DSC_WDDM_INC);
  PTStbLow;
  pt6331_Write(CMD3_ASC | (digit_pos * 0x03));
  pt6331_Write((uint8_t)(data      ));
  pt6331_Write((uint8_t)(data >>  8));
  pt6331_Write((uint8_t)(data >> 16));
  PTStbHigh;
}

void pt6311_write_char(uint8_t d_pos, char ch) {
  uint16_t _raw = (ch >= 32 && ch <= 127) ? VFD_SYMB[ch - 32] : 0;
  pt6311_SetData(13 - d_pos - 1, _raw);
}

void pt6331_WriteDisplayRam(unsigned char addr, unsigned char *data, unsigned char len) {
  pt6331_Cmd(CMD2_DSC_WDDM_INC);
  PTStbLow;
  pt6331_Write(CMD3_ASC | addr);
  while (len--) {
    pt6331_Write(*data++);
  }
  PTStbHigh;
  _delay_us(2);
}

void pt6331_Init(unsigned char dispMode, unsigned char dispCtrl) {
  PTStbHigh;
  PTClkHigh;
  PTDinHigh;
  _delay_us(2);
  pt6331_SetMem(0x00);
  pt6331_Cmd(dispMode);
  pt6331_Cmd(dispCtrl);
}

void pt6311_readKeypad() {
  PTStbLow;
  _delay_us(2);
  pt6331_Write(0x42);
  _delay_us(2);
  DDRD &= ~(1 << 7); // INPUT
  _delay_us(2);
  PTClkLow;
  for (int i = 0; i <= 47; i++) {
    PTClkHigh;
    _delay_us(2);
    buttons[i] = bitRead(PIND, 7);
    PTClkLow;
    _delay_us(2);
  }
  PTStbHigh;
  _delay_us(2);
  DDRD |= (1 << 7); // OUTPUT
  _delay_us(2);
  pt6331_Cmd(CMD3_DCC_12_16);
  _delay_us(2);
}

byte spi_receive() {
  while (!(SPSR & (1 << SPIF))) {};
  return SPDR;
}

void sendKey() {
  bool flag = false;
  int n = 0;
  while (!bitRead(PINB, 2)) { //пока SS опущен
    if (!bitRead(PINB, 5) && !flag) { // digitalRead(SCK_PIN) == LOW
      flag = true;
      (buttons[n]) ? PORTB |= (1 << 3) : PORTB &= ~(1 << 3);
      n++;
    } else if (bitRead(PINB, 5) && flag) {
      flag = false;
    }
  }
}

void pools() {
  byte b = 0;
  byte bt = 0XFF;
  while (pool) {
    _delay_us(10);
    while (bitRead(PINB, 2)) {
      while (!bitRead(PINB, 2)) { //пока SS опущен
        sck = bitRead(PINB, 5);
        if (sck == 1 && old_sck != 1) {
          old_sck = sck;
          //PORTD |= (1 << 4);
          (bitRead(PINB, 3)) ? bt |= (1 << b) : bt &= ~(1 << b);
          b++;
          if (b > 7) {
            //PORTD &= ~(1 << 4);
            if (bt == 0x42) {
              _delay_us(8);
              DDRB |= (1 << 3); //MOSI_PIN OUTPUT
              PORTB &= ~(1 << 3); //MOSI_PIN LOW
              sendKey();
            }
          }
        } else if (sck != old_sck) {
          old_sck = sck;
        }
        //PORTD &= ~(1 << 4);
      }
    }
    if (pool) {
      pool = false;
      SPCR |= (1 << SPE); // Включаем SPI
      DDRB &= ~(1 << 3); //MOSI_PIN INPUT
      SPCR |= (1 << SPIE); //Разрешение прерываний SPI
    }
  }
}

ISR (SPI_STC_vect) {
  byte c = SPDR;
  //Serial.print(c, HEX);
  //Serial.print(" ");
  if (c == 0xC0) {
    flag_data = true;
  }
  if (c == 0x8F) {
    SPCR &= ~(1 << SPIE);
    SPCR &= ~(1 << SPE); // Отключаем SPI
    pool = true;
    pools();
  }
  if (flag_data && nn < 39) {
    buff[nn] = c;
    nn++;
  } else {
    flag_data = false;
    nn = 0;
  }
}

void SPI_Slave_Init() {
  byte IOReg;
  pinMode(MOSI_PIN, INPUT); //11
  pinMode(SCK_PIN, INPUT); //13
  pinMode(SS_PIN, INPUT); //10
  pinMode(MISO_PIN, OUTPUT); //12
  SPCR = B00000000;
  SPCR = (1 << SPIE) | (1 << SPE) | (1 << DORD) | (1 << CPOL) | (1 << CPHA);
  IOReg = SPSR; // Очистить бит SPIF в регистре SPSR.
  IOReg = SPDR;
  sei();
}

void setup() {
  SPI_Slave_Init();
  Serial.begin(19200);
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
  //pinMode(4, OUTPUT);
  //DDRD |= (1 << 4); //Для дебага
  //PORTB &= ~(1 << 4);
  //PORTD |= (1 << 4);
  delay(200);
  pt6331_Init(CMD1_DMSC_13_15, CMD3_DCC_12_16);
  pt6331_SetMem(0x00); // clear all segments
  //pt6331_WriteDisplayRam(24, &data, 1); // light up some segments only(the wheel part)
  //pt6331_SetMem(0xFF); // light up all segments
  //pt6331_WriteDigit(10,0xff,0xff,0xff);
}

void readEq() {
  if (_buff[35] == 0x01 && _buff[34] == 0x06 && _buff[32] == 0x02 && _buff[31] == 0x8A) {
    live  = true;
    rock  = false;
    disco = false;
    simul = false;
    matr  = false;
    clas  = false;
    hall  = false;
    pops  = false;
  }
  if (_buff[35] == 0x71 && _buff[34] == 0x94 && _buff[32] == 0x11 && _buff[31] == 0xC6) {
    live  = false;
    rock  = true;
    disco = false;
    simul = false;
    matr  = false;
    clas  = false;
    hall  = false;
    pops  = false;
  }
  if (_buff[35] == 0x12 && _buff[34] == 0xCA && _buff[32] == 0x02 && _buff[31] == 0x8A) {
    live  = false;
    rock  = false;
    disco = true;
    simul = false;
    matr  = false;
    clas  = false;
    hall  = false;
    pops  = false;
  }
  if (_buff[35] == 0x61 && _buff[34] == 0xC2 && _buff[32] == 0x02 && _buff[31] == 0x8A) {
    live  = false;
    rock  = false;
    disco = false;
    simul = true;
    matr  = false;
    clas  = false;
    hall  = false;
    pops  = false;
  }
  if (_buff[35] == 0x1D && _buff[34] == 0x44 && _buff[32] == 0x71 && _buff[31] == 0xC4) {
    live  = false;
    rock  = false;
    disco = false;
    simul = false;
    matr  = true;
    clas  = false;
    hall  = false;
    pops  = false;
  }
  if (_buff[35] == 0x01 && _buff[34] == 0x86 && _buff[32] == 0x01 && _buff[31] == 0x06) {
    live  = false;
    rock  = false;
    disco = false;
    simul = false;
    matr  = false;
    clas  = true;
    hall  = false;
    pops  = false;
  }
  if (_buff[35] == 0x71 && _buff[34] == 0x44 && _buff[32] == 0x71 && _buff[31] == 0xC4) {
    live  = false;
    rock  = false;
    disco = false;
    simul = false;
    matr  = false;
    clas  = false;
    hall  = true;
    pops  = false;
  }
  if (_buff[35] == 0x71 && _buff[34] == 0x84 && _buff[32] == 0x11 && _buff[31] == 0xC6) {
    live  = false;
    rock  = false;
    disco = false;
    simul = false;
    matr  = false;
    clas  = false;
    hall  = false;
    pops  = true;
  }
}

void loop() {
  while (1) {
    for (byte i = 0; i < 39; i++) {
        _buff[i] = buff[i];
    }
    int ram_pos = 0;
    int dbl_byte = 0;
    bool dsp = false;
    bool db = false;
    bool ms = false;
    bool ac3 = false;
    bool mic = false;
    bool mute = false;
    bool ch5 = false;
    bool stereo3 = false;
    readEq();    
    if (bitRead(buff[38], 1)) dsp = true;
    if (bitRead(buff[38], 2)) ac3 = true;
    if (bitRead(buff[38], 3)) stereo3 = true;
    if (bitRead(buff[38], 4)) db = true;
    if (bitRead(buff[38], 5)) ms = true;
    if (bitRead(buff[38], 6)) mute = true;
    if (bitRead(buff[37], 0)) mic = true;
    if (bitRead(buff[37], 7)) ch5 = true;
    
    for (uint8_t i = 0; i < 21; i++) { //Отключаем ненужные сегменты
        _buff[i] = 0;
    }
    for (uint8_t i = 0; i < 13; ++i) {
      /*if (i == 12) {
        Serial.print(ram_pos);
        Serial.print(" - ");
        Serial.print(i);
        Serial.print(" - ");
        Serial.print(buff[ram_pos + 2], HEX);
        Serial.print(":");
        Serial.println(buff[ram_pos + 1], HEX);
        //Serial.print(":");
        //Serial.println(buff[ram_pos], HEX);
      }*/ 
      if (db) {
        _buff[17] = 0b01100101; //  0b00010101 01100101, // 66: B //34
        _buff[16] = 0b00010101;
        _buff[15] = 0xFF;
        _buff[20] = 0b11100000; //  0b00010001 11100000, // 100: d ///
        _buff[19] = 0b00010001;
        _buff[18] = 0xFF;
      }
      if (ms) {
        _buff[17] = 0b01010001; //  0b00010001 01010001, // 53: 5
        _buff[16] = 0b00010001;
        _buff[15] = 0xFF;
        _buff[20] = 0b11000000; //  0b00000101 11000000, // 100: d ///
        _buff[19] = 0b00000101;
        _buff[18] = 0xFF;
      }
      if (mute) {
        _buff[10] |= (1 << 6);
        dbl_byte = ((_buff[11] << 0) & 0xFF) + ((_buff[10] << 8) & 0xFFFF);
      } else {
        _buff[10] &= ~(1 << 6);
        dbl_byte = ((_buff[11] << 0) & 0xFF) + ((_buff[10] << 8) & 0xFFFF);
      }
      if (mic) {
        _buff[10] |= (1 << 4);
        dbl_byte = ((_buff[11] << 0) & 0xFF) + ((_buff[10] << 8) & 0xFFFF);
      } else {
        _buff[10] &= ~(1 << 4);
        dbl_byte = ((_buff[11] << 0) & 0xFF) + ((_buff[10] << 8) & 0xFFFF);
      }

      if (i == 12) {
        _buff[38] &= ~(1 << 6); //выкл SRS
        _buff[38] &= ~(1 << 5); //выкл BBE
        (ch5) ? _buff[37] |= (1 << 0) : _buff[37] &= ~(1 << 0);
        (live) ? _buff[37] |= (1 << 6) : _buff[37] &= ~(1 << 6);
        (dsp) ? _buff[38] |= (1 << 0) : _buff[38] &= ~(1 << 0);
        (ac3) ? _buff[38] |= (1 << 7) : _buff[38] &= ~(1 << 7);
        (ac3) ? _buff[38] |= (1 << 2) : _buff[38] &= ~(1 << 2); //dolby
        dbl_byte = ((_buff[38] << 0) & 0xFF) + ((_buff[37] << 8) & 0xFFFF);
      }
      if (i > 6 && i < 12) {
        dbl_byte = getSymb(_buff[ram_pos + 2], _buff[ram_pos + 1]);
      } else if (i != 12 && i != 3) {
        dbl_byte = ((_buff[ram_pos + 2] << 0) & 0xFF) + ((_buff[ram_pos + 1] << 8) & 0xFFFF);
      }

      if (rock && i == 11) {
        dbl_byte |= (1 << 14);
      } else  if (disco && i == 10) {
        dbl_byte |= (1 << 14);
      } else  if (simul && i == 9) {
        dbl_byte |= (1 << 14);
      } else  if (matr && i == 8) {
        dbl_byte |= (1 << 14);
      } else if (clas && i == 7) {
        dbl_byte |= (1 << 14);
      } else if (hall && i == 6) {
        dbl_byte |= (1 << 14);
      } else if (pops && i == 5) {
        dbl_byte |= (1 << 14);
      }
      if (!dsp && (i > 4 && i <= 12 )) {
        dbl_byte &= ~(1 << 14);
      }
      if (stereo3 && i == 7) {
        dbl_byte |= (1 << 13);
      } else {
        dbl_byte &= ~(1 << 13);
      }

      pt6331_WriteDigit(i, ((dbl_byte >> 0) & 0xFF), ((dbl_byte >> 8) & 0xFF), 0xFF);
      ram_pos = ram_pos + 3;
    }
    pt6311_readKeypad();
  };
}
