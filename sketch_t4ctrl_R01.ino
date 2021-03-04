byte REV_Code[] = {0x00, 0x81}; // [0]: free, [1] Rev code of sketch
byte CMD;  // 0xAA = Read, 0xA5 = Write, 0x55 = GPIO, 0x5A = SPI, 0xC3 = Pattern, 0x69 = Clock, 0x96 = Config

int led = 13;
int intTO = 100;

int i;
int j;

byte PAT[256];
int PAT_WD;
int PAT_LEN;
int PAT_EN;
int PAT_CNT;
int PAT_RPT;
int PAT_CYC;
int PAT_UNIT = 0;

int cnt_to;
byte ADR;
byte inDAT[512]; //no delimiter, byte data only
byte outDAT[256];
int cntDAT;
byte byteDAT;
int LNG; // Data length for Read
byte VAL;
byte RC;
int DLY;
byte SPI_BO;
byte SPI_CD;
byte SPI_DM;
int intAVAL[] = {0, 0};
byte byteAVAL[4];
byte Item;
byte Param;
uint32_t uintTMP;

IMXRT_LPI2C_t *port_i2c;
IMXRT_REGISTER32_t *port_iomuxc;
IMXRT_REGISTER32_t *port_iomuxc_b;
// IMXRT_REGISTER32_t *port_iomuxc_gpr;
IMXRT_ADCS_t *port_adc1;
IMXRT_ADCS_t *port_adc2;
// IMXRT_REGISTER32_t *port_gpio1;
// IMXRT_REGISTER32_t *port_gpio2;
// IMXRT_REGISTER32_t *port_gpio4;
IMXRT_REGISTER32_t *port_gpio6;
IMXRT_REGISTER32_t *port_gpio7;
IMXRT_REGISTER32_t *port_gpio9;
IMXRT_REGISTER32_t *port_spi;
IMXRT_FLEXPWM_t *port_pwm;

// for I2C function
const int CONST_TIMEOUT = 100;
const byte SC_SUCCESS = 0xFF;
const byte SC_TIMEOUT = 0x80;
const byte SC_NACK = 0x00;

const byte SC_SUCCESS_GPIO = 0xFD;

void setup() {
  Serial.begin(9600);

  // pointer
  port_i2c = &IMXRT_LPI2C1; //0x403F0000
  port_iomuxc = &IMXRT_IOMUXC;  //0x401F8000
  port_iomuxc_b = &IMXRT_IOMUXC_b;  //0x401F8400
  port_adc1 = &IMXRT_ADC1; //0x400C4000
  port_adc2 = &IMXRT_ADC2; //0x400C8000
  port_gpio6 = &IMXRT_GPIO6; //0x42000000
  port_gpio7 = &IMXRT_GPIO7; //0x42004000
  port_gpio9 = &IMXRT_GPIO9; //0x4200C000
  port_spi = &IMXRT_LPSPI4; //0x403A0000
  port_pwm = &IMXRT_FLEXPWM2; //0x403E0000

  // setup for I2C
  // enable Clock for LPI2C1
  CCM_CSCDR2 = (CCM_CSCDR2 & ~CCM_CSCDR2_LPI2C_CLK_PODF(63)) | CCM_CSCDR2_LPI2C_CLK_SEL;
  CCM_CCGR2 |= 0x000000C0; // CG3 = 11, enable LPI2C1 clock

  // #19 = SCL, GPIO_AD_B1_00
  port_iomuxc->offset0FC = 0x00000013; // SION = 1, MUX_MODE = 011(LPI2C_SCL)
  // port_iomuxc->offset2EC = 0x0000F861; // Pull-up = 22kohm, PUS = 11 (22kohm), PUE = 1, PKE = 1, ODE = 1, SPEED = 01, DSE = 100, SRE = 1
  port_iomuxc->offset2EC = 0x0000E861; // Pull-up = none, PUS = 11 (22kohm), PUE = 1, PKE = 0, ODE = 1, SPEED = 01, DSE = 100, SRE = 1
  port_iomuxc_b->offset0CC = 0x00000001; // DAISY = 1

  // #18 = SDA, GPIO_AD_B1_01
  port_iomuxc->offset100 = 0x00000013; // SION = 1, MUX_MODE = 11(LPI2C_SDA)
  // port_iomuxc->offset2F0 = 0x0000F861; // Pull-up = 22kohm
  port_iomuxc->offset2F0 = 0x0000E861; // Pull-up = none
  port_iomuxc_b->offset0D0 = 0x00000001; // DAISY = 1

  i2c_set_freq();

  // Enable I2C
  port_i2c->MCR = 0x00000001;

  // setup for GPIO
  // Clock, ipg_clk_root = 108 MHz?
  CCM_CCGR2 |= 0x00030300; // CG8 = CG4 = 11, enable ADC1/ADC2 clock

  // #2 = D2, GPIO_EMC_04
  port_iomuxc->offset024 = 0x00000015; // SIN = 1, MUX_MODE = 101 (GPIO4/9_IO04)
  port_iomuxc->offset214 = 0x000000B0; // PKE = 0 (disable pull-up/Keeper), Speed = 10 (Fast), DSE = 110, SRE = 0 (Slow)

  // #3 = D3, GPIO_EMC_05
  port_iomuxc->offset028 = 0x00000015; // SIN = 1, MUX_MODE = 101 (GPIO4/9_IO05)
  port_iomuxc->offset218 = 0x000000B0; // PKE = 0 (disable pull-up/Keeper), Speed = 10 (Fast), DSE = 110, SRE = 0 (Slow)

  // #4 = D4, GPIO_EMC_06
  port_iomuxc->offset02C = 0x00000015; // SIN = 1, MUX_MODE = 101 (GPIO4/9_IO06)
  port_iomuxc->offset21C = 0x000000B0; // PKE = 0 (disable pull-up/Keeper), Speed = 10 (Fast), DSE = 110, SRE = 0 (Slow)

  // #5 = D5, GPIO_EMC_08
  port_iomuxc->offset034 = 0x00000015; // SIN = 1, MUX_MODE = 101 (GPIO4/9_IO08)
  port_iomuxc->offset224 = 0x000000B0; // PKE = 0 (disable pull-up/Keeper), Speed = 10 (Fast), DSE = 110, SRE = 0 (Slow)
  port_gpio9->offset004 = 0x00000170; // GPIO9_IO04/05/06/08 = Output

  // #6 = D6, GPIO_B0_10
  port_iomuxc->offset164 = 0x00000015; // SIN = 1, MUX_MODE = 101 (GPIO2/7_IO10)
  port_iomuxc->offset354 = 0x0000A0B0; // PKE = 0 (disable pull-up/Keeper), PUS = 10 (100 kohm pull-up), Speed = 10 (Fast), DSE = 110, SRE = 0 (Slow)

  // #7 = D7, GPIO_B1_01
  port_iomuxc->offset180 = 0x00000015; // SIN = 1, MUX_MODE = 101 (GPIO2/7_IO17)
  port_iomuxc->offset370 = 0x0000A0B0; // PKE = 0 (disable pull-up/Keeper), PUS = 10 (100 kohm pull-up), Speed = 10 (Fast), DSE = 110, SRE = 0 (Slow)

  // #14 = A0, GPIO_AD_B1_02
  port_iomuxc->offset104 = 0x00000015; // SION = 1, MUX_MODE = 101 (GPIO1/6_IO18)
  port_iomuxc->offset2F4 = 0x00000000; // Disable Pull-up/Keeper
  port_adc1->CFG = 0x00000204;
  port_adc1->GC = 0x00000080; // Calibration

  // #15 = A1, GPIO_AD_B1_03
  port_iomuxc->offset108 = 0x00000015; // SION = 1, MUX_MODE = 101 (GPIO1/6_IO19)
  port_iomuxc->offset2F8 = 0x00000000; // Disable Pull-up/Keeper
  port_adc2->CFG = 0x00000204;
  port_adc2->GC = 0x00000080; // Calibration

  // Setup for SPI (8-bit)
  // Clock
  uintTMP = CCM_CBCMR;
  uintTMP &= 0xE3FFFFCF;
  uintTMP |= 0x0C000030;
  CCM_CBCMR = uintTMP; // LPSPI_PODF = 011 (div 4), LPSPI_CLK_SEL = 11 (PLL2_PFD2 = 396 MHz) => 99 MHz
  CCM_CCGR1 |= 0x000000C0; // CG3 = 11 (lpspi4_clk_enable)

  // #10 = CS, GPIO_B0_00
  port_iomuxc->offset13C = 0x00000013; // SIN = 1, MUX_MODE = 011 (LPSPI4_PCS0)
  port_iomuxc->offset32C = 0x000020B0; // HYS = 0, PUS = 00 (Pull-down 100 kohm), PUE = 1 (Pull), PKE = 0 (Disable Pull), ODE = 0 (Disable), SPEED = 10 (Fast), DSE = 110, SRE = 0 (Slow)
  port_iomuxc_b->offset11C = 0x00000000; // DAISY = 0 (GPIO_B0_00_ALT3)

  // #11 = MOSI, GPIO_B0_02
  port_iomuxc->offset144 = 0x00000013; // SIN = 1, MUX_MODE = 011 (LPSPI4_SDO)
  port_iomuxc->offset334 = 0x000020B0; // HYS = 0, PUS = 00 (Pull-down 100 kohm), PUE = 1 (Pull), PKE = 0 (Disable Pull), ODE = 0 (Disable), SPEED = 10 (Fast), DSE = 110, SRE = 0 (Slow)
  port_iomuxc_b->offset128 = 0x00000000; // DAISY = 0 (GPIO_B0_02_ALT3)

  // #12 = MISO, GPIO_B0_01
  port_iomuxc->offset140 = 0x00000013; // SIN = 1, MUX_MODE = 011 (LPSPI4_SDI)
  port_iomuxc->offset330 = 0x000020B0; // HYS = 0, PUS = 00 (Pull-down 100 kohm), PUE = 1 (Pull), PKE = 0 (Disable Pull), ODE = 0 (Disable), SPEED = 10 (Fast), DSE = 110, SRE = 0 (Slow)
  port_iomuxc_b->offset124 = 0x00000000; // DAISY = 0 (GPIO_B0_01_ALT3)

  // #13 = SCK, GPIO_B0_03
  port_iomuxc->offset148 = 0x00000013; // SIN = 1, MUX_MODE = 011 (LPSPI4_SCK)
  port_iomuxc->offset338 = 0x000020B0; // HYS = 0, PUS = 00 (Pull-down 100 kohm), PUE = 1 (Pull), PKE = 0 (Disable Pull), ODE = 0 (Disable), SPEED = 10 (Fast), DSE = 110, SRE = 0 (Slow)
  port_iomuxc_b->offset120 = 0x00000000; // DAISY = 0 (GPIO_B0_03_ALT3)

  // Setting
  spi_setup();

  // Setup for PWM (Clock)
  // Clock, ipg_clk_root = 108 MHz?
  CCM_CCGR4 |= 0x000C0000; // CG9 = 11 (pwm2_clk_enable)

  // #9 = PWM, GPIO_B0_11
  port_iomuxc->offset168 = 0x00000012; // SION = 1, MUX_MODE = 010 (FLEXPWM2_PWMB02)
  port_iomuxc->offset358 = 0x000020B0; // HYS = 0, PUS = 00 (Pull-down 100 kohm), PUE = 1 (Pull), PKE = 0 (Disable Pull), ODE = 0 (Disable), SPEED = 10 (Fast), DSE = 110, SRE = 0 (Slow)
  port_iomuxc_b->offset090 = 0x00000001; // DAISY = 1 (GPIO_B0_11_ALT2)

  // Setting
  port_pwm->MCTRL = 0x0400; // RUN = 0100 (enable clock to PWM generator of SM2)
  port_pwm->SM[2].CTRL2 = 0x2080; // INDEP = 1, FRCEN = 1
  port_pwm->SM[2].INIT = 0x0000; // Initial value = 0x0000
  port_pwm->SM[2].VAL0 = 0x8000; // Mid
  port_pwm->SM[2].VAL1 = 0xFFFF; // PWM Period
  port_pwm->SM[2].VAL2 = 0x0000; // = Initlal
  port_pwm->SM[2].VAL3 = 0x8000; // PWM Pulse Width for PWMA
  port_pwm->SM[2].VAL4 = 0x0000; // = Initial
  port_pwm->SM[2].VAL5 = 0x8000; // PWM Pulse Width for PWMB

  port_pwm->MCTRL |= 0x0004; // LDOK = 0100


  // for Pattern
  PAT_WD = 1000;
  PAT_LEN = 0;
  PAT_EN = 0;
  PAT_CNT = 0;
  PAT_RPT = 0;
  PAT_CYC = 0;

  for (i = 0; i < 256; i++) {
    PAT[i] = 0;
  }
}

void loop() {
  cntDAT = 0;

  while (Serial.available()) {
    delay(5);

    inDAT[cntDAT] = Serial.read();
    cntDAT = cntDAT + 1;

    if (cntDAT == 512) {
      break;
    }
  }

  if (cntDAT == 1) {
    REV_Code[0] = inDAT[0];
    Serial.write(REV_Code, 2);
  }

  if (cntDAT > 2) {
    CMD = inDAT[0];

    if (CMD == 0xAA) {
      LNG = inDAT[1];
      ADR = inDAT[2];

      RC = i2c_read(ADR, inDAT + 3, cntDAT - 3, outDAT, LNG);
      Serial.write(RC);

      if (RC == SC_SUCCESS) {
        for (i = 0; i < LNG; i++) {
          Serial.write(outDAT[i]);
        }
      }
    }
    else if (CMD == 0xA5) {
      ADR = inDAT[2];
      RC = i2c_write(ADR, inDAT + 3, cntDAT - 3);
      Serial.write(RC);
    }
    else if (CMD == 0x55 && cntDAT > 3) {
      Serial.write(SC_SUCCESS_GPIO);

      PAT_EN = 0;
      PAT_CNT = 0;
      PAT_RPT = 0;

      DLY = inDAT[1] * 256 + inDAT[2];
      PAT_WD = DLY;
      PAT_LEN = cntDAT - 3;

      for (i = 3; i < cntDAT; i++) {
        byteDAT = inDAT[i];
        PAT[i - 3] = byteDAT;

        CTRL_D72(byteDAT);

        delayMicroseconds(DLY);

        uintTMP = port_gpio7->offset008;
        byteDAT = (CK_BIT32(uintTMP, 17) << 5) | (CK_BIT32(uintTMP, 10) << 4);
        uintTMP = port_gpio9->offset008;
        byteDAT = byteDAT | (CK_BIT32(uintTMP, 8) << 3) | ((uintTMP & 0x00000070) >> 4);

        Serial.write(byteDAT);

        intAVAL[0] = gpio_analog_read(port_adc1, 14);
        intAVAL[1] = gpio_analog_read(port_adc2, 15);

        for (j = 0; j < 2; j++) {
          byteAVAL[2 * j] = intAVAL[j] / 256;
          byteAVAL[(2 * j) + 1] = intAVAL[j] % 256;
          Serial.write(byteAVAL[2 * j]);
          Serial.write(byteAVAL[(2 * j) + 1]);
        }
      }
    }
    else if (CMD == 0x5A) {
      Serial.write(SC_SUCCESS);

      SPI_BO = inDAT[1] & 0x20;
      SPI_BO = SPI_BO >> 5;

      SPI_CD = inDAT[1] & 0x1C;
      SPI_CD = SPI_CD >> 2;
      SPI_DM = inDAT[1] & 0x03;

      spi_reset();
      spi_command(SPI_BO, SPI_CD, SPI_DM, (8 * (cntDAT - 2)));
      spi_data(inDAT + 2, outDAT, cntDAT - 2);

      for (i = 0; i < cntDAT - 2; i++) {
        Serial.write(outDAT[i]);
      }
    }

    else if (CMD == 0xC3) {
      if (inDAT[1] > 0) {
        PAT_EN = 1;
        PAT_CYC = 0;
        PAT_RPT = inDAT[2];
        Serial.write(0xFA);
      }
      else {
        PAT_EN = 0;
        Serial.write(0xF0);
      }

    }
    else if (CMD == 0x69) {
      if (inDAT[1] == 0 && inDAT[2] == 0) {
        clk_en(false);
        Serial.write(0xFB);
      }
      else {
        if (cntDAT < 5) {
          uintTMP = ((inDAT[1] << 8) & 0x0000FF00) | (inDAT[2] & 0x000000FF);
          uintTMP = (uintTMP >> 1);
          inDAT[3] = (uintTMP >> 8) & 0xFF;
          inDAT[4] = uintTMP & 0xFF;
        }

        clk_set(inDAT + 1, inDAT + 3);

        delay(10);

        clk_en(true);

        Serial.write(0xFC);
      }
    }

    else if (CMD == 0x96) {
      Item = inDAT[1];
      Param = inDAT[2];

      switch (Item) {
        /*
          case 1: // Item = 1: analogReference
          switch(Param){
            case 0:
              analogReference(DEFAULT);
              break;
            case 1:
              analogReference(EXTERNAL);
              break;
            case 2:
              analogReference(INTERNAL);
              break;
          }
          Serial.write(SC_SUCCESS);
          break;
        */
        case 2: // Item = 2: SCK/SCL Pull-up
          switch (Param) {
            case 0:
              port_iomuxc->offset2EC &= ~(0x00001000); // PKE = 0
              port_iomuxc->offset2F0 &= ~(0x00001000); // PKE = 0
              break;
            case 1:
              port_iomuxc->offset2EC |= 0x00001000; // PKE = 1
              port_iomuxc->offset2F0 |= 0x00001000; // PKE = 1
              break;
          }
          Serial.write(SC_SUCCESS);
          break;
        /*
          case 3: // Item = 3: SCL Freq
          TWBR = Param;
          Serial.write(SC_SUCCESS);
          break;
        */
        case 4: // Item = 4: unit of delay time for pattern gen
          PAT_UNIT = Param;
          break;
      }
    }

  }

  if (PAT_EN == 1) {
    if (PAT_CNT >= PAT_LEN) {
      PAT_CNT = 0;
      PAT_CYC = PAT_CYC + 1;

      if ((PAT_CYC >= PAT_RPT) && (PAT_RPT != 0)) {
        PAT_EN = 0;
      }
    }

    byteDAT = PAT[PAT_CNT];
    CTRL_D72(byteDAT);
    PAT_CNT = PAT_CNT + 1;

    if (PAT_UNIT == 0) {
      delayMicroseconds(PAT_WD);
    }
    else {
      delayNanoseconds(PAT_WD);
    }
  }
  else {
    delay(100);
  }

}

byte i2c_write(byte slave_adr, byte *data, int data_length) {
  byte status_code;

  status_code = i2c_main(slave_adr, data, data_length);

  if (status_code == SC_SUCCESS) {
    if (i2c_wait_FIFO_TX(CONST_TIMEOUT, 4)) {
      i2c_stop();

      if (i2c_master_done(CONST_TIMEOUT)) {
        status_code = i2c_status_code();
      }
      else {
        status_code = SC_TIMEOUT;
      }
    }
    else {
      status_code = SC_TIMEOUT;
    }
  }

  if (status_code != SC_SUCCESS) {
    i2c_master_reset();
  }

  return status_code;
}

byte i2c_read(byte slave_adr, byte *reg_adr, int reg_adr_length, byte *read_data, int read_data_length) {
  byte status_code;

  status_code = i2c_main(slave_adr, reg_adr, reg_adr_length);

  if (i2c_wait_FIFO_TX(CONST_TIMEOUT, 1)) {
    status_code = i2c_status_code();

    if (status_code == SC_SUCCESS) {
      status_code = i2c_read_start(slave_adr, read_data, read_data_length);
    }

    if (status_code != SC_SUCCESS) {
      i2c_master_reset();
    }
  }
  else {
    status_code = i2c_status_code();
    i2c_master_reset();
  }

  return status_code;
}

byte i2c_main(byte slave_adr, byte *data, int data_length) {
  byte status_code;

  status_code = i2c_start(slave_adr, 0);

  if (status_code == SC_SUCCESS) {
    status_code = i2c_send_data(data, data_length);
  }

  return status_code;
}

//rw = 0: Write, 1: Read
byte i2c_start(byte slave_adr, int rw) {
  byte status_code;
  uint32_t pageTMP = 0x00000400;

  if (i2c_master_done(CONST_TIMEOUT)) {
    i2c_clear();

    if (i2c_bus_available(CONST_TIMEOUT)) {
      pageTMP = pageTMP | (slave_adr << 1) | rw;
      port_i2c->MTDR = pageTMP;

      status_code = SC_SUCCESS;
    }
    else {
      status_code = SC_TIMEOUT;
    }
  }
  else {
    status_code = SC_TIMEOUT;
  }

  return status_code;
}

void i2c_stop() {
  port_i2c->MTDR = 0x00000200;
}

byte i2c_send_data(byte *data, int data_length) {
  byte status_code = SC_SUCCESS;
  int i_i2c;
  uint32_t pageTMP;

  for (i_i2c = 0; i_i2c < data_length; i_i2c++) {

    if (i2c_wait_FIFO_TX(CONST_TIMEOUT, 4)) {
      pageTMP = data[i_i2c];
      port_i2c->MTDR = pageTMP;
    }
    else {
      status_code = SC_TIMEOUT;
      break;
    }
  }

  return status_code;
}

byte i2c_read_start(byte slave_adr, byte *read_data, int read_data_length) {
  int cnt_timeout = 0;
  int cnt_RX = 0;
  int cnt_TX = 0;
  uint32_t pageTMP = 0x00000400;
  byte status_code;

  pageTMP = pageTMP | (slave_adr << 1) | 1;
  port_i2c->MTDR = pageTMP;

  if (i2c_wait_FIFO_TX(CONST_TIMEOUT, 1)) {
    status_code = i2c_status_code();

    if (status_code == SC_SUCCESS) {

      while (cnt_RX < read_data_length) {

        if (cnt_TX < read_data_length) {
          if ((port_i2c->MFSR & 0x00000007) < 4) {
            port_i2c->MTDR = 0x00000100;
            cnt_TX++;
            cnt_timeout = 0;
          }
        }

        if ((port_i2c->MFSR & 0x00070000) > 0) {
          pageTMP = port_i2c->MRDR;
          read_data[cnt_RX] = pageTMP & 0xFF;
          cnt_RX++;
          cnt_timeout = 0;
        }

        delayMicroseconds(1);
        cnt_timeout++;

        if (cnt_timeout == 1000) {
          status_code = SC_TIMEOUT;
          break;
        }
      }

      if (status_code == SC_SUCCESS) {
        if (i2c_wait_FIFO_TX(CONST_TIMEOUT, 4)) {
          i2c_stop();

          if (i2c_master_done(CONST_TIMEOUT)) {
            status_code = i2c_status_code();
          }
          else {
            status_code = SC_TIMEOUT;
          }
        }
        else {
          status_code = SC_TIMEOUT;
        }
      }
    }
  }
  else {
    status_code = SC_TIMEOUT;
  }

  return status_code;
}

void i2c_master_reset() {
  port_i2c->MCR = 0x00000002;
  delay(1);
  port_i2c->MCR = 0x00000000;
  i2c_set_freq();
  port_i2c->MCR = 0x00000001;
}

void i2c_clear() {
  i2c_clear_FIFO();
  i2c_clear_FLAG();
}

void i2c_clear_FIFO() {
  port_i2c->MCR = 0x00000301;  // reset Tx/Rx FIFO
}

uint32_t i2c_clear_FLAG() {
  uint32_t pageTMP = port_i2c->MSR;
  port_i2c->MSR = 0x00007F00;

  return pageTMP;
}

// cnt_TX = 1, 2, 3 or 4.
bool i2c_wait_FIFO_TX(int timeout, uint32_t cnt_TX) {
  int cnt_timeout = 0;

  while (cnt_timeout < timeout) {
    delayMicroseconds(100);

    if ((port_i2c->MFSR & 0x00000007) < cnt_TX) {
      return true;
    }

    cnt_timeout++;
  }

  return false;
}

bool i2c_wait_FIFO_RX(int timeout) {
  int cnt_timeout = 0;

  while (cnt_timeout < timeout) {
    delayMicroseconds(100);

    if ((port_i2c->MFSR & 0x00070000) > 0) {
      return true;
    }

    cnt_timeout++;
  }

  return false;
}

bool i2c_master_done(int timeout) {
  int cnt_timeout = 0;

  while ((port_i2c->MSR & 0x01000000) > 0) {
    delayMicroseconds(100);

    if (cnt_timeout == timeout) {
      return false;
    }

    cnt_timeout++;
  }

  return true;
}

bool i2c_bus_available(int timeout) {
  int cnt_timeout = 0;

  while ((LPI2C1_MSR & 0x02000000) > 0) {
    delayMicroseconds(100);

    if (cnt_timeout == timeout) {
      i2c_clear();
      return false;
    }

    cnt_timeout++;
  }

  return true;
}

void i2c_set_freq() {
  // I2C Freq = 100 kHz
  port_i2c->MCCR0 = LPI2C_MCCR0_DATAVD(25) | LPI2C_MCCR0_SETHOLD(40) | LPI2C_MCCR0_CLKHI(55) | LPI2C_MCCR0_CLKLO(59);
  port_i2c->MCFGR1 = LPI2C_MCFGR1_PRESCALE(1);
  port_i2c->MCFGR2 = LPI2C_MCFGR2_FILTSDA(5) | LPI2C_MCFGR2_FILTSCL(5) | LPI2C_MCFGR2_BUSIDLE(3000); // BUSIDLE=3000(Idole timeout 250us)

  // Others
  port_i2c->MCCR1 = port_i2c->MCCR0;
  port_i2c->MFCR = LPI2C_MFCR_RXWATER(1) | LPI2C_MFCR_TXWATER(1);
}

byte i2c_status_code() {
  uint32_t pageTMP;

  delayMicroseconds(100);

  pageTMP = port_i2c->MSR;

  if ((pageTMP & 0x00000400) > 0) { // NACK
    return SC_NACK;
  }
  if ((pageTMP & 0x00002000) > 0) { // Pin Low Timeout
    return SC_TIMEOUT;
  }
  if ((pageTMP & 0x00001000) > 0) { // FIFO Error Flag
    return SC_TIMEOUT;
  }
  if ((pageTMP & 0x00000800) > 0) { // Arbitration Lost Flag
    return SC_TIMEOUT;
  }

  return SC_SUCCESS;
}

int gpio_analog_read(IMXRT_ADCS_t *port_adc, int pin_number) {
  uint32_t pageTMP = pin_number - 7;

  port_adc->HC0 = pageTMP;

  while ((port_adc->HS & 0x00000001) == 0) {
    // wait for the conversion
  }

  return  (port_adc->R0 & 0x3FF);
}

void CTRL_D72(byte byteDAT) {
  int HL_D7 = CK_BIT(byteDAT, 7);
  int HL_D6 = CK_BIT(byteDAT, 6);
  uint32_t pageTMP;

  pageTMP = (HL_D7 << 17) | (HL_D6 << 10);
  port_gpio7->offset004 = pageTMP; // set #7/#6 = GPIO2_IO17/10 direction

  if (CK_BIT(byteDAT, 5) == 1) {
    if (HL_D7 == 1) {
      port_gpio7->offset000 |= (0x00000001 << 17);
    }
    else {
      port_iomuxc->offset370 |= 0x00001000; // enable pull-up
    }
  }
  else {
    if (HL_D7 == 1) {
      port_gpio7->offset000 &= ~(0x00000001 << 17);
    }
    else {
      port_iomuxc->offset370 &= 0xFFFFEFFF; // disable pull-up
    }
  }

  if (CK_BIT(byteDAT, 4) == 1) {
    if (HL_D6 == 1) {
      port_gpio7->offset000 |= (0x00000001 << 10);
    }
    else {
      port_iomuxc->offset354 |= 0x00001000; // enable pull-up
    }
  }
  else {
    if (HL_D6 == 1) {
      port_gpio7->offset000 &= ~(0x00000001 << 10);
    }
    else {
      port_iomuxc->offset354 &= 0xFFFFEFFF; // disable pull-up
    }
  }

  pageTMP = port_gpio9->offset000;
  pageTMP &= 0xFFFFFE8F;
  pageTMP |= ((CK_BIT(byteDAT, 3) << 8) | ((byteDAT & 0x07) << 4));
  port_gpio9->offset000 = pageTMP;
}

int CK_BIT(byte byteDAT, int intDIG) {
  byte byteTMP = 1;

  if (intDIG < 8) {
    byteTMP = byteTMP << intDIG;
  }
  else {
    byteTMP = byteTMP << 7;
  }

  if ((byteDAT & byteTMP) > 0) {
    return 1;
  }
  else {
    return 0;
  }
}

int CK_BIT32(uint32_t pageDAT, int intDIG) {
  uint32_t pageTMP = 1;

  if (intDIG < 32) {
    pageTMP = pageTMP << intDIG;
  }
  else {
    pageTMP = pageTMP << 31;
  }

  if ((pageDAT & pageTMP) > 0) {
    return 1;
  }
  else {
    return 0;
  }
}

void spi_setup() {
  port_spi->offset024 |= 0x00000001; // MASTER = 1 (Master mode)
  port_spi->offset010 |= 0x00000001; // MEN = 1 (Module is enabled)
}

void spi_reset() {
  while ((port_spi->offset014 & 0x01000000) > 0) {
  }

  port_spi->offset010 = 0x00000002;
  delay(1);
  port_spi->offset010 = 0x00000000;

  spi_setup();
}

void spi_command(byte byteSPI_BO, byte byteSPI_CD, byte byteSPI_DM, int intSPI_SIZE) {
  uint32_t pageTMP = port_spi->offset060;

  pageTMP &= ~(0xF8800000);

  if (byteSPI_BO == 1) {
    pageTMP |= 0x00800000; // LSBF = 1 (LSB first)
  }

  switch (byteSPI_CD) { // Original SCK = LPSPI_Clock (99 MHz) / 2 = 49.5 MHz
    case 0:
      pageTMP |= (0x00000000 << 27); // PRESCALE = 000 (Divide by 1)
      break;
    case 1:
      pageTMP |= (0x00000001 << 27); // PRESCALE = 001 (Divide by 2)
      break;
    case 2:
      pageTMP |= (0x00000002 << 27); // PRESCALE = 010 (Divide by 4)
      break;
    case 3:
      pageTMP |= (0x00000003 << 27); // PRESCALE = 011 (Divide by 8)
      break;
    case 4:
      pageTMP |= (0x00000004 << 27); // PRESCALE = 100 (Divide by 16)
      break;
    case 5:
      pageTMP |= (0x00000005 << 27); // PRESCALE = 101 (Divide by 32)
      break;
    case 6:
      pageTMP |= (0x00000006 << 27); // PRESCALE = 110 (Divide by 64)
      break;
    default:
      pageTMP |= (0x00000007 << 27); // PRESCALE = 111 (Divide by 128)
      break;
  }

  switch (byteSPI_DM) {
    case 0:
      pageTMP |= 0x00000000; // CPOL = 0, CPHA = 0
      break;
    case 1:
      pageTMP |= 0x01000000; // CPOL = 0, CPHA = 1
      break;
    case 2:
      pageTMP |= 0x11000000; // CPOL = 1, CPHA = 1
      break;
    case 3:
      pageTMP |= 0x10000000; // CPOL = 1, CPHA = 0
      break;
  }

  if (intSPI_SIZE < 8) {
    pageTMP |= 0x00000007;
  }
  else if (intSPI_SIZE >= 4096) {
    pageTMP |= 0x00000FFF;
  }
  else {
    pageTMP |= ((intSPI_SIZE - 1) & 0x00000FFF);
  }

  port_spi->offset060 = pageTMP;
}

void spi_data(byte *send_data, byte *read_data, byte send_data_length) {
  int cnt_send = 0;
  int cnt_read = 0;
  int i_spi;
  int word_ready = 0;
  int last = 0;
  uint32_t pageSEND = 0;
  uint32_t pageREAD;

  while (cnt_read < send_data_length) {
    if ((word_ready == 0) && (last == 0)) {
      for (i_spi = 0; i_spi < 4; i_spi++) {
        pageSEND |= (send_data[cnt_send] << (8 * (3 - i_spi)));
        cnt_send++;
        if (cnt_send == send_data_length) {
          last = 1;
          break;
        }
      }

      word_ready = 1;
    }

    if (word_ready == 1) {
      if ((port_spi->offset05C & 0x0000001F) < 16) {
        port_spi->offset064 = pageSEND;
        pageSEND = 0;
        word_ready = 0;
      }
    }

    if ((port_spi->offset05C & 0x001F0000) > 0) {
      pageREAD = port_spi->offset074;

      for (i_spi = 0; i_spi < 4; i_spi++) {
        read_data[cnt_read] = (pageREAD >> (8 * (3 - i_spi))) & 0xFF;
        cnt_read++;

        if (cnt_read == send_data_length) {
          break;
        }
      }
    }
  }
}


void clk_en(boolean cEN) {
  if (cEN) {
    port_pwm->OUTEN |= 0x0040;
  }
  else {
    port_pwm->OUTEN &= ~(0x0040);
  }
}

void clk_set(byte *pwm_period, byte *pwm_width) {
  uint16_t pageTOP;
  uint16_t pageMID;
  uint16_t pageWIDTH;

  pageTOP = (pwm_period[0] << 8) | (pwm_period[1] & 0xFF);
  pageMID = (pageTOP >> 1);
  pageWIDTH = (pwm_width[0] << 8) | (pwm_width[1] & 0xFF);

  // port_pwm->SM[2].INIT = 0x0000; // Initial value = 0x0000
  port_pwm->SM[2].VAL0 = pageMID; // Mid
  port_pwm->SM[2].VAL1 = pageTOP; // PWM Period
  // port_pwm->SM[2].VAL2 = 0x0000; // = Initlal
  port_pwm->SM[2].VAL3 = pageWIDTH; // PWM Pulse Width for PWMA
  // port_pwm->SM[2].VAL4 = 0x0000; // = Initial
  port_pwm->SM[2].VAL5 = pageWIDTH; // PWM Pulse Width for PWMB

  port_pwm->MCTRL |= 0x0004; // LDOK = 0100
}
