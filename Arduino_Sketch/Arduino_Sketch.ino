//import processing.serial.*;
// Pin Mapping

int done = 0;
const int t_trig = 23;
const int sdiA = 27;
const int ssbA = 29;
const int FSMEN = 31;
const int sclkA = 33;
const int sdoA = 35;
const int gRSTM = 21;
const int gRSTA = 37;
const int noiseInj = 39;
const int dpwm = 43;
const int sdiM = 45;
const int sclkM = 47;
const int sdoM = 49;
const int ssbM = 51;
const int rdEN = 53;
const int tempPin = 46;

// Clock variables
const double clockPeriod = 0.002;
const double delayPeriod = 0.001;

// Variables
String ptString = "";
String keyString = "";
byte config_adr; byte adr; byte config_data;
String inputString = "";         // a string to hold incoming data
String inputString4 = "";         // a string to hold incoming data
String inputString1 = "";         // a string to hold incoming data
String inputString2 = "";         // a string to hold incoming data
String inputString3 = "";         // a string to hold incoming data
String oldString="";
int countx=0;
String vref;
int dvfs_count=0;

void loop() {
  char receivedChar;
  while (Serial.available()) {
    char receivedChar = Serial.read();
    inputString += receivedChar;
    countx = countx + 1; 
    if (countx<=32)
    {
      inputString1 += receivedChar;
    }
    else if (countx<=64)
    {
      inputString2 += receivedChar;
    }
    else if (countx<=96)
    {
      inputString3 += receivedChar;
    }
    else
    {
      inputString4 += receivedChar;
    }
//    Serial.print(inputString1);Serial.print("\n");
    if (receivedChar == '\n') {
      oldString=inputString;
          if (dvfs_count%40==0) {
      int mode_cntrl=0;   writeMSPI(mode_cntrl); // 0=normal, 1=SNI, 2=RVREF, 3=SNI+R-VREF;
    } else if (dvfs_count%40==10) {
      int mode_cntrl=1;   writeMSPI(mode_cntrl); // 0=normal, 1=SNI, 2=RVREF, 3=SNI+R-VREF;
    } else if (dvfs_count%40==20) {
      int mode_cntrl=2;   writeMSPI(mode_cntrl); // 0=normal, 1=SNI, 2=RVREF, 3=SNI+R-VREF;
    } else if (dvfs_count%40==30) {
      int mode_cntrl=3;   writeMSPI(mode_cntrl); // 0=normal, 1=SNI, 2=RVREF, 3=SNI+R-VREF;
    }
      dvfs_count=dvfs_count+1;
      Serial.print("\n");
      resetA();
    config_adr = B00000001; config_data = B11000000; spi_out(config_adr, config_data); digitalChipWait(); //clkpostdiv(2'b11), clkdiv(2'b11), clken(2'b00), sdo_control(2'b00).
    config_adr = B00000010; config_data = B00000101; spi_out(config_adr, config_data); digitalChipWait(); //none (2'b00), EncRd5 (1) or EncRd9 (0), rd_config_en, en_adaptclk, clkrstb (1'b1), enc_dec, rd_key_data.
    config_adr = B00000100; config_data = B00000000; spi_out(config_adr, config_data); digitalChipWait(); // none  (4'00), cfg_attack_byte(config2[3:0]
      adr = B01000000; ptString = inputString; spi_out128(adr, inputString); digitalChipWait(); // ciphertext = 7FE2B913744937A220A77310DCB475A9
      adr = B10000000; keyString = "1F8E3BE2D4372D788CD2039FA8FED757"; spi_out128(adr, keyString); digitalChipWait();
      digitalWrite(FSMEN, HIGH); 
      digitalWrite(FSMEN, LOW);
      config_adr = B00000100; config_data = B01010101; digitalChipRead2(config_adr, config_data);
      inputString = "";inputString1 = "";inputString2 = "";inputString3 = "";inputString4 = "";countx=0;
    }
//        delay(50);
  }
}

void setup() {
  // put your setup code here, to run once:
  pinMode(noiseInj, OUTPUT);
  pinMode(gRSTA, OUTPUT);
  pinMode(FSMEN, OUTPUT);
  pinMode(ssbA, OUTPUT);
  pinMode(sclkA, OUTPUT);
  pinMode(sdiA, OUTPUT);
  pinMode(tempPin, OUTPUT);
  pinMode(sclkM, OUTPUT);
  pinMode(sdiM, OUTPUT);
  pinMode(sdoM, INPUT);
  pinMode(ssbM, OUTPUT);
  pinMode(gRSTM, OUTPUT);
  pinMode(rdEN, OUTPUT);
  pinMode(dpwm, OUTPUT);
  pinMode(sdoA, INPUT);

  // default
  digitalWrite(gRSTA, HIGH);
  digitalWrite(FSMEN, LOW);
  digitalWrite(ssbA, HIGH);
  digitalWrite(sclkA, LOW);
  digitalWrite(sdiA, LOW);
  digitalWrite(noiseInj, LOW);
  digitalWrite(sclkM, LOW);
  digitalWrite(sdiM, LOW);
  digitalWrite(ssbM, HIGH);
  digitalWrite(gRSTM, LOW);
  digitalWrite(dpwm, HIGH);
  digitalWrite(rdEN, LOW);
  digitalWrite(tempPin, LOW);

  vref="10101";
  writeMSPI(1); // 0=normal, 1=SNI, 2=RVREF, 3=SNI+R-VREF;
  resetA();
  //outputFile=createWriter("ciphertext.txt");

  Serial.begin(115200);
    config_adr = B00000001; config_data = B11000000; spi_out(config_adr, config_data); digitalChipWait(); //clkpostdiv(2'b11), clkdiv(2'b11), clken(2'b00), sdo_control(2'b00).
    config_adr = B00000010; config_data = B00000101; spi_out(config_adr, config_data); digitalChipWait(); //none (2'b00), EncRd5 (1) or EncRd9 (0), rd_config_en, en_adaptclk, clkrstb (1'b1), enc_dec, rd_key_data.
    config_adr = B00000100; config_data = B00000000; spi_out(config_adr, config_data); digitalChipWait(); // none  (4'00), cfg_attack_byte(config2[3:0]
}

void spi_transfer(byte input) {
  for (int i = 1; i <= 8; i++) {
    if (input > 127) {
      digitalWrite(sclkA, HIGH);
      digitalWrite(sdiA, HIGH);
    }
    else {
      digitalWrite(sclkA, HIGH);
      digitalWrite(sdiA, LOW);


    }
    delay(clockPeriod / 2);
    digitalWrite(sclkA, LOW);
    delay(clockPeriod / 2);
    input = input << 1;
  }
}

void spi_transfer2(byte input) {
  for (int i = 1; i <= 8; i++) {
    if (input > 127) {
      digitalWrite(sclkM, HIGH);
      digitalWrite(sdiM, HIGH);
    }
    else {
      digitalWrite(sclkM, HIGH);
      digitalWrite(sdiM, LOW);
    }
    delay(clockPeriod / 2);
    digitalWrite(sclkM, LOW);
    delay(clockPeriod / 2);
    input = input << 1;
  }
}

byte spi_transferread(byte input) {
  //  for (int i = 1; i <= 8; i++) {
  //    if (input > 127) {
  //      digitalWrite(sclkM, HIGH);
  //      digitalWrite(sdiM, HIGH);
  //    }
  //    else {
  //      digitalWrite(sclkM, HIGH);
  //      digitalWrite(sdiM, LOW);
  //    }
  //    delay(clockPeriod / 2);
  //    digitalWrite(sclkM, LOW);
  //    delay(clockPeriod / 2);
  //    input = input << 1;
  //  }
  uint8_t output = 0x00;
  //char output[8];
  for (int i = 8; i > 0; i--) {
    output = output << 1;
    digitalWrite(sclkM, HIGH);
    delay(clockPeriod / 2);
    digitalWrite(sclkM, LOW);
    delay(clockPeriod / 2);
    if (digitalRead(sdoM) == HIGH) {
      Serial.print(1);
      output = output | 0x01;
    } else if (digitalRead(sdoM) == LOW) {
      Serial.print(0);
      output = output | 0x00;
    }
    Serial.print(" ");
  }
  Serial.println("");
  //  Serial.println(output, HEX);
  return output;
}

void spi_transferLSBFIRST(byte input) {
  uint8_t mask = 0x01;
  for (int i = 8; i > 0; i--) {
    digitalWrite(sclkA, HIGH);
    digitalWrite(sdiA, input & mask);
    delay(clockPeriod / 2);
    digitalWrite(sclkA, LOW);
    delay(clockPeriod / 2);
    mask = mask << 1;

  }
}

void spi_out(byte config_adr, byte config_data) {
  digitalWrite(ssbA, LOW);
  spi_transferLSBFIRST(config_adr); spi_transferLSBFIRST(config_data);
  digitalWrite(ssbA, HIGH);
}

void spi_out128(byte adr, String data) {
  digitalWrite(ssbA, LOW);
  spi_transferLSBFIRST(adr);
  for (int i = 31; i > 0; i = i - 2) {
    byte data_out = getVal(data[i]) + (getVal(data[i - 1]) << 4);
    spi_transferLSBFIRST(data_out);
    //Serial.println(data_out, BIN);
    //    delay(100);
  }
//  delay(0.050);
  digitalWrite(ssbA, HIGH);
}

void digitalChipWait() {
  for (int i = 1; i <= 4; i++) {
    digitalWrite(sclkA, HIGH);
    delay(clockPeriod / 2);
    digitalWrite(sclkA, LOW);
    delay(clockPeriod / 2);
  }
}

byte spi_transferLSBFIRST2(byte input) {
  uint8_t mask = 0x01;
  uint8_t output = 0x00;
  //char output[8];
  for (int i = 8; i > 0; i--) {
    output = output << 1;
    digitalWrite(sclkA, HIGH);
    digitalWrite(sdiA, input & mask);
    delay(clockPeriod / 2);
    digitalWrite(sclkA, LOW);
    delay(clockPeriod / 2);
    if (digitalRead(sdoA) == HIGH) {
      output = output | 0x01;
      //Serial.print(1);
    } else if (digitalRead(sdoA) == LOW) {
      output = output | 0x00;
      //Serial.print(0);
    }
    mask = mask << 1;
  }
  return output;
}

byte spi_transferLSBFIRST3() {
  uint8_t mask = 0x01;
  uint8_t output = 0x00;
  //char output[8];
  for (int i = 8; i > 0; i--) {
    output = output << 1;
    if (digitalRead(sdoM) == HIGH) {
      output = output | 0x01;
      //Serial.print(1);
    } else if (digitalRead(sdoM) == LOW) {
      output = output | 0x00;
      //Serial.print(0);
    }
    mask = mask << 1;
  }
  return output;
}

void digitalChipRead3(byte config_adr, byte config_data) {
  for (int i = 1; i <= 10; i++) {
    digitalWrite(sclkM, HIGH);
    delay(clockPeriod / 2);
    digitalWrite(sclkM, LOW);
    delay(clockPeriod / 2);
  }
  uint8_t temp;
  digitalWrite(ssbM, LOW);
  //temp = spi_transferLSBFIRST2(config_adr);
  Serial.print(temp, HEX);
  //temp = spi_transferLSBFIRST2(config_data);
  Serial.print(temp, HEX);
  Serial.println("");
  //delay(0.2);
  digitalWrite(ssbM, HIGH);
}

void digitalChipRead2(byte config_adr, byte config_data) {
  for (int i = 1; i <= 10; i++) {
    digitalWrite(sclkA, HIGH);
    delay(clockPeriod / 2);
    digitalWrite(sclkA, LOW);
    delay(clockPeriod / 2);
  }
  String* ciphertext[16];
  uint8_t temp;
  digitalWrite(ssbA, LOW);
  for (int j = 1; j <= 8; j++) {
    //digitalWrite(sclkA, HIGH);
    //delay(clockPeriod / 2);
    //digitalWrite(sclkA, LOW);
    //delay(clockPeriod / 2);
    //int val=digitalRead(sdoA);
    temp = spi_transferLSBFIRST2(config_adr);
//    Serial.print(temp, HEX);
    PrintHex8(temp, 8);
    temp = spi_transferLSBFIRST2(config_data);
    //Serial.print(temp, HEX);
    PrintHex8(temp, 8);
  }
  //  for (int i=1; i<=8, i++) {
  //    if(temp{i}=='0') {
  //      Serial.print(0);
  //    }
  //    else if(temp{i}=='1'){
  //      Serial.print(1);
  //    }
  //  }
  //
  //  for (int i=1; i<=8, i++) {
  //    Serial.println(temp[i]);
  //
  //  }
  Serial.println("");
  //delay(0.2);
  digitalWrite(ssbA, HIGH);
}

void digitalChipRead() {
  for (int i = 1; i <= 4; i++) {
    digitalWrite(sclkA, HIGH);
    delay(clockPeriod / 2);
    digitalWrite(sclkA, LOW);
    delay(clockPeriod / 2);
  }
  digitalWrite(ssbA, LOW);
  for (int i = 1; i <= 128; i++) {
    digitalWrite(sclkA, HIGH);
    delay(clockPeriod / 2);
    digitalWrite(sclkA, LOW);
    delay(clockPeriod / 2);
    //int val=digitalRead(sdoA);
  }
  digitalWrite(ssbA, HIGH);
}

void digitalChipRead128() {
  for (int i = 1; i <= 200; i++) {
    digitalWrite(sclkA, HIGH);
    delay(clockPeriod / 2);
    digitalWrite(sclkA, LOW);
    delay(clockPeriod / 2);
  }
  digitalWrite(ssbA, LOW);
  for (int i = 1; i <= 100000; i++) {
    digitalWrite(sclkA, HIGH);
    delay(clockPeriod / 2);
    digitalWrite(sclkA, LOW);
    delay(clockPeriod / 2);
    //int val=digitalRead(sdoA);
  }
  digitalWrite(ssbA, HIGH);
}

byte getVal(char c)
{
  if (c >= '0' && c <= '9')
    return (byte)(c - '0');
  else
    return (byte)(c - 'A' + 10);
}

int writeMSPI(int mode_cntrl) {
  // reset
//  int mode_cntrl=strToInt(mode);
  reset_IVR(gRSTM);
  //  init_DPWM_IVR(dpwm);

//  delay(1);

  Serial.begin(115200);

  byte CONFIG_ADR_M; byte CONFIG;

  //  digitalChipReadMSPI(20);

  Serial.begin(115200);
  String kp = "00001"; String kp_addr = "000000"; //0 - ki_base[2:0] (01010, 10), kp_base=00111 (7)
  String ki = "00001"; String ki_other = "000001"; String ki_addr = "000001"; //1 - ki_base[2:0] (01010, 10), kp_base=00111 (7)
  String kd = "0001"; String kd_other = "1111"; String kd_addr = "000111";  //7 - kd_base (0010, 2), ki_up[4:1] (11111, 31)
  String vref = "0001"; String vref_other = "0001"; String vref_addr = "001101"; // 13 - cfg_vref_base (1000, 8),cfg_en_hvt_pfet (0),cfg_ss_clk_sel (0),cfg_dm_en (1),cfg_en_coarse_cntrl (1)
  String load = "0100"; String load_other = "1111"; String load_addr = "001010"; //10 - cfg_load_set (0101, 5), kd_up (1111, 15)
  String load_tuning = "00000000"; String load_low = "0010"; String load_high = "0100"; String load_tuning_addr = "001011"; //11 - cfg_load_high (1010, 10), cfg_load_low (0001, 1)

 // reference and load settings
//    int kp_int = 22; int ki_int = 24; int kd_int = 9; int vref_int = 9; int load_1 = 5; int load_2 = load_1; // DM (TUNED, wo DM) -- 1
    int kp_int = 10; int ki_int = 7; int kd_int = 2; int vref_int = 10; int load_1 = 5; int load_2 = load_1; // DM (TUNED, wo DM) -- 1

  int en_tuning = 0; int en_iter = 0; int en_load_jump = 0;

  // Loop Delay
  int loopDelay = 1; int tuneInterval = 1000; //ms - Approximate tuning delay is 16.8us (128 cycles for 32*32*16, half the clock speed)
  int i1_max, i2_max, i3_max;
  // Iterate over - Max, Min; i1 = kp, i2 = ki, i3 = kd
  if (en_iter == 1) {
    i1_max = 32; i2_max = 32; i3_max = 16;
  } else {
    i1_max = 2; i2_max = 2; i3_max = 2;
  }
  int i1_min = 1; int i2_min = 1; int i3_min = 1;
  int step1 = 4; int step2 = 2;

  for (int i1 = i1_min; i1 < i1_max; i1 = i1 + step1) {
    for (int i2 = i2_min; i2 < i2_max; i2 = i2 + step1) {
      for (int i3 = i3_min; i3 < i3_max; i3 = i3 + step2) {
        if (en_iter == 1) {
          kp_int = i1; ki_int = i2; kd_int = i3; // Choose the loop variables
        }
        kp = String(kp_int, BIN); int kp_len = kp.length(); kp = genPad(kp, 5) + kp;
        ki = String(ki_int, BIN); int ki_len = ki.length(); ki = genPad(ki, 5) + ki;
        kd = String(kd_int, BIN); int kd_len = kd.length(); kd = genPad(kd, 4) + kd;
        vref = String(vref_int, BIN); int vref_len = vref.length(); vref = genPad(vref, 4) + vref;
        load = String(load_1, BIN); int load_len = load.length(); load = genPad(load, 4) + load;
        load_low = String(load_1, BIN); load_high = String(load_2, BIN); load_tuning = genPad(load_high, 4) + load_high + genPad(load_low, 4) + load_low;

//        Serial.println(kp_int); Serial.println(ki_int); Serial.println(kd_int);
        byte CONFIG_ADR_M; byte CONFIG;

        String kp_addr_m = kp.substring(3, 5) + kp_addr; String kp_cfg =  genPad(ki.substring(2, 5) + kp.substring(0, 3), 8) + ki.substring(2, 5) + kp.substring(0, 3);
        byte kp_addr_m_b = byte(strToInt(kp_addr_m)); byte kp_cfg_b = byte(strToInt(kp_cfg));
        spi_out2(kp_cfg_b, kp_addr_m_b); internalRead();
        //Serial.println(b1_cfg_b, BIN); Serial.println(b1_addr_m_b, BIN);

        String ki_addr_m = ki.substring(0, 2) + ki_addr; String ki_cfg =  genPad(ki_other, 8) + ki_other;
        byte ki_addr_m_b = byte(strToInt(ki_addr_m)); byte ki_cfg_b = byte(strToInt(ki_cfg));
        spi_out2(ki_cfg_b, ki_addr_m_b); internalRead();

        String kd_addr_m = kd_other.substring(2, 4) + kd_addr; String kd_cfg =  genPad(kd + kd_other.substring(0, 2), 8) + kd + kd_other.substring(0, 2);
        byte kd_addr_m_b = byte(strToInt(kd_addr_m)); byte kd_cfg_b = byte(strToInt(kd_cfg));
        spi_out2(kd_cfg_b, kd_addr_m_b); internalRead();

        String vref_addr_m = vref_other.substring(2, 4) + vref_addr; String vref_cfg =  genPad(vref + vref_other.substring(0, 2), 8) + vref + vref_other.substring(0, 2);
        byte vref_addr_m_b = byte(strToInt(vref_addr_m)); byte vref_cfg_b = byte(strToInt(vref_cfg));
        spi_out2(vref_cfg_b, vref_addr_m_b); internalRead();

        String load_addr_m = load_other.substring(2, 4) + load_addr; String load_cfg =  genPad(load + load_other.substring(0, 2), 8) + load + load_other.substring(0, 2);
        byte load_addr_m_b = byte(strToInt(load_addr_m)); byte load_cfg_b = byte(strToInt(load_cfg));
        spi_out2(load_cfg_b, load_addr_m_b); internalRead();

        String load_tuning_addr_m = load_tuning.substring(6, 8) + load_tuning_addr; String load_tuning_cfg =  genPad(load_tuning.substring(0, 6), 8) + load_tuning.substring(0, 6);
        byte load_tuning_addr_m_b = byte(strToInt(load_tuning_addr_m)); byte load_tuning_cfg_b = byte(strToInt(load_tuning_cfg));
        spi_out2(load_tuning_cfg_b, load_tuning_addr_m_b); internalRead();

        CONFIG_ADR_M = B10000010; CONFIG = B00011011; //2 - kp_m[3:0] (00111), ki_l[4:1] (00110)
        spi_out2(CONFIG, CONFIG_ADR_M); internalRead();
        CONFIG_ADR_M = B10000011; CONFIG = B00101110; //3 - kp_h[1:0] (01011),ki_m (01010),kp_m[4](00111)
        spi_out2(CONFIG, CONFIG_ADR_M); internalRead();
        CONFIG_ADR_M = B01000100; CONFIG = B00111010; //4 - ki_h (01110),kp_h[4:2] (01011)
        spi_out2(CONFIG, CONFIG_ADR_M); internalRead();
        CONFIG_ADR_M = B01000101; CONFIG = B00111000; //5 - kp_up[2:0] (11111, 31),kp_low (00101, 5)
        spi_out2(CONFIG, CONFIG_ADR_M); internalRead();
        CONFIG_ADR_M = B11000110; CONFIG = B00100001; //6 - ki_up[0] (11111, 31),ki_low (00101, 5),kp_up[4:3] (11111, 31)
        spi_out2(CONFIG, CONFIG_ADR_M); internalRead();
        CONFIG_ADR_M = B01001000; CONFIG = B00010100; //8 - kd_m (0010, 2), kd_l (0000, 0)
        spi_out2(CONFIG, CONFIG_ADR_M); internalRead();
        CONFIG_ADR_M = B01001001; CONFIG = B00000101; //9 - kd_low (0101, 5), kd_h (0110, 6)
        spi_out2(CONFIG, CONFIG_ADR_M); internalRead();
        CONFIG_ADR_M = B00001100; CONFIG = B00000011; //12 - cfg_en_fine_cntrl (0),cfg_tol_reg_err (0), cfg_en_adapt_load (0), cfg_en_autotuning (0),cfg_sense_ctrl (10), cfg_adc_res_sel (00)
        spi_out2(CONFIG, CONFIG_ADR_M); internalRead();
        CONFIG_ADR_M = B00001110; CONFIG = B00100110; //14 - cfg_vref2 (0010, 2),cfg_vref1 (0001, 1)
        spi_out2(CONFIG, CONFIG_ADR_M); internalRead();
        CONFIG_ADR_M = B10001111; CONFIG = B00101110; //15 - cfg_vref4 (0100, 4),cfg_vref3 (0011, 3)
        spi_out2(CONFIG, CONFIG_ADR_M); internalRead();
        CONFIG_ADR_M = B00010000; CONFIG = B00110111; //16 - cfg_vref6 (0110, 4),cfg_vref5 (0101, 5)
        spi_out2(CONFIG, CONFIG_ADR_M); internalRead();
        CONFIG_ADR_M = B00010001; CONFIG = B00100110; //17 - cfg_vref8 (1000, 8),cfg_vref7 (0111, 7)
        spi_out2(CONFIG, CONFIG_ADR_M); internalRead();
        CONFIG_ADR_M = B10010010; CONFIG = B00101110; //18 - cfg_vref10 (1010, 10),cfg_vref9 (1001, 9)
        spi_out2(CONFIG, CONFIG_ADR_M); internalRead();
        CONFIG_ADR_M = B00010011; CONFIG = B00110111; //19 - cfg_vref12 (1100, 12),cfg_vref11 (1011, 11)
        spi_out2(CONFIG, CONFIG_ADR_M); internalRead();
        CONFIG_ADR_M = B00010100; CONFIG = B00100110; //20 - cfg_vref14 (1110, 14),cfg_vref13 (1101, 13)
        spi_out2(CONFIG, CONFIG_ADR_M); internalRead();
//        CONFIG_ADR_M = B10010101; CONFIG = B00011010; //21 - cfg_en_lfsr_as_vref (0),cfg_en_vref_lfsr (0),cfg_sel_vref_lfsr_clk (00),cfg_vref15 (1111, 15)
        if (mode_cntrl==1) {
        CONFIG_ADR_M = B10010101; CONFIG = B00001010; //21 - cfg_en_lfsr_as_vref (0),cfg_en_vref_lfsr (0),cfg_sel_vref_lfsr_clk (00),cfg_vref15 (1111, 15)
        } else if (mode_cntrl==2) {
        CONFIG_ADR_M = B10010101; CONFIG = B00011010; //21 - cfg_en_lfsr_as_vref (0),cfg_en_vref_lfsr (0),cfg_sel_vref_lfsr_clk (00),cfg_vref15 (1111, 15)
        } else if (mode_cntrl==3) {
        CONFIG_ADR_M = B10010101; CONFIG = B00011010; //21 - cfg_en_lfsr_as_vref (0),cfg_en_vref_lfsr (0),cfg_sel_vref_lfsr_clk (00),cfg_vref15 (1111, 15)
        } else {
        CONFIG_ADR_M = B10010101; CONFIG = B00001010; //21 - cfg_en_lfsr_as_vref (0),cfg_en_vref_lfsr (0),cfg_sel_vref_lfsr_clk (00),cfg_vref15 (1111, 15)
        }
        spi_out2(CONFIG, CONFIG_ADR_M); internalRead();
        CONFIG_ADR_M = B00010110; CONFIG = B00000000; //22 - cfg_bs_gain (00),cfg_adc_tune (0010),cfg_adc_tune_coarse (01)
        spi_out2(CONFIG, CONFIG_ADR_M); internalRead();
        CONFIG_ADR_M = B00010111; CONFIG = B00000000; //23 - cfg_se_wt (00),cfg_ae_wt (00),cfg_tr_clk_div (00),cfg_ss_clk_div (00)
        spi_out2(CONFIG, CONFIG_ADR_M); internalRead();
        if (mode_cntrl==1) {
          CONFIG_ADR_M = B00011000; CONFIG = B00110001; //24 - sel_pulse_width_ext (00),sel_pulse_width_lfsr_clk (00),en_pulse_width_lfsr (0), cfg_en_sw_pulse (0), cfg_ct_wt (00)
        } else if (mode_cntrl==2) {
          CONFIG_ADR_M = B00011000; CONFIG = B00110000; //24 - sel_pulse_width_ext (00),sel_pulse_width_lfsr_clk (00),en_pulse_width_lfsr (0), cfg_en_sw_pulse (0), cfg_ct_wt (00)
        } else if (mode_cntrl==3) {
          CONFIG_ADR_M = B00011000; CONFIG = B00110001; //24 - sel_pulse_width_ext (00),sel_pulse_width_lfsr_clk (00),en_pulse_width_lfsr (0), cfg_en_sw_pulse (0), cfg_ct_wt (00)
        } else {
          CONFIG_ADR_M = B00011000; CONFIG = B00110000; //24 - sel_pulse_width_ext (00),sel_pulse_width_lfsr_clk (00),en_pulse_width_lfsr (0), cfg_en_sw_pulse (0), cfg_ct_wt (00)
        }
        spi_out2(CONFIG, CONFIG_ADR_M); internalRead();
        CONFIG_ADR_M = B01011001; CONFIG = B00100110; //25 - cfg_ad_load_set_m[0] (1010, 10),cfg_ad_load_set_l (0001, 1),hyst_width_stable (01),Auto_Load_Jump_EN (0)
        spi_out2(CONFIG, CONFIG_ADR_M); internalRead();
        CONFIG_ADR_M = B10011010; CONFIG = B00001010; //26 - cfg_early_dm[3:0] (0100, 4),cfg_sel_skew (1),cfg_ad_load_set_m[3:1] (1010, 10)
        spi_out2(CONFIG, CONFIG_ADR_M); internalRead();
        CONFIG_ADR_M = B10100100; CONFIG = B00000001; //36  - 3'd0, cfg_sel_cap (0010), (cfg_sel_cap [1] = AES_drive), sel_load_cap_sw = cfg_sel_cap[0], cfg_early_dm[4]=0;
        spi_out2(CONFIG, CONFIG_ADR_M); internalRead();
        CONFIG_ADR_M = B11100101; CONFIG = B00111111; //37 - temp_c38
        spi_out2(CONFIG, CONFIG_ADR_M); internalRead();
        CONFIG_ADR_M = B11100110; CONFIG = B00000001; //38 - temp_c38
        spi_out2(CONFIG, CONFIG_ADR_M); internalRead();
 
        delayMicroseconds(1);

        int vref_measured, kd_measured, kp_measured, ki_measured, pmos_out_measured; String o1, o2, o3, o4, o5, o6;

//        read_PID();

        if (en_tuning == 1) {
          // Enable Tuning
          digitalWrite(t_trig, HIGH);
          CONFIG_ADR_M = B00001100; CONFIG = B00000111; //12 - cfg_en_fine_cntrl (0),cfg_tol_reg_err (0), cfg_en_adapt_load (0), cfg_en_autotuning (0),cfg_sense_ctrl (10), cfg_adc_res_sel (00)
          spi_out2(CONFIG, CONFIG_ADR_M); internalRead();
          digitalWrite(t_trig, LOW);
          delay(tuneInterval); // After Tuning
          read_PID();
          delay(tuneInterval);
          read_PID();
          delay(tuneInterval);
          read_PID();
        }

        for (int l = 1; l <= 1; l++) {
          if (en_load_jump == 1) {
            load = String(load_2, BIN); load_len = load.length(); load = genPad(load, 4) + load;
            digitalWrite(t_trig, HIGH);
            load_addr_m = load_other.substring(2, 4) + load_addr; load_cfg =  genPad(load + load_other.substring(0, 2), 8) + load + load_other.substring(0, 2);
            load_addr_m_b = byte(strToInt(load_addr_m)); load_cfg_b = byte(strToInt(load_cfg));
            spi_out2(load_cfg_b, load_addr_m_b); internalRead();
            digitalWrite(t_trig, LOW);
            delayMicroseconds(1);
            read_PID();
//         Arvind's siganls - config31, 32, 33 and 34 - default speed with adcm= ~75MHz at vref_int=8, vin=1V
        CONFIG_ADR_M = B00011111; CONFIG = B00011000; //config31 - none(1'b0), sel_design_clk (1'b0), v_noise_lfsr_clk(2'b11), adaptCLK_lfsr_clk(2'b11), v_noise_lfsr_en(1'b1), adaptCLK_lfsr_en(1'b1)
        spi_out2(CONFIG, CONFIG_ADR_M); internalRead();
        CONFIG_ADR_M = B00100000; CONFIG = B00011100; //config32 - PA_M(4'b0111), sel_v_noise(4'b1111)
        spi_out2(CONFIG, CONFIG_ADR_M); internalRead();
        CONFIG_ADR_M = B10100001; CONFIG = B00000011; //config33 - none(2'b00), PA_L(5'b00111), PA_M[4](1'b0)
        spi_out2(CONFIG, CONFIG_ADR_M); internalRead();
        CONFIG_ADR_M = B00100010; CONFIG = B00000000; //config34 - DM_L(4'b0011), DM_M(4'b0011)
        spi_out2(CONFIG, CONFIG_ADR_M); internalRead();
      }
    }
  }
}
}
}

//char Keypad::waitForKey() {
//  char waitKey = NO_KEY;
//  while( (waitKey = getKey()) == NO_KEY );  // Block everything while waiting for a keypress.
//  return waitKey;
//}

void spi_out2(byte config_adr, byte config_data) {
  digitalWrite(ssbM, LOW);
  spi_transfer2(config_adr); spi_transfer2(config_data);
  digitalWrite(ssbM, HIGH);
}
void reset_IVR (int rst) {
//  delay(0.050);
  digitalWrite(rst, HIGH);
//  delay(0.050);
  digitalWrite(rst, LOW);
//  delay(0.5);
}

void init_DPWM_IVR (int dpwm) {
//  delay(0.050);
  digitalWrite(dpwm, LOW);
//  delay(0.050);
  digitalWrite(dpwm, HIGH);
//  delay(0.5);
}

void internalRead() {
  digitalWrite(rdEN, HIGH);
  //delay(5);
  digitalWrite(rdEN, LOW);
}

void internalReadsp() {
  digitalWrite(rdEN, HIGH);
  digitalWrite(rdEN, LOW);
}

int strToInt(String s)
{
  int value = 0;
  for (int i = 0; i < s.length(); i++) // for every character in the string  strlen(s) returns the length of a char array
  {
    value *= 2; // double the result so far
    if (s[i] == '1') value++;  //add 1 if needed
  }
  return value;
}

byte digitalChipReadMSPI(int config_adr) {
  digitalWrite(rdEN, LOW);
  digitalWrite(ssbM, LOW);
  spi_transfer2(config_adr);
  digitalWrite(ssbM, HIGH);
  spi_transfer2(config_adr);//read
  digitalWrite(ssbM, LOW);
  byte output = spi_transferread(config_adr);//read
  digitalWrite(ssbM, HIGH);
  return output;
}

String genPad(String s, int len) {
  int s_len = s.length();
  int pad_len = len - s_len;
  String pad;
  if (pad_len == 0) {
    pad = "";
  } else if (pad_len == 1) {
    pad = "0";
  } else if (pad_len == 2) {
    pad = "00";
  } else if (pad_len == 3) {
    pad = "000";
  } else if (pad_len == 4) {
    pad = "0000";
  } else if (pad_len == 5) {
    pad = "00000";
  } else if (pad_len == 6) {
    pad = "000000";
  }
  return pad;
}

void read_PID() {
  int vref_measured, kd_measured, kp_measured, ki_measured, pmos_out_measured; String o1, o2, o3, o4, o5, o6;
  o1 = String(digitalChipReadMSPI(27), BIN); // 27 - config27[7:4] = kd_out (00010), config27[3:0] = adc_out // autotuning kd=0110=7, kp=01011=11, ki=11100=28
  o2 = String(digitalChipReadMSPI(28), BIN); // 28 - config28[7:5] = ki_out[2:0] (11010), config28[4:0] = kp_out (11111)
  o3 = String(digitalChipReadMSPI(29), BIN); // 29 - config29[7:2] = 6'b0, config29[1:0] = ki_out[4:3] (11010)
  o4 = String(digitalChipReadMSPI(30), BIN); // 30 - config30[7:2] = 3'b0, config30[1:0] = pmos_out[4:0]
  o1 = genPad(o1, 8) + o1; o2 = genPad(o2, 8) + o2; o3 = genPad(o3, 8) + o3; o4 = genPad(o4, 8) + o4;
  vref_measured = strToInt(o1.substring(4, 8)); kd_measured = strToInt(o1.substring(0, 4)); kp_measured = strToInt(o2.substring(3, 8)); ki_measured = strToInt(o3.substring(6, 8) + o2.substring(0, 3)) ; pmos_out_measured = strToInt(o4.substring(3, 8));
  Serial.println("vref: " + String(vref_measured) + ", kp: " + String(kp_measured) + ", ki: " + String(ki_measured) + ", kd: " + String(kd_measured) + ", pmos_out: " + String(pmos_out_measured));
}

void resetA() {
  // reset
//  delay(0.20);
  digitalWrite(gRSTA, LOW);
//  delay(0.20);
  digitalWrite(gRSTA, HIGH);
//  delay(1);
}

void PrintHex8(uint8_t data, uint8_t length) // prints 8-bit data in hex with leading zeroes
{
       //Serial.print("0x"); 
       //for (int i=0; i<length; i++) { 
         if (data<0x10) {Serial.print("0");} 
         Serial.print(data,HEX); 
         //Serial.print(" "); 
       //}
}

void PrintHex16(uint16_t *data, uint8_t length) // prints 16-bit data in hex with leading zeroes
{
       Serial.print("0x"); 
       for (int i=0; i<length; i++)
       { 
         uint8_t MSB=byte(data[i]>>8);
         uint8_t LSB=byte(data[i]);
         
         if (MSB<0x10) {Serial.print("0");} Serial.print(MSB,HEX); Serial.print(" "); 
         if (LSB<0x10) {Serial.print("0");} Serial.print(LSB,HEX); Serial.print(" "); 
       }
}
