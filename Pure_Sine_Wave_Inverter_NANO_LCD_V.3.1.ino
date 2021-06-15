/*
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~350 W Pure Sine Wave Inverter with CC CV Charging, Automatic Changeover [Rev. 2]~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
****[The author will NOT take any liability if you receive any kind of damage using this code. Use at your own risk!]****
  Pure_Sine_Wave_Inverter_NANO_V.3.1 [NOT compatible with version 1.x or 2.x boards]
  Change log:
  16x2 LCD added instead of LED pannel
  pin# changed: HS2 "D11 --> D10", LS1 "D10 --> D11"
  MOSFET driver shutdown (SD) pin disconnected and grounded (i.e. drivers are now always ON)
  UPS_ON/OFF switch added at D13
  Fuse Blown signal added at D8 [Not implemented so far. Kept for future upgradability]
  Sayantan Sinha: 06/06/2021
  sPWM on the atMega328P for the arduino NANO. H-bridge output with deadtime.
*/
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define FB 0                                    // Fuse Blown                            (D8  i.e. bit-0 in PORTB) (LOW: OK, HIGH: iether DC fuse is blown or any-or-both of the low-side MOSFETs are not working) 
#define HS1 1                                   // High side drive pin of Half Bridge #1 (D9  i.e. bit-1 in PORTB)
#define HS2 2                                   // Low side drive pin of Half Bridge #1  (D10 i.e. bit-2 in PORTB)
#define LS1 3                                   // High side drive pin of Half Bridge #2 (D11 i.e. bit-3 in PORTB)
#define LS2 4                                   // Low side drive pin of Half Bridge #2  (D12 i.e. bit-4 in PORTB)
#define SW 5                                    // UPS ON/OFF switch                     (D13 i.e. bit-5 in PORTB) (HIGH: UPS OFF, LOW: UPS ON)
#define PIN_ZC (PIND & (1 << PD3))              // Read pinZc
#define PIN_MNS (PIND & (1 << PD2))             // Read pinMns
#define PIN_SW (PINB & (1 << PB5))              // Read pinSw
#define W_CNG_RLY_HIGH (PORTD |= 0b10000000)    // Make change over relay HIGH
#define W_CNG_RLY_LOW (PORTD &= 0b01111111)     // Make change over relay LOW
// A4 & A5 are used in I2C communication
#define SW_FREQ 16000                           // sPWM switching frequency in Hz
#define MAX_COUNT 1000                          // (16 MHz / SW_FREQ) / 2
#define MAX_COUNT_CHG 400                       // Max count for charging which corresponds to a max duty cycle = (MAX_COUNT_CHG / MAX_COUNT) * 100% during charging
#define NP 160                                  // Number of Pulses in one half cycle = (16 kHz / 50 Hz) / 2 = 160
#define Q_CYCLE 80                              // Pulse counter at Querter of cycle
#define DT 100                                  // Deadtime between low-side & high-side switching
#define THC 200                                 // Threshold count = the min count needs to execute the compare match int reliably
#define MOD_INDX_MAX 95                         // Max modulation index of sPWM
#define MOD_INDX_MIN 40                         // Min modulation index of sPWM
#define VO_LO 537                               // ADC reading equivalent for the lower threshold output voltage  = ((vtx / c) / ADC res); e.g. vtx = 10.95 V, voltage devider fact. c = 2.660, ADC res = 4.887 mV
#define VO_HI 562                               // ADC reading equivalent for the upper threshold output voltage  = ((vtx / c) / ADC res); e.g. vtx = 12.40 V, voltage devider fact. c = 2.660, ADC res = 4.887 mV
#define VB_MAX 950                              // ADC reading equivalent for the max bat voltage during cc-cv charge = ((vb / k) / ADC res); e.g. vb = 14.20 V, voltage devider fact. k = 3.122, ADC res = 4.887 mV
#define VB_TOP 920                              // ADC reading equivalent for the max bat voltage during topping charge = ((vb / k) / ADC res); e.g. vb = 13.80 V, voltage devider fact. k = 3.122, ADC res = 4.887 mV
#define VC_MAX 85                               // ADC reading equivalent for the max chg current during cc-cv charge = ((ic * a) / ADC res); e.g. ic = 6.0 A, voltage amplification fact. a = 0.07, ADC res = 4.887 mV
#define VC_TOP 25                               // ADC reading equivalent for the max chg current during topping charge = ((ic * a) / ADC res); e.g. ic = 0.5 A, voltage amplification fact. a = 0.22, ADC res = 4.887 mV
#define VB_LOW 750                              // ADC reading equivalent for the low bat voltage during running the inverter = ((vb / k) / ADC res); e.g. vb = 11.50 V, voltage devider fact. k = 3.122, ADC res = 4.887 mV
#define VB_CRIT 690                             // ADC reading equivalent for the critical bat voltage during running the inverter = ((vb / k) / ADC res); e.g. vb = 10.50 V, voltage devider fact. k = 3.122, ADC res = 4.887 mV
#define VL_MAX 560                              // ADC reading equivalent for the max load current during inverter running = ((il * b) / ADC res); e.g. il = 50 A, voltage amplification fact. b = 0.0545, ADC res = 4.887 mV
#define MNS_WAIT_TIME 3000                      // Wait time (ms) from detecting mains voltage to setting the mnsOn flag
#define CNG_OVER_DLY 8                          // Delay time (ms) after zero-crossing to start the change over procedure

#define CNG_OVER true

#define MNS_ON 0b10000000                       // Mask to read MNS_ON bit from the LED Status Reg
#define INV_ON 0b01000000
#define CHG_ON 0b00100000
#define BAT_LO 0b00010000
#define OVR_LD 0b00001000
#define FUS_BL 0b00000100

const int pinVo = A0, pinVt = A1, pinVb = A2, pinMd = A3, pinVl = A6, pinVc = A7;                                              // Analog pin declearations
const int pinFan = 1, pinMns = 2, pinZc = 3, pinBuz = 4, pinChRng = 5, pinChRly = 6, pinCngRly = 7, pinFb = 8, pinSw = 13;     // Digital pin declerations

byte ledsr = 0;                                 // LED Status Register [MNS_ON  INV_ON  CHG_ON  BAT_LO  OVR_LD  FUS_BL  0  0]
byte erCode = 0;                                // Error code for inverter shutdown
volatile byte i = 0;                            // The counter for the lookUp array
volatile bool pcyl = false;                     // Flag for positive half cycle

bool invOn = false;                             // Inverter on flag
bool mnsOn = false;                             // Mains on flag
bool chgOn = false;                             // Charging on flag
bool cngOverRequest = false;                    // Change over to mains request flag
bool zcPrev = true;                             // Stores the status of pinZc
bool topChg = false;
byte dispStatus = 0;                            // LCD display status register [MNS_ON  INV_ON  CHG_ON  BAT_LO  OVR_LD  FUS_BL  0  0]
unsigned int modIndx = 50;                      // Start the inverter with this modulation index
unsigned int vo;                                // Stores output voltage feedback
unsigned int vb;                                // Stores bat voltage
unsigned int vl;                                // Stores load current
unsigned int vc;                                // Stores charging current
unsigned long timePrev = 0;                     // General purpose time recorder
unsigned long timeInvOff = 0;                   // Inverter off time recorder
unsigned long timeMnsOn = 0;                    // Mains on time recorder
unsigned long zcTime = 0;                       // Records zero crossing time of mains voltage
unsigned long mnsTime = 0;                      // Records the mains voltage detection time
unsigned long tRefresh = 0;                     // Records the time of display refresh
//Look up tables with 160 entries each, normalised to have max value of 500 which is the period of the PWM loaded into register ICR1.(D:\Project Files\MATLAB\SPWM_PhaseFreqCorrect_LookUp_Table_Gen_for_V_3_0.m)
unsigned int lookUpInverseHalf[NP] = {0, 10, 20, 29, 39, 49, 59, 69, 78, 88, 98, 107, 117, 126, 136, 145, 155, 164, 173, 182, 191, 200, 209, 218, 227, 236, 244, 253, 261, 270, 278, 286, 294, 302, 310, 317, 325, 332, 339, 347, 354, 360, 367, 374, 380, 387, 393, 399, 405, 410, 416, 421, 426, 431, 436, 441, 446, 450, 454, 458, 462, 466, 469, 472, 476, 478, 481, 484, 486, 488, 490, 492, 494, 495, 497, 498, 498, 499, 500, 500, 500, 500, 500, 499, 498, 498, 497, 495, 494, 492, 490, 488, 486, 484, 481, 478, 476, 472, 469, 466, 462, 458, 454, 450, 446, 441, 436, 431, 426, 421, 416, 410, 405, 399, 393, 387, 380, 374, 367, 360, 354, 347, 339, 332, 325, 317, 310, 302, 294, 286, 278, 270, 261, 253, 244, 236, 227, 218, 209, 200, 191, 182, 173, 164, 155, 145, 136, 126, 117, 107, 98, 88, 78, 69, 59, 49, 39, 29, 20, 10};
unsigned int lookUp[NP] = {1000, 980, 961, 941, 922, 902, 882, 863, 844, 824, 805, 786, 767, 748, 729, 710, 691, 672, 654, 636, 617, 599, 581, 564, 546, 529, 511, 494, 478, 461, 444, 428, 412, 396, 381, 366, 351, 336, 321, 307, 293, 279, 266, 252, 240, 227, 215, 203, 191, 180, 169, 158, 147, 137, 128, 118, 109, 100, 92, 84, 76, 69, 62, 55, 49, 43, 38, 32, 28, 23, 19, 16, 12, 9, 7, 5, 3, 2, 1, 0, 0, 0, 1, 2, 3, 5, 7, 9, 12, 16, 19, 23, 28, 32, 38, 43, 49, 55, 62, 69, 76, 84, 92, 100, 109, 118, 128, 137, 147, 158, 169, 180, 191, 203, 215, 227, 240, 252, 266, 279, 293, 307, 321, 336, 351, 366, 381, 396, 412, 428, 444, 461, 478, 494, 511, 529, 546, 564, 581, 599, 617, 636, 654, 672, 691, 710, 729, 748, 767, 786, 805, 824, 844, 863, 882, 902, 922, 941, 961, 980};

void changeDuty(void);
void fireInv(void);
void shutdownInv(bool coEnable = false);
void startChg(void);
void stopChg(void);
void refreshDisp(const byte oneByte);
void beepErr(void);

LiquidCrystal_I2C lcd(0x3f, 16, 2);

void setup()
{
  DDRB = 0b00011110;                                       // Set pin D12 to D9 as output and D13 & D8 as input. D13 = PB5 (PB7 & PB6 are unusable). PORTB = [XTAL2 XTAL1 D13 D12 D11 D10 D9 D8]]
  pinMode(pinFan, OUTPUT);
  pinMode(pinMns, INPUT_PULLUP);
  pinMode(pinZc, INPUT_PULLUP);
  pinMode(pinBuz, OUTPUT);
  pinMode(pinChRng, INPUT_PULLUP);
  pinMode(pinChRly, OUTPUT);
  pinMode(pinCngRly, OUTPUT);
  pinMode(SCL, INPUT_PULLUP);
  pinMode(SDA, INPUT_PULLUP);
  lcd.begin();
  changeDuty();
  refreshDisp(MNS_ON | INV_ON | CHG_ON | BAT_LO | OVR_LD | FUS_BL);  // Show welcome msg ("INVERTER ON / CHARGING")
  digitalWrite(pinBuz, HIGH);                              // Start up beep!
  delay(500);
  digitalWrite(pinBuz, LOW);
  delay(2000);
  refreshDisp(0);                                          // Clear LCD
  mnsOn = !PIN_MNS;                                        // Mains On flag will be cleared if pinMns is high
  if (PIN_SW)
    if (!mnsOn)                                            // No mains power detected
      fireInv();
}

void loop()
{
  if (invOn) {                                             // Inverter Mode routine
    if (i == 1) {                                          // Zero-crossing of the inverter
      if (cngOverRequest)                                  // If there is a change over to mains request pending
        shutdownInv();                                     // Shutdown inverter but without changing over to the mains
      else {                                               // If there is no shutdown request pending
        vo = analogRead(pinVo);                            // Read output voltage feedback at (+Ve & -Ve peaks)
        vb = analogRead(pinVb);                            // Read bat volatage
        vl = analogRead(pinVl);                            // Read load current
        if (vo < VO_LO) {                                  // If output voltage is lower than the VO_LO
          if (modIndx < MOD_INDX_MAX) {
            modIndx++;                                     // Increase modulation index by 1%
            changeDuty();
          }
        }
        else if (vo > VO_HI) {                             // If output voltage is higher than the VO_HI
          if (modIndx > MOD_INDX_MIN) {
            modIndx--;                                     // Decrease modulation index by 1%
            changeDuty();
          }
        }
        erCode = vb < VB_CRIT ? BAT_LO : vl > VL_MAX ? OVR_LD : erCode;
        if (erCode) {                                      // If either bat voltage is below critical or load current is higher that max rating
          shutdownInv(CNG_OVER);                           // Shut down inverter with change over to mains enable
          beepErr();                                       // Gives error notification by beeping and makes an 1 s delay
        }
        if (!PIN_SW)                                       // If inverter on/off switch is turned off
          shutdownInv(CNG_OVER);                           // Shut down inverter with change over to mains enable
      }
    }
    if (mnsOn) {                                           // If mains voltage is on but inverter is still running
      if (cngOverRequest) {                                // If there is a change over to mains request pending
        if (millis() - zcTime > CNG_OVER_DLY)              // The change over to mains process starts after CNG_OVER_DLY
          shutdownInv(CNG_OVER);                           // Shut down inverter with change over to mains enable
      }
      else if (PIN_ZC && !zcPrev) {                        // When the mains voltage is going from -Ve half to +Ve half. (pinZc = HIGH: Mains voltage is at +Ve hallf cycle)
        cngOverRequest = pcyl;                             // A change over request to mains voltage is issued if inverter and mains both are at +Ve half
        zcTime = millis();                                 // Record the time of zero crossing
      }
      zcPrev = PIN_ZC;                                     // Save the state of zero-crossing pin
      mnsOn = !PIN_MNS;                                    // Mains On flag will be cleared if pinMns goes high
    }
    else {
      if (!PIN_MNS) {                                      // Mains voltage detected
        if (!mnsTime)
          mnsTime = millis();                              // Record the time when the mains voltage is detected
        else if (millis() - mnsTime > MNS_WAIT_TIME) {
          mnsOn = !PIN_MNS;                                // Mains On flag will be set if pinMns is still low after MNS_WAIT_TIME is elapsed
          mnsTime = 0;                                     // Clear the mnsTime
          zcPrev = true;
          cngOverRequest = false;                          // Clear any pending change over to mains request
        }
      }
      else                                                 // Mains power failed before the mns wait time
        mnsTime = 0;                                       // Clear the mnsTime
    }
  }
  else {                                                   // Mains Mode routine (will be executed even if mains is off, if the inverter on/off switch is at "off" position)
    if (mnsOn) {                                           // If mains voltage is on
      if (!chgOn) {
        if (millis() - timeMnsOn > 2500)                   // If 2.5 s has been elapsed after mains voltage on
          startChg();                                      // Start charging
      }
      else {                                               // Charging is on
        if (millis() - timePrev > 9) {                     // Actuate charging current in every 10 ms interval
          unsigned int vbatMax = topChg ? VB_TOP : VB_MAX; // If topping charge is on max charging voltage is VB_TOP equiv. else VB_MAX equiv.
          vb = analogRead(pinVb);
          vc = analogRead(pinVc);
          if (vc < VC_MAX && vb < vbatMax) {
            if (OCR1B < MAX_COUNT_CHG)                     // Maximum duty cycle = (MAX_COUNT_CHG / MAX_COUNT) * 100 %;
              OCR1B = OCR1B + 1;                           // Increase duty-cycle slowly
          }
          else if (vb > vbatMax || vc > VC_MAX) {
            if (OCR1B > 0)                                 // Minimum possible duty cycle 0%
              OCR1B = OCR1B - 1;                           // Decrease duty-cycle slowly
          }
          topChg = topChg ? vc <= VC_TOP : (vb >= VB_MAX && vc < VC_TOP); // If battery voltage reaches VB_MAX equiv. and charging current drops below VC_TOP equiv. then topping charge is started. If topping chg is already set then it will be reset if chg current goes above VC_TOP equiv.
          if ((ledsr & CHG_ON) && topChg)                  // If cc-cv charging ended and topping charge has been started but charge on led is still on
            refreshDisp(ledsr & ~CHG_ON);                  // Clear charging status on LCD
          else if (!(ledsr & CHG_ON) && !topChg)           // If cc-cv charging is on but, chg on LED is off
            refreshDisp(CHG_ON);                           // Show charging status on LCD
          timePrev = millis();                             // Record the charging current actuation time
        }
      }
      if (!(ledsr & MNS_ON))                               // If mains is on but, mains on LED is off
        refreshDisp(MNS_ON);
      pcyl = !PIN_ZC;                                      // If mains voltage is at +Ve half, the inverter will start at -Ve half in case of mains failure
    }
    else {                                                 // If mains voltage fails
      if (PIN_SW) {                                        // Check if inverter on/off switch is at "on" position
        fireInv();                                         // Start inverter and switch to inverter mode
      }
      if (!invOn) {                                        // If last fireInv call did not turn on the inverter
        timeMnsOn = millis();                              // Mains on timing is reset to present time
        if (chgOn)                                         // Mains off, Inverter off but charging is still on
          stopChg();                                       // Stop chargig
        if (ledsr & MNS_ON)                                // If mains failed but mains on LED is still on
          refreshDisp(ledsr & ~MNS_ON);                    // Turn off mains on LED
      }
    }
    mnsOn = !PIN_MNS;                                      // Mains On flag will be cleared if pinMns goes high
  }
}

ISR(TIMER1_OVF_vect)
{
  if (i >= NP) {
    i = 0;
    pcyl = !pcyl;
  }
  if (i == 1) {
    TCCR1A = pcyl ? 0b11000010 : 0b00110010;       // +Ve half cycle: Set OC1A (pin D9) on Compare Match; Disconnect OC1B (pin D10); -Ve half cycle: Opposite
    TIFR1 |= 0b00000110;                           // Clears the Output compare match A&B flags. [Ref. Datasheet p. 136]
    TIMSK1 = pcyl ? 0b00000101 : 0b00000011;       // +ve half cycle: INT on comp match B, -ve half cycle: INT on comp match A, both halves: INT on timer overflow
  }
  if (pcyl) {
    OCR1A = lookUp[i];
    OCR1B = lookUp[i] - DT;
  }
  else {
    OCR1B = lookUp[i];
    OCR1A = lookUp[i] - DT;
  }
  if (lookUp[i] > THC)
    PORTB |= 0b00011000;
  else if (pcyl)
    PORTB &= 0b11110111;
  else
    PORTB &= 0b11101111;
  i++;
}

ISR(TIMER1_COMPA_vect)
{
  if (invOn)
    PORTB &= 0b11101111;  // Turn off LS2
  else if (OCR1B > 0)
    PORTB |= 0b00011000;  // Turn on LS1 & LS2 in charging mode
}

ISR(TIMER1_COMPB_vect)
{
  if (invOn)
    PORTB &= 0b11110111;  // Turn off LS1
  else
    PORTB &= 0b11100111;  // Turn off LS1 & LS2 in charging mode
}

void changeDuty(void)
{
  modIndx = modIndx > MOD_INDX_MAX ? MOD_INDX_MAX : modIndx < MOD_INDX_MIN ? MOD_INDX_MIN : modIndx;           // Above 92% the minimum count will be below dead-time (DT)
  lookUp[0] = MAX_COUNT;
  for (int i = 1; i <= Q_CYCLE; i++) {
    lookUp[i] = MAX_COUNT - ((lookUpInverseHalf[i] * modIndx) / 50);                                          // Calculate new duty cycle for quarter cycle
  }
  int j = Q_CYCLE - 1;
  for (int i = Q_CYCLE + 1; i < NP; i++) {                                                                    // Copy the duty cycle for next quarter cycle
    lookUp[i] = lookUp[j];
    j--;
  }
}

void fireInv(void)
{
  TIMSK1 = 0;                                    // Disable Timer1 interrupts
  PORTB &= 0b11100001;                           // Pull down LS2, LS1, HS2, & HS1.
  vb = analogRead(pinVb);
  if ( vb > VB_LOW) {
    W_CNG_RLY_HIGH;                              // Engage change over relay
    refreshDisp(INV_ON);                         // Show the inverter on status on LCD
    TCCR1A = pcyl ? 0b11000010 : 0b00110010;     // To start with: +Ve half cycle-> OC1A (pin D9) = Set on Compare Match; OC1B (pin D10) = Disconnect; -Ve half cycle-> Opposite
    TCCR1B = 0b00011001;                         // Fast PWM (Mode 14); Clock Select = System clock (No Prescaling) [Ref. Data Sheet, p. 132]
    ICR1 = MAX_COUNT;                            // Switching frequency = 16 kHz (1 / (TIMER1_TOP * 62.5e-9))
    i = 0;                                       // Clear counter for lookup table
    if (pcyl) {
      OCR1A = lookUp[i];
      OCR1B = lookUp[i] - DT;
    }
    else {
      OCR1B = lookUp[i];
      OCR1A = lookUp[i] - DT;
    }
    PORTB &= 0b11100001;                         // Pull down LS2, LS1, HS2, & HS1.
    TIFR1 |= 0b00000111;                         // Clears the Output compare match A&B flags. Clear timer1 overflow flag [Ref. Datasheet pp. 136-137]
    TIMSK1 = pcyl ? 0b00000101 : 0b00000011;     // +ve half cycle: INT on comp match B, -ve half cycle: INT on comp match A, both halves: INT on timer overflow
    sei();                                       // Enable global interrupt
    invOn = true;                                // Inverter on flag set
    mnsOn = false;                               // Clear the mains on status flag
    chgOn = false;                               // Clear the charge on status flag
    erCode = 0;                                  // Inverter shutdown error code. 0 implies no error, inv shut down due to mains on
  }
  else {
    if (!(ledsr & BAT_LO)) {                     // If battery low indicator is not glowing
      refreshDisp(BAT_LO);                           // Indicate battery low
      digitalWrite(pinBuz, HIGH);                // Put a long beep
    }
    delay(800);
    digitalWrite(pinBuz, LOW);
    delay(200);
  }
}

void shutdownInv(bool coEnable = false)        // If coEnable is true, change over to mains will occur
{
  TIMSK1 = 0;                                  // No more interrupt!
  TCCR1A = 0b00000010;                         // OC1A (D9) & OC1B (D10) = Disconnect;
  PORTB &= 0b11100001;                         // Pull down LS2, LS1, HS2, & HS1.
  timeInvOff = millis();                       // Record the time when the inverter is turned off
  if (coEnable) {                              // If change over is enabled
    delay(1);
    W_CNG_RLY_LOW;                             // Disengage change over relay to switch to the mains power
    invOn = false;                             // Clear inverter on flag only after a successfull changeover to mains
    refreshDisp(!erCode ? MNS_ON : erCode);    // Indicate mains on if error code is zero else indicate the specific error
    mnsOn = !PIN_MNS;                          // Mains On flag will be cleared if pinMns goes high
    timeMnsOn = millis();                      // Cng over to mains time is recorded
    cngOverRequest = false;                    // Clear change over request pending status flag
    modIndx = 70;
    changeDuty();
  }
  chgOn = false;                               // Clear the charge on status flag
}

void startChg(void)
{
  refreshDisp(ledsr | CHG_ON);                 // Show the charging on status on display
  TCCR1A = 0b00000010;                         // OC1A (pin D9) & OC1B (pin D10) = Disconnect
  TCCR1B = 0b00011001;                         // Fast PWM (Mode 14); Clock Select = System clock (No Prescaling) [Ref. Data Sheet, p. 132]
  ICR1 = MAX_COUNT;                            // Switching frequency = 16 kHz (1 / (TIMER1_TOP * 62.5e-9))
  OCR1A = 0;                                   // Turn on LS1 & LS2 @ count 0 of timer1
  OCR1B = 0;                                   // This dictates the duty cycle. 0 = 0%, MAX_COUNT = 100%
  PORTB &= 0b11100001;                         // Pull down LS2, LS1, HS2, & HS1.
  TIFR1 |= 0b00000111;                         // Clears the Output compare match A&B flags. Clear timer1 overflow flag [Ref. Datasheet pp. 136-137]
  TIMSK1 = 0b00000110;                         // INT on output comp match A & B
  chgOn = true;                                // Inverter on flag set
  topChg = false;                              // Clear topping charge flag
}

void stopChg(void)
{
  OCR1A = 0;                                   // Turn on LS1 & LS2 @ count 0 of timer1
  OCR1B = 0;                                   // This dictates the duty cycle. 0 = 0%, MAX_COUNT = 100%
  TIMSK1 = 0b00000000;                         // Disable INT on output comp match A & B
  PORTB &= 0b11100001;                         // Pull down LS2, LS1, HS2, & HS1.
  chgOn = false;                               // Clear the charge on status flag
  refreshDisp(ledsr & ~CHG_ON);                // Turn off the charging on indicator
}

void refreshDisp(const byte oneByte)
{
  ledsr = oneByte;                             // update LED status register
  lcd.clear();
  if (oneByte & INV_ON)
    lcd.print("INVERTER ON");
  else if (oneByte & MNS_ON)
    lcd.print("MAINS ON");
  if (oneByte & 0x3F) {                        // If any flag other than INV_ON or MNS_ON is(are) set
    lcd.setCursor(0, 1);
    if (oneByte & CHG_ON)
      lcd.print("CHARGING...");
    else if (oneByte & BAT_LO)
      lcd.print("BATTERY LOW!");
    else if (oneByte & OVR_LD)
      lcd.print("OVER LOAD!");
    else if (oneByte & FUS_BL)
      lcd.print("FUSE BLOWN!");
  }
}

void beepErr(void)
{
  digitalWrite(pinBuz, HIGH);
  delay(800);
  digitalWrite(pinBuz, LOW);
  delay(3000);
}
