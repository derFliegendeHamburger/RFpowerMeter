/*
 * arduino IDE 1.8.13: 
 * 
 *  Board='Arduino NANO', 
 *  Processor='328P-Old-Bootlaoder'
 *
 */

//#define __AVR_ATmega328P__

/* RF Power Meter for AD8317 or similar
 *  default 5800MHz band, no ATTN
 *  
 *  connected:
 *  D13 buzzer (parallel to inbuilt LED)
 *  encoder with pushbutton for click and doubleclick (pins see below)
 *  A0 input for AD8317
 *  display via i2c
 *  
 *  
 *  v3.0b make bucket increment tunable
 *  v3.0c in loop(), do ticker.updates & encoder handling only every nth run
 *  v3.1  begin refactor outputs to lcd and BT
 *  v3.1b more refactoring
 *  v3.2  BT on/off
 *        
 */

#define ENCODER_A_PIN 4 // ec11 encoder A-pin 
#define ENCODER_B_PIN 3 // ec11 encoder B-pin 
#define ENCODER_BTN_PIN 2 // ec11 encoder E-button-pin 
#define BT_RX_PIN 9 
#define BT_TX_PIN 8 

#include <LibPrintf.h>

#if (ARDUINO < 10000)
   #include <Wire/src/Wire.h>
#else
   #include <Wire.h>
#endif
#include "LiquidCrystal_I2C.h"
#include "LcdBarGraphX.h"
#include "ClickEncoder.h"
#include "Ticker.h"
#include <SoftwareSerial.h>

SoftwareSerial btSerial(BT_RX_PIN, BT_TX_PIN); // RX, TX

#define VERSION "3.2_"
#define DEBUG 0

#define LCD_COLS 20
#define LCD_ROWS 4

#define SENSOR_PIN A0
#define VREF 2.50 //5.0 2.5V from external reference


// ------------------ declarations ------------------------
void update_display();
void do_update_display();
void call_encoder_service();
void single_read();
void splashScreen();
void setup();
void loop();


#define MV_MIN 350 // for AD8318 below this value is not sane
#define MV_MAX 2300 // for AD8318 above this value is not sane

#define ANZ_BANDS 2
int band=0; // 0==5800MHz 1==2400MHz
const char band_names[ANZ_BANDS][5]  = { "5G8", "2G2" };
float slope[] = { -25.2 , -24.4};   // mV/db averaged from 50dB-attn; or use  from datasheet:  { -24.3 , -24.4}; 
float intercept[] = { 25.0 , 19.6 };  // dbm at 0V
int dbm_low[] = { -64, -69 }; // below value for 50ohm load
int dbm_high[] = { -1, -2 }; // +-1dB error

#define ANZ_EDITS 5
int edit = 0; // 0==keinEdit 1==band.edit  2==attn.edit 3==binc.edit 4==bt.edit

#define MAX_ATTN 99
int attn = 0;

#define MAX_BINC 33 // bucket increment
int binc = 3;       // bucket increment, tunable

int bt_output = false;

LiquidCrystal_I2C lcd(0x3F,2,1,0,4,5,6,7,3,POSITIVE); 

LcdBarGraphX lbg_bucket(&lcd,LCD_COLS,0,2); 


ClickEncoder *encoder;
int16_t encoder_last = -1, encoder_value = 0;
ClickEncoder::Button encoder_button = encoder->getButton();



Ticker ticker1(update_display,1001,0,MILLIS); // 501
Ticker ticker2(call_encoder_service,1,0,MILLIS);

int aMin, aMax, aBucket;
unsigned long aCount;


boolean LOCK = false; // set to true during execution of ticker execution functions. If true, single_read() does nothing

float dbmTrend;

char s[21], t[21], s_dbm[21], s_pep[21], s_trend[21], s_mv[21], s_ar[21];

//----------------------------------------------------------------------------------
#define analog2mVolt(a) ( (float)(a) * VREF / 1023.0 * 1000 )
#define mVolt2db(mv) ( (float)(mv)/ slope[band] + intercept[band]  )
#define db2mW(db) ( pow(10, (  (float)(db) / 10.0) ) )

//----------------------------------------------------------------------------------
void call_encoder_service() 
{ 
  LOCK = true; 
  encoder->service(); 
  LOCK = false;
}
//----------------------------------------------------------------------------------
//#define BUZ_ON   digitalWrite(LED_BUILTIN, HIGH);
//#define BUZ_OFF  digitalWrite(LED_BUILTIN, LOW);
#define BUZ_ON   {PORTB |= _BV(5);}
#define BUZ_OFF  {PORTB &= ~_BV(5);}
#define BUZ_TICK 1
#define BUZ_CLICK 5

void buzz(int n) {
  BUZ_ON;
  delay(n);
  BUZ_OFF
}
//----------------------------------------------------------------------------------
void reset_analog_values() {
  aMin = 1024;
  aMax = 0;
  aBucket = 512;
  dbmTrend = 2; // acts as countdown when > 0
  aCount = 0;
}

void end_edit(void) {
  edit= 0; 
  reset_analog_values();
  splashScreen(); 
  delay(2600);
  lbg_bucket.refresh();
}
//----------------------------------------------------------------------------------
/*
 * Function to read ADC within 20us instead of 116us 
 * This function is borrowed from Albert http://www.avdweb.nl/arduino/libraries/fast-10-bit-adc.html
 */
int analogReadFast(byte ADCpin, byte prescalerBits=4)
{ 
  byte ADCSRAoriginal = ADCSRA;
  ADCSRA = (ADCSRA & B11111000) | prescalerBits;
  int adc = analogRead(ADCpin); 
  ADCSRA = ADCSRAoriginal;
  return adc;
}
//----------------------------------------------------------------------------------

//----------------------------------------------------------------------------------
// perform one read
// update the min & bucket value; inc the counter
void single_read(void) {
    if (LOCK) return;
    int analog = analogReadFast(SENSOR_PIN); // useful read()
    if (analog < aMin) aMin = analog;
    else if (analog > aMax) aMax = analog;
    if (analog < aBucket) {
      aBucket -= binc;
    } else if (analog > aBucket) {
      aBucket += 1;
    }
    aBucket = constrain( aBucket, 0, 1023);
    aCount++;
    if (DEBUG) { Serial.print(analog); Serial.print(' '); }
  }


//----------------------------------------------------------------------------------

//----------------------------------------------------------------------------------

void update_display() {
  LOCK = true;
  do_update_display();
  aCount = 0;
  LOCK = false;
}
void sprint_mW( char s[], float mW) {
      if (mW > 1000.0) {
        sprintf( s, "%.2fW",  mW / 1000.0); 
      } else if (mW > 1) {
        sprintf( s, "%.1fmW",  mW); 
      } else if (mW > 0.001) {
        sprintf( s, "%.1fuW",  1000.0 * mW ); 
      } else {
        sprintf(s, "%.1fnW", 1000000.0 * mW); 
      }
      //lcd.print(s);
}

void btVT100clear() {
  //btSerial.print("---\r\n");
    btSerial.write(0x1B); btSerial.write(0x5B); btSerial.write("2J"); //ED2
    btSerial.write(0x1B); btSerial.write(0x5B); btSerial.write("1;1H");//cursor top left
}
void btVT100clearToEOL() {
    btSerial.write(0x1B); btSerial.write(0x5B); btSerial.write("0K"); //EL0
}
void btVT100setLine(byte line) { // line = [ 1 ; 99 ]
    btSerial.write(0x1b); btSerial.write(0x5b);
    btSerial.write( '0' );
    btSerial.write( '0' + line );
    btSerial.write(";1H"); //pos line 1
    btSerial.write(0x1b); btSerial.write(0x5b); btSerial.write("2K");//EL2

}
//----------------------------------------------------------------------------------
void lcdSetLine(byte line) { // line = [ 0 ; 9 ]
    lcd.setCursor(0,line); lcd.print(F("                    "));
    lcd.setCursor(0,line);
}
//----------------------------------------------------------------------------------

void do_update_display() {
    float mvMin = analog2mVolt(aMin);
    float mvMax = analog2mVolt(aMax);
    float mvBucket = analog2mVolt(aBucket);
    float dbmMax = mVolt2db(mvMin); // higher voltage gives lower (negative) db - example mv-min = 1600 => db-max = -30
    float dbmMin = mVolt2db(mvMax); // -example mv-max = 1900 => db-min = -55
    float dbmBucket = mVolt2db(mvBucket); // this is our PEP !

    int error = (mvMin < MV_MIN) || (mvMax > MV_MAX);

    sprintf(s_dbm, "dBm[%.1f;%.1f]", dbmMin, dbmBucket);
    sprint_mW(s, db2mW((dbmBucket+attn)) );
    sprintf(s_pep, "%.1fdBm=%s PEP", dbmBucket+attn, s);
    sprintf(s_mv, "mV[%.0f;%.0f]%.0f", mvMax, mvBucket, mvMin);
    sprintf(s_ar, "aR[%d;%d]%d#%d", aMax, aBucket, aMin, aCount);

    //btVT100clear();
    sprintf(t, "- - -");

// 0  - 0 - 0  - 0 - 0  - 0 - 0  - 0 - 0  - 0 - 0 - 0 - 0 - 0 - 0 - 0 - 0 - 0 - 0 - 0 - 0 - 0
    lcdSetLine(0);
    if (error) {
      lcd.print(F("*Sensor Voltage KO*"));
    } else if ( (! edit) && (dbmMin < dbm_low[band]) ) {
      lcd.print(F("*below 50ohm load*"));
    } else if ( (! edit) && (dbmMax > dbm_high[band]) ) {
      lcd.print(F("*outside 1dB error*"));
    } else if (edit) {
      if (edit==1) lcd.print('*'); lcd.print(band_names[band]); //lcd.print(F("MHz"));
      lcd.print((edit==2) ? '*': ' '); lcd.print(attn); lcd.print(F("dB"));
      lcd.print((edit==3) ? '*': ' '); lcd.print(binc); lcd.print(F("inc"));
      lcd.print((edit==4) ? '*': ' '); lcd.print(bt_output ? "BTon" : "BToff"); 
    } else {
      lcd.print(s_dbm);
    }
    if (bt_output) {
      btVT100setLine(1);
      btSerial.print(s_dbm); btSerial.print(dbmMax,1); 
    }

//  1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1
    lcdSetLine(1);
    if (DEBUG || edit || error) {
      lcd.print(s_dbm);  lcd.print(dbmMax,1);
    } else {
      lcd.print(s_pep);
    }
    if (bt_output) {
      btVT100setLine(4);
      btSerial.print(s_pep); 
    }
    
//  2 - 2 - 2 - 2 - 2 - 2 - 2 - 2 - 2 - 2 - 2 - 2 - 2 - 2 - 2 - 2 - 2 - 2 - 2 - 2 - 2 - 2 - 2
    lcdSetLine(2);
    if (DEBUG || edit || error) {
      lcd.print(s_mv); 
    } else {
      //
      int t = constrain( dbmBucket - dbm_low[band] , 0, dbm_high[band] - dbm_low[band] );
      lbg_bucket.drawValue( t , dbm_high[band] - dbm_low[band]); // all our relevant dbm values are negative     
      lbg_bucket.refresh(); 
    }
    if (bt_output) {
      btVT100setLine(3);
      btVT100clearToEOL();
    }
    

//  3 - 3 - 3 - 3 - 3 - 3 - 3 - 3 - 3 - 3 - 3 - 3 - 3 - 3 - 3 - 3 - 3 - 3 - 3 - 3 - 3 - 3 - 3
    lcdSetLine(3);
    if (DEBUG || edit || error) {
      lcd.print(s_ar); 
    } else {  //                     0123456789.123456789
      if (dbmTrend >= 0) {
        sprintf(t, "Peak Envelope Power" );
        lcd.print(t);
      }
      // trend
      #define TREND 5.0
      if (dbmTrend > 0) 
        dbmTrend--;
      else if (dbmTrend == 0) 
        dbmTrend = dbmBucket;
      else {
        dbmTrend = ( TREND * dbmTrend + dbmBucket) / (TREND + 1.0);
        sprintf(s_trend, "%.1fdBm PEP Trend", dbmTrend+attn);
        lcd.print(s_trend);
      }
    }
    if (bt_output) {
      btVT100setLine(2);
      btSerial.print(s_trend);
   
      btVT100setLine(5);
      btSerial.print(s_mv);
      btVT100setLine(6);
      btSerial.print(s_ar);
      btVT100setLine(7);
    }

    if (DEBUG) {
      Serial.print(F("\t%\t"));
      Serial.print(aCount);  Serial.println('\t');
    }
}
//----------------------------------------------------------------------------------
void splashScreen() {
  Serial.print(F("\n\n"));
  lcd.clear();
  lcd.setCursor(0,0);
  //           0123456789.123456789 // last 4 chars reserved for version
  lcd.print(F("RF PeakEnvelopePower"));
  lcd.setCursor(0,1);
    //         0123456789.12345.... // last 4 chars reserved for version
  lcd.print(F("with AD8318 v"));
  lcd.print(VERSION);
  lcd.setCursor(0,2);
  lcd.print(band_names[band]); lcd.print(' ');
  lcd.print(attn); lcd.print(F("dB "));
  lcd.print(binc); lcd.print(F("inc "));
  lcd.print(bt_output); lcd.print(F("BT"));
  lcd.setCursor(0,3);
  lcd.print(F("Initialising..."));

  Serial.print(F("Band:"));
  Serial.print(band_names[band]);
  Serial.print(F(" ATTN:"));
  Serial.print(attn);
  Serial.print(F(" Slope:"));
  Serial.print(slope[band],5);
  Serial.print(F(", Intercept:"));
  Serial.println(intercept[band],5);
  
  btVT100clear();

  return ;
}
//----------------------------------------------------------------------------------
void usageScreen() {
  lcd.clear();
  lcd.setCursor(0,0);
  //           0123456789.123456789
  lcd.print(F("single click: setup"));
  lcd.setCursor(0,1);
  lcd.print(F("double click:+exit"));
  lcd.setCursor(0,2);
  lcd.print(F("             +reset"));
  lcd.setCursor(0,3);
  lcd.print(F("-select band&attn"));

  Serial.println(F("-end of usage()"));
  return ;
}

//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
 
void setup(){
  analogReference(EXTERNAL);
  
  Serial.begin(38400);
  btSerial.begin(38400);
  lcd.begin(LCD_COLS,LCD_ROWS);
  splashScreen();

  btSerial.println(F("\r\nRunning setup..."));

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(A1, OUTPUT); digitalWrite(A1, LOW);
  pinMode(A2, OUTPUT); digitalWrite(A2, LOW);
  pinMode(A3, OUTPUT); digitalWrite(A3, LOW);
  //pinMode(A4, OUTPUT); digitalWrite(A4, LOW);
  //pinMode(A5, OUTPUT); digitalWrite(A5, LOW);
  pinMode(A6, OUTPUT); digitalWrite(A6, LOW);
  pinMode(A7, OUTPUT); digitalWrite(A7, LOW);

  TWBR = ((F_CPU / 400000) - 16) / 2; // change the I2C clock rate.

  encoder = new ClickEncoder(ENCODER_A_PIN, ENCODER_B_PIN, ENCODER_BTN_PIN, 4); // A, B, BTN, 4=stepsPerNotch 
  encoder->setAccelerationEnabled(false);

  ticker1.start(); //attach(1,cbtick1);
  ticker2.start(); //attach(1,cbtick1);

  reset_analog_values();

  delay(3000);
  usageScreen();
  delay(5000);

  single_read();  // this should preset the relevant variables aMin, aBucket, aMax, aCount to something sane

  lbg_bucket.begin();

  lcd.clear();
  btVT100clear();
  Serial.println(F("-end of setup"));
}

void loop(){
//  a) take another read
//  b)  every nth run, do 
//    - update tickers
//    - handle encoder
//    - handle encoder button
// 
//  takes approx 31useconds

static uint8_t z = 0;
//  a) take another read :
  single_read();

  if ( (++z % 11) == 0 ) {
    // update tickers :
    ticker1.update();
    ticker2.update();
  
    // encoder:
    encoder_value = encoder->getValue();
    if (encoder_value != 0) {
      encoder_last = encoder_value;
      //Serial.print("Encoder encoder_value: ");
      Serial.println(encoder_value);
      buzz(BUZ_TICK);
      if (edit == 1) { // band edit
        band += encoder_value;
        band = max( min(band, ANZ_BANDS-1), 0);
      } else if (edit ==2) { // attn edit
        attn += encoder_value;
        attn = max(min(attn, MAX_ATTN), 0);
      } else if (edit ==3) { // binc edit
        binc += encoder_value;
        binc = max(min(binc, MAX_BINC), 1);
      } else if (edit ==4) { // BT edit
        bt_output = ! bt_output;
      }
    }
    
    // encoder-button :
    encoder_button = encoder->getButton();
    if (encoder_button != ClickEncoder::Open) {
      //Serial.print("Button: ");
      switch (encoder_button) {
      case  ClickEncoder::Pressed :
      case  ClickEncoder::Held :
        {
          Serial.print(F("pressed or held "));
          Serial.println(encoder_button);
            break;
        }
      case  ClickEncoder::Clicked :
        {
          Serial.println(F("Clicked "));
          buzz(BUZ_CLICK);
          edit++;
          if (edit>= ANZ_EDITS) end_edit(); 
          break;
        }
      case  ClickEncoder::DoubleClicked :
        {
          Serial.println(F("Double Clicked "));
          //Serial.println(encoder_button);
          buzz(BUZ_CLICK); delay(5); buzz(BUZ_CLICK);
          end_edit();
          break;
        }
      }
    }
  } // %
}
