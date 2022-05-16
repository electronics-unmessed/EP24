// 
//  !!!!!!!!!  IMPORTANT HINT: Choose Arduino Nano Board and "ATmega328P (Old Bootloader) !!!!!!!!!!!
//
//          02/08/2022  Added Capability to show the Peak-hold on an old electromechanical mechanical Gauge
//                      Cool old school look!
//                      Gauge is controlled by PWM output on PIN D11
//                      Only 1 Potentiometer
//                      Switch Modes:   a) Peakhold - Peakhold time adjust by Potentiometer
//                                      b) Direct - Gain 1 .. 5 by Potentiometer
//                      1 LED flickering accoring the fieldstrength
//                      1 LED for TXT Mode
//                      1 LED for CSV Mode
//                      Works great!!
//          02/09/2022  debugging, tests several h
//          02/11/2022  adjusting LEDs brightness
//                      Pins 5, 9, 10: LEDs for CSV, TXT, Peak_hold are dimmed by the variable Bri
//                      For the NeoPixels, Bri is automatically kept in the range 40 .. 500 -> not relevant here
//                      For the LEDs Bri can be 0 .. 1000
//                      -> I can later adjust these by the default Bri, if too bright
//
//                      Test to find a well-balanced brightness for the LEDs
//
//                      Br = 250 (default)
//                                                    Pot.        fix
//                      yellow  peak-hold   0.20mA    2kOhm  ->   2k
//                      red     PWM RSSI    0.22mA    8k     ->   7.5k
//                      green   TXT Data    0.28mA    13k    ->   15k
//                      blue    CSV Data    0.24mA    11k    ->   8.2k
//
//          02/12/2022  Voltage Divider for analog inputs: Calculation, Test and Pre-Calibration 
//
//                      Calculation:  R1 = 330k, R2 = 100k, Trimmer = 10k
//                      
//          02/14/2022  changed to 5mm LEDs instead of 3mm -> increased default brigtness from 250 to 350.
//
//          02/15/2022  HW completely finished, additional tests, longterm tests
//                      Debugging overflow problem for PWM out by setting an upper limit to PWM_Funktion
//                      works perfect now!!
//
//          02/17/2022  Time calibration for Tn
//
// ************* P2 RF RSSI with 3 x Analog Input and Data Logging Arduino Uno and with NeoPixel Ring 16 ***************** *******
// ********************************** Version see below *********** *************************************************
//
//  Idea:     Pass several parameters via terminal program and report back
//
//            1. explanation with "Help"
//            2. Parameters for data logging: time interval, number of values, value type and many others
//            3. display via NeoPixel
//            4. Measure and record another 3 analog inputs voltage
//            5. RSSI on NeoPixel, gain 1 .. 5, peak hold 0.x sec .. 20 min
//            6. Switch for NeoPix to the three analog input voltages in 3 colors
//            7. LED display for data, external mode
//
// Status:    12/21/2020   Transfer of any parameters via string implemented
//                         Caution: String must not be larger than 63 characters!!
//            12/22/2020   Any order of the controls possible,
//                         Unchanged values ​​remain saved
//            12/23/2020   Clean version with all controls ready !!
//            12/24/2020   Small additions, many tests !!
//            12/25/2020   P2 Neo Pixel, Advanced Data Log Integration, tests, time correction
//            12/26/2020   tests with pixel colors, peak hold, etc. -> best integration so far !!
//            12/27/2020   Additions: PeakHold slowly falling, column headings,
//                         DutyCycle calculation and output, brightness control via serial
//            12/28/2020   error improved on output, resetting k was a problem
//            12/30/2020   Memory optimization through functions, subprograms
//                         reduced critical dynamic memory usage from 92% to 64%
//            01/15/2021   LED for endless CSV data and text data with a defined end separated
//                         Pin 10: Text data, Pin 11: Endless CSV data
//                         Sketch adapted to variable number of neopixels
//            01/16/2021   Programmed potentiometer control for gain and peakhold_time
//                         and tested
//            01/17/2021   LED brightness control via SW, linked to the brightness of the NeoPixel integrated
//                         Tests: Test shorter measurement intervals: 5 ms (DECT makes 10 ms pulses), my
//                         Default is 25 ms because then the time intervals can be better calibrated
//                         interrupt program for the measurement intervals?
//                         Integration switch: switching the display Ext A1 to dBmRSSI
//                         Many tests - still found errors, memory problem, output error, controls error
//            01/18/2021   TXT file output error corrected, was a (dyn.) memory problem!
//                         small optimizations in the peak display
//                         Dynamic memory optimized again by 25% by Serial.println(F("..Function
//                         Help function integrated
//                         Tests: Many with controls and PC -> runs stably and correctly!
//            01/23/2021   16 bit NeoPixel Ring Integration, no problems, but runs to the left!!
//                         Therefore LED control adapted for "counterclockwise" -> tested, works
//            01/28/2021   Input value divider for analog inputs A1, A2, A3. R = 100k + 100k
//                         This eliminates the floating input and extends the voltage range to 0 .. 10 volts
//            01/29/2021   When switching to external: display maximum values ​​of A1, A2, A3 in red, green, blue
//                         Realized in 2h, many tests
//            01/30/2021   Start-up display external mode improved.
//                         tests, tests, tests
//                           Brightness/Power: Max Brightness is 3 * 3 * 500 / 10 = 90
//                           -> NeoPix current about 100mA
//                          Arduino about 40mA
//                          RSSI chip 120mA = 60mA * 9V / (0.9 * 5V) step-up efficiency 90%
//
//                          Total approx. 260mA -> is OK for USB power supply or computer
//                          -> approx. 10h running time with 3000mAh power bank
//            02/25/201     Many tests.
//                          Error in peak value generation Gauge output RSSI fixed. Error with negative values
//                          fixed when creating peak values. Voltage inputs calibrated.
//                          Accuracy is about 0.1 volts, readings vary +-0.04 volts
//            03/11/2021    Peakhold function of the NeoPixel display improved. Relapse now occurs relatively quickly.
//                          Maximum peakhold time is now adjustable from 0...30 seconds
//                          Testing.
//                          Attention: Data logging only works via CoolTerm - not via Arduino serial monitor
//            03/12/2021    Peakhold exponential setting made. 0.4 ... 1200 seconds = 20 minutes
//                          Testing.
//
//
//  Open:                   RSSIdBm has peaks down to -80dBm (?) in "o" mode.
//
//  Others:
//                          01/30/2021 time calibration tests:
//                          1 hour 3600 readings -> finished 10 seconds too early, i.e. -0.28% deviation -> very good
//                          30 min 18000 readings -> finished 5 minutes late, i.e. +16% deviation -> fast measurement OK
//                          1 hour 360 readings -> finished 2.6 minutes too early, i.e. -4.4% deviation -> is OK, will
//                          1 hour 6 readings -> finished 4 minutes early
//
//
//
// ******* Amperage 24 LEDs ************************************** ************************************************** *
//  Testing:
//
//          Brightness  Level 1 25mA
//                      Level 5 30mA
//                      Level 10 36mA (would still be feasible without a separate power supply with Arduino Nano)
//                      Level 50 80mA
//                      Level 100 120mA
//
//                      Rule of thumb: current = 25mA + level * 1mA
//                      Maximum current would then be 255mA + 25mA = 280 mA
//
//
// ******************* explanation, help function ************************ ************************************************
// Explanation appears on Serial Interface
//

// **** debug mode off -> just comment out with // ********************************** *****************************
// 
// ************* P2 RF RSSI mit 3 x Analog Input und Data Logging Arduino Uno und mit NeoPixel Ring 16 ************************
// ********************************** Version siehe unten ************************************************************
//
// Idee:    Mehrere Parameter per Terminalprogramm übergeben und zurück melden
//
//          1. Erklärung mit "Help"
//          2. Parameter für Data Logging:  Zeitintervall, Anzahl der Werte, Werte Typ und viele andere
//          3. Anzeige per NeoPixel
//          4. Weitere 3 Analogeingänge Spannung messen und aufzeichnen
//          5. RSSI auf NeoPixel, Verstärkung 1 .. 5, Peakhold 0,x Sek .. 20 Min
//          6. Umschalter für NeoPix auf die drei Analog Input Spannungen in 3 Farben
//          7. LED Anzeige für Daten, Extern Modus
//
// Status:  21.12.2020  Übergabe beliebiger Parameter per String realisiert 
//                      Achtung: String darf nicht größer als 63 Characters sein!!
//          22.12.2020  Beliebige Reihenfolge der Controls möglich,
//                      Nicht veränderte Werte bleiben gesichert
//          23.12.2020  Saubere Version mit allen Controls fertig !!
//          24.12.2020  Kleine Ergänzungen, viele Tests !!
//          25.12.2020  P2 Neo Pixel, Advanced Data Log Integration, Tests, Zeitkorrektur
//          26.12.2020  Tests mit Pixelfarben, Peakhold usw. -> bisher beste Integration !!
//          27.12.2020  Ergänzungen: PeakHold fällt langsam, Spaltenüberschriften, 
//                      DutyCycle Berechnung u Ausgabe, Brightness Steuerung über Serial
//          28.12.2020  Fehler verbessert bei der Ausgabe, Rücksetzen von k war ein Problem
//          30.12.2020  Speicheroptimierung durch Funktionen, Unterprogramme 
//                      den kritischen dynamischen Speicherbedarf von 92% auf 64% reduziert
//          15.01.2021  LED für endlose CSV Daten und Text Daten mit definiertem Ende getrennt
//                      Pin 10: Text Daten,   Pin 11: Endlose CSV Daten
//                      Sketch auf variable Anzahl Neopixels angepasst
//          16.01.2021  Poti Steuerung für Gain und Peakhold_time programmiert
//                      und getestet
//          17.01.2021  LED Helligkeitssteuerung per SW, gekoppelt an Helligkeit der NeoPixel integriert
//                      Tests: Kürzere Messintervalle testen: 5 ms (DECT macht 10 ms Pulse), mein 
//                      Standard ist 25 ms, da dann die Zeitintervalle besser kalibriert werden können
//                      Interupt Program für die Messintervalle?
//                      Integration Schalter: Umschaltung der Anzeige Ext A1 zu dBmRSSI
//                      Viele Tests - noch Fehler gefunden, Speicherproblem, Ausgabefehler, Controls Fehler
//          18.01.2021  TXT File Ausgabe Fehler korrigiert, war ein (dyn.) Speicher Problem!
//                      kleine Optimierungen bei der Peak Anzeige
//                      Dyn. Speicher nochmals 25% optimiert durch Serial.println(F("..Funktion
//                      Help Funktion integriert
//                      Tests: Viele mit Controls und PC -> läuft stabil und korrekt!
//          23.01.2021  16 bit NeoPixel Ring Integration, problemlos, läuft aber links herum!!
//                      Daher LED Ansteuerung für "linksdrehend" angepasst -> getestet, läuft
//          28.01.2021  Eingangangsteiler für analoge Eingänge A1, A2, A3. R = 100k + 100k
//                      Damit wird der floating Input beseitigt und der Spannungsbereich auf 0 .. 10 Volt erweitert
//          29.01.2021  Bei Umschaltung auf Extern: Maxwerte von A1, A2, A3 in rot, grün, blau anzeigen
//                      in 2h realisiert, viele Tests
//          30.01.2021  Anlaufanzeige Extern Modus verbessert.
//                      Tests, Tests, Tests
//                      Helligkeit/Strom: Max Helligkeit ist 3 * 3 * 500 / 10 = 90 
//                      -> NeoPix Strom ca.   100mA 
//                      Arduino ca.            40mA
//                      RSSI Chip             120mA  =  60mA * 9V / (0.9 * 5V) Step-Up Wirkungsgrad 90%
//
//                      Summe ca.             260mA   -> ist OK für USB Netzteil oder Rechner
//                                                    -> ca. 10h Laufzeit mit 3000mAh Powerbank
//          25.02.201   Viele Tests. 
//                      Fehler bei Peakwertbildung Gauge Ausgabe RSSI behoben. Fehler bei dem negative Werte
//                      beim der Peakwertbildung entstehen behoben. Spannungseingänge kalibriert. 
//                      Genauigkeitkeit ist ca. 0.1 Volt, Messungen schwanken +-0.04 Volt
//          11.03.2021  Peakhold Funktion der NeoPixel Anzeige verbessert. Rückfall erfolgt jetzt relativ schnell.
//                      Peakhold-Zeit des Maximums ist jetzt 0 ... 30 Sekunden einstellbar
//                      Tests. 
//                      Achtung: Data Logging funktioniert nur noch über CoolTerm - nicht über Arduino Serieller Monitor
//          12.03.2021  Peakhold exponentielle Einstellung gemacht. 0,4 ... 1200 Sekunden = 20 Minuten
//                      Tests.
//
//
// Offen:               RSSIdBm hat im "o" Modus Peaks nach unten, bis -80dBm (?). 
//
// Sonstiges:     
//          30.01.2021  Zeitkalibrierung Tests:
//                      1 Stunde 3600 Messwerte -> 10 Sekunden zu früh fertig, also -0,28% Abweichung -> sehr gut
//                      30 min 18000 Messwerte -> 5 Minuten zu spät fertig, also +16% Abweichung -> schnelle Messung OK
//                      1 Stunde 360 Messwerte -> 2,6 Minuten zu früh fertig, also -4,4% Abweichung -> ist OK, wird 
//                      1 Stunde 6 Messwerte   -> 4 Minuten zu früh fertig
//
//
//
// ******* Stromstärken 24 LEDs *****************************************************************************************
//  Tests:
//
//  Helligkeit Stufe 1       25mA
//             Stufe 5       30mA
//             Stufe 10      36mA      ( wäre noch ohne separate Stromversorgung mit Arduino Nano realisierbar )
//             Stufe 50      80mA
//             Stufe 100    120mA
//
//  Faustregel: Strom = 25mA + Stufe * 1mA
//  Maximal Strom wäre dann 255mA + 25mA = 280 mA
// 
//
// ******************* Erklärung, Help Funktion *************************************************************************
// Erklärung erscheint auf Serial Interface
//

// **** Debug Modus aus -> einfach auskommentieren mit // ***************************************************************

//  #define debug           // bedingte Compilierung

  #include <Adafruit_NeoPixel.h>
  #define PIN  6
  #define NUMPIXELS 16
  
  #define Data_Int_Ext 8        // Input für Peak-hold-On Schalter
  
  Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
  
  const int analogInPin0 = A0;   // Analog input A0, 0 .. 5 V Spannung, für RSSI Signal Spannung
  const int analogInPin1 = A1;   // Analog input A1, 0 .. 5 V Spannung
  const int analogInPin2 = A2;   // Analog input A2, 0 .. 5 V Spannung
  const int analogInPin3 = A3;   // Analog input A2, 0 .. 5 V Spannung

  const int analogInPin4 = A4;   // Analog input vom Poti:  Gain
  const int analogInPin5 = A5;   // Analog input vom Poti:  Peakhold_time
  
  
  float counter = 0;
  float countPeak = 0;
  float countPeak_Pix = 0;

  float Gain = 1;                // Bereich: 1 .. 5, wirkt nur auf die NeoPixel Anzeige, später mit Poti steuern
  float GainNo = 0;              // Input Roh-Wert vom Poti: Gain
  float GainAlt = 0;             // Für Ausmittelung von kleinen Spannungsschwankungen benötigt
  
  float Peakhold_time = 60;      // Bereich Sekunden 0 .. 200, 
  float Peakhold_timeNo = 0;     // Input Roh-Wert vom Poti: Peakhold_time
  float Peakhold_timeAlt = 0;    // Für Ausmittelung von kleinen Spannungsschwankungen benötigt
  float Peakhold_exp = 1 ;       // für exponentielle Peakhold Time mit Poti
  
  float Funktion = 0;           // SignalStärke Input für LED NeoPixel Ansteuerung
  float FunktionPeak = 0;       // SignalStärke Input für LED NeoPixel Ansteuerung RSSI
  float FunktionPeak_Pix = 0 ;
  float FunktionPeakAlt = 0;
  float FunktionPeakAlt_Pix = 0;

  float FunktionPeak_1 = 0;       // SignalStärke Input für LED NeoPixel Ansteuerung Analog 1
  float FunktionPeakAlt_1 = 0;
  
  float FunktionPeak_2 = 0;       // SignalStärke Input für LED NeoPixel Ansteuerung Analog 2
  float FunktionPeakAlt_2 = 0;

  float FunktionPeak_3 = 0;       // SignalStärke Input für LED NeoPixel Ansteuerung Analog 3
  float FunktionPeakAlt_3 = 0;
  
  float dBmRSSI = 0;            // RF Signalstärke
  float dBmRSSImean = 0;
  float dBmRSSIpeak = -65;
  float DutyLevel = -50;        // Signalstärke, ab der "Duty Level" gemessen wird, einstellbar über Serial
  float DutyCycle = 0;          // gibt Duty Cycle in % an
  
  float inputA0 = 0;            // Variable für analoge Eingänge
  float inputA1 = 0;
  float inputA2 = 0;
   float inputA3 = 0;
 

  float inputA4 = 0;            // Input vom Poti: Gain
  float inputA5 = 0;            // Input vom Poti: Peakhold_time

 
  float voltA0 = 0;               // Wert in Volt umgerechnet 
  float voltA1 = 0;               // Wert in Volt umgerechnet 
  float voltA2 = 0;               // Wert in Volt umgerechnet 
   float voltA3 = 0;               // Wert in Volt umgerechnet 
 
  
  float E_Funktion = 0;           
  
  // float TestFunktion = 0;
  // float Rechteck1 = 0;
  
  float d = 0;                      // counter for Duty Cycle
  float e = 0;                      // counter for Duty Cycle
  
  float E_FunktionMax = 1000;
  int Anzahl_LED = 16;              // Anzahl der NeoPixel LEDs

float PWM_Funktion = 0 ;            // Old Gauge
  

// *********** Farben und Helligkeit ************************************************************************************
// *********** Hintergrund NeoPixels ************************************************************************************
  
  int HG_rot  = 0;  // bleibt dunkel
  int HG_gruen = 0;
  int HG_blau = 0;

// *********** PeakWerte Anzeige über NeoPixels *************************************************************************
  
  float PW_rot  = 9;  // rot
  int PW_gruen = 0;
  float PW_blau = 0;

// *********** Vordergrund Anzeige über NeoPixels ***********************************************************************
  
  int VG_rot  = 3;
  int VG_gruen = 3; // grün
  int VG_blau = 3;

// **********************************************************************************************************************
// ******** Testfunktion ************************************************************************************************

// float Testfunktion1 = 0;
// float Testfunktion2 = 0;
// float Testfunktion3 = 0;

// ************** Ausgabe ***********************************************************************************************
  
  float A0_F = 0;   // Messwerte
  float A1_F = 0;
  float A2_F = 0;
   float A3_F = 0;
  
  float Rs_F = 0;   // Messwerte umgerechnet in RSSI 
  float Rm_F = 0;
  float Rp_F = 0;
  
  float Dc_F = 0;   // Messwert Duty Cycle
  
  String A0_F_S = "";   // String für die Messwerte
  String A1_F_S = "";
  String A2_F_S = "";
   String A3_F_S = "";
 
  
  String Rs_F_S = "";   // String für die Messwerte
  String Rm_F_S = "";
  String Rp_F_S = "";
  
  String Dc_F_S = "";   // String für die Messwerte

// ********* Internal Measurement Timings *******************************************************************************
  
  long int Increment = 0;
  float TaktZeit = 0;
  float EchtZeit = 0;
  float AusgabeZeit = 0;
  String AusgabeZeit_S = "";
  
//  float Zeitkorr = 1.349;     // Time calibration 
  float Zeitkorr = 1.3333;     // Time calibration up-> faster, down->slower
  
  int MeasDelay = 25;             // intene Messintervalle in ms
  
  String Control_1;
  String shortInput;
  String gesamtInput_S;
  String com1;

// ***** Log Timing Parameters & Defaults *******************************************************************************
  
  long int Interval = 1000;     // Mess-Intervall in Millisekunden
  long int Number = 1000;        // Anzahl der Messwerte
  float Peakhold = 1;           // Intervall in Sekunden für neue Peakhold Berechnung
// +++++++++++++++++++++++++++++++++++++++++++++
  float Peakhold_Pix = 5 ;   // Intervall in Sekunden für neue Peakhold Berechnung für NeoPixelAnzeige 

// ++++++++++++++++++++++++++++++++++++++++++++
  
  float AverageInt = 20;        // Anzahl der Werte für gleitenden Mittelwert
  
  float PeakholdNo = 1;          // wird aus Peakhold Zeit und MeasDelay berechnet
  float PeakholdNo_Pix = 1;       // dito für Neopixel
  
  long int m = 0;
  long int n = 0;
  long int k = 0;
  
  String com;
  String Mode_S;
  String Interval_S;
  String Number_S;
  String AverageInt_S;
  String Peakhold_S;
  String DutyCycle_S;
  String DutyLevel_S;

// ******* Log Columns Steuerung und Defaults ***************************************************************************
  
  String Col_0 = "No";
  String Col_1 = "Ti";
  String Col_2 = "A1";
  String Col_3 = "A2";
  String Col_4 = "A3";
  String Col_5 = "Rs";
  String Col_6 = "Rm";
  String Col_7 = "Rp";
  String Col_8 = "Dc";
  
  String Col_00 = "";
  String Col_10 = "";
  String Col_20 = "";
  String Col_30 = "";
  String Col_40 = "";
  String Col_50 = "";
  String Col_60 = "";
  String Col_70 = "";
  String Col_80 = "";
  
  String Col_0_S;
  String Col_1_S;
  String Col_2_S;
  String Col_3_S;
  String Col_4_S;
  String Col_5_S;
  String Col_6_S;
  String Col_7_S;
  String Col_8_S;
  
  String Spalte;      // Ausgabe
  
  String Brightness_S;  // String für Helligkeit

// ******* Gauge Steuerung und Defaults *********************************************************************************
  
  float Brightness = 350;  // Gesamte Helligkeit der LEDs, gültige Werte 1 bis 10 (255)
  float Bri = 1;          // Variable Helligkeit für NeoPixel
  

// ********* LEDs Steuerung ********************************************************************************

  int Data_LED_external = 9;      // definiert die PIN Nummer LED Anzeige für Daten von A1 (analog)
  int Data_LED = 10;              // definiert die PIN Nummer LED Anzeige für Datenfluss
  int Data_LED_endless = 5;      // definiert die PIN Nummer LED Anzeige für Endlos Datenfluss
  float Bri_LED = 10;             // Variable Helligkeit für LED

  int Switch_Status = 1;          // Schalter Status
  int Switch_Status_Alt = 1;   

  int Gauge_PWM_Output = 11;

 float GainOldGauge = 1 ;
  float GainOldGaugeNo = 0;            
  float GainOldGaugeAlt = 0;             
 

// ******************* Ende Definitions *********************************************************************************************
// ***********************************************************************************************************************

// ***********************************************************************************************************************
// ******************** Set-up *******************************************************************************************

void setup() {
 Serial.flush();
   Serial.begin(38400);
   Serial.setTimeout(200);     // Standard ist 1000ms
   pinMode(Data_LED, OUTPUT);
   pinMode(Data_LED_endless, OUTPUT);
   pinMode(Data_LED_external, OUTPUT);
   pinMode(Data_Int_Ext, INPUT_PULLUP);

   pinMode(Gauge_PWM_Output, OUTPUT);
   
   n = 0;
   k = 0;

Help();
  Serial.flush();
  Serial.println(F(" "));
  Serial.println(F("Default settings:"));
  
ParameterAusg();
  Serial.flush();
}

// **********************************************************************************************************************
// **************************** Hauptprogramm ***************************************************************************
// **********************************************************************************************************************

void loop() {

// **************** Serial String Leseroutine ***************************************************************************
  
 // Serial.flush();
  
  counter = counter+0.15;
  countPeak = countPeak+1;
  countPeak_Pix = countPeak_Pix +1 ;
  
  // if (cos(0.5*counter) > 0.-0.5) {Rechteck1 = 1;} else {Rechteck1=0;};
  // TestFunktion = 2400 * (0.5 + 0.5* cos(counter) * cos(1.3*counter) * Rechteck1);

// ********** Analogwert A0, A1, A2 lesen ********************************************************************************************************************

  inputA0 = analogRead(analogInPin0);
  inputA1 = analogRead(analogInPin1);
  inputA2 = analogRead(analogInPin2);
  inputA3 = analogRead(analogInPin3);

// *********** Voltage in Volts berechnen ********************************************************************************* 

// *** Kalibrierungsfaktor, um die Toleranzen des Eingangsspannungsteilers auszugleichen

  voltA0 = 0.00243 * map(inputA0,0,1023,0,2000);

  


// +++++++++++++++++++ Calibration for 0 .. 20 Volts +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  voltA1 = 4.1 * 0.00243 * map(inputA1,0,1023,0,2000);   // Faktor 2, da Eingangsteileer 100k + 100k verwendet wird
  voltA2 = 4.1 * 0.00243 * map(inputA2,0,1023,0,2000);   // Faktor 2, da Eingangsteileer 100k + 100k verwendet wird
  voltA3 = 4.1 * 0.00243 * map(inputA3,0,1023,0,2000);   // Faktor 2, da Eingangsteileer 100k + 100k verwendet wird

// ********** Umrechnung in dBm RSSI **************************************************************************************
  
  dBmRSSI = 23-(inputA0/210.0*1000/24.5);                                           // Berechnung RSSI aus A0 - OK
  dBmRSSImean = (1 - 1/AverageInt) * dBmRSSImean + (1/AverageInt) * dBmRSSI;        // Berechnung RSSImean 
  PeakholdNo = 1000 * Peakhold / MeasDelay;
  if (dBmRSSI > dBmRSSIpeak) {dBmRSSIpeak = dBmRSSI; countPeak=0;};
  if (countPeak > PeakholdNo) {countPeak = 0; dBmRSSIpeak = dBmRSSImean;};
  
// ********* Berechnung Duty Cycle ***************************************************************************************

  if (dBmRSSI > DutyLevel && EchtZeit < Interval) {d = d+1;};
  if (EchtZeit < Interval) {e = e+1;};
       // Serial.print(EchtZeit);Serial.print(" , ");


// ***************** Potentiometer Steuerung Gain ***************************************************************************************************
 
   inputA4 = analogRead(analogInPin4);
   GainNo = map(inputA4,0,1023,0,1023);

   GainAlt = Gain;
   Gain = 1 + GainNo/255;      // Gain von 1 .. ?? ausprobieren!
   Gain = 0.7*GainAlt + 0.3*Gain; // mittelt Störpeaks aus

   // Serial.print(Gain) ; Serial.print("  , ") ;

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

   inputA5 = analogRead(analogInPin5);
   GainOldGaugeNo = map(inputA5,0,1023,0,1023);

   GainOldGaugeAlt = GainOldGauge;
   GainOldGauge = 1 + GainOldGaugeNo/255;      // Gain von 1 .. ?? ausprobieren!
   GainOldGauge = 0.7 * GainOldGaugeAlt + 0.3*GainOldGauge; // mittelt Störpeaks aus
  


// ***************** Potentiometer Steuerung Peakhold_time  *******************************************************************************************
 
   inputA5 = analogRead(analogInPin5);
   Peakhold_timeNo = map(inputA5,0,1023,0,1023);

   Peakhold_timeAlt = Peakhold_time;
   Peakhold_time = 0.5 * Peakhold_timeNo;      // Peakhold_time von 0 .. 100 (grob in Sekunden)
   Peakhold_time = 0.7 * Peakhold_timeAlt + 0.3 * Peakhold_time; // mittelt Störpeaks aus

  // Serial.println(Peakhold_time) ;


#ifdef debug 
   Serial.print("         Gain:"); Serial.print("  ,  " ); Serial.print(Gain); Serial.print("  ,  " ); Serial.println(GainNo);
   Serial.print("Peakhold_time:"); Serial.print("  ,  " ); Serial.print(Peakhold_time); Serial.print("  ,  " ); Serial.println(Peakhold_timeNo); 
   delay(100);
#endif

// ************ Umschalten intern dBmRSSI auf A1 Voltage  **************************************************************************************************

 Switch_Status  = digitalRead(Data_Int_Ext);  // Schalter Status

 // if (Switch_Status < 0.5) {analogWrite(Data_LED_external, Bri_LED); Ext_A1_Gauge();} else {analogWrite(Data_LED_external, 0); dBmRSSI_Gauge();};
if (Switch_Status < 0.5) {analogWrite(Data_LED_external, Bri_LED); } else {analogWrite(Data_LED_external, 0); };

//
// +++++++++++++++++++++++++++++++++++++++++++++++++ Old Gauge Control ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

dBmRSSI_Gauge();

// if (Switch_Status < 0.5) {analogWrite(Gauge_PWM_Output, 10);} else {analogWrite(Gauge_PWM_Output, 13 * FunktionPeak); };

PWM_Funktion = GainOldGauge * 13 * Funktion ;

if (PWM_Funktion > 13*18) {PWM_Funktion = 13*18;} 

if (Switch_Status > 0.5) {analogWrite(Gauge_PWM_Output, PWM_Funktion);} else {analogWrite(Gauge_PWM_Output, 13 * FunktionPeak); };

// XXXXX

 // Serial.print("PWM_Funktion:  ") ; Serial.println(PWM_Funktion) ; 
//  Serial.print("Funktion:  ") ; Serial.println(Funktion) ; 
// delay(300) ;
 
    if (Switch_Status != Switch_Status_Alt) {counter = 0;};
 Switch_Status_Alt = Switch_Status;
 
/*
// ************* alt alt Ansteuerung der NEO Pixel LEDs rechtsdrehend ********************************************************

  pixels.begin();
  for (int j=0; j<16; j++) { 
 
  Bri = Brightness/50;
  
  if (Bri < 0.4) {Bri = 0.4;};  // Sicherungsfunktion!! damit Display nicht ganz ausgeht -> würde unnötige Fehlersuche verursachen
  if (Bri > 10) {Bri = 10;};   // Sicherungsfunktion!! damit Strom nicht zu hoch wird für Arduino bzw. USN

// *** wenn veränderliche Farben möchte, kann man das machen *************************************************************
//  PW_rot = 24 - 1.2*FunktionPeak;
//  if (PW_rot < 0) {PW_rot =0;};
//  PW_blau = 0.8*FunktionPeak; 
//  Serial.print("r ,b ");Serial.print(" , ");Serial.print(PW_rot);Serial.print(" , ");Serial.println(PW_blau);
//  delay(5);

  if (FunktionPeak > j) {pixels.setPixelColor(j, pixels.Color(Bri*PW_rot,Bri*PW_gruen,Bri*PW_blau)); } 
     else {pixels.setPixelColor(j, pixels.Color(Bri*HG_rot,Bri*HG_gruen,Bri*HG_blau));}
       if (FunktionPeak > j+1) {pixels.setPixelColor(j, pixels.Color(0,0,0));}; 
         if (Funktion > j) {pixels.setPixelColor(j, pixels.Color(Bri*VG_rot,Bri*VG_gruen,Bri*VG_blau));} 
  }; 
  pixels.show();

// ***********************************************************************************************************************


// ************* alt alt Ansteuerung der NEO Pixel LEDs linksdrehend ******************************************************

  pixels.begin();
  for (int j=0; j<16; j++) { 
 
  Bri = Brightness/50;
  
  if (Bri < 0.4) {Bri = 0.4;};  // Sicherungsfunktion!! damit Display nicht ganz ausgeht -> würde unnötige Fehlersuche verursachen
  if (Bri > 10) {Bri = 10;};   // Sicherungsfunktion!! damit Strom nicht zu hoch wird für Arduino bzw. USN

  if (Anzahl_LED-1-FunktionPeak < j) {pixels.setPixelColor(j, pixels.Color(Bri*PW_rot,Bri*PW_gruen,Bri*PW_blau)); } 
     else {pixels.setPixelColor(j, pixels.Color(Bri*HG_rot,Bri*HG_gruen,Bri*HG_blau));}
       if (Anzahl_LED-1-FunktionPeak < j-1) {pixels.setPixelColor(j, pixels.Color(0,0,0));}; 
         if (Anzahl_LED-Funktion-1 < j) {pixels.setPixelColor(j, pixels.Color(Bri*VG_rot,Bri*VG_gruen,Bri*VG_blau));} // OK funktioniert
  }; 

  pixels.show();

// ***********************************************************************************************************************
*/


// *** Liest alle Characters aus gesendetem String ***********************************************************************

  if (Serial.available() !=0) {
  //   Serial.flush();
    gesamtInput_S = Serial.readString(); 
             #ifdef debug
             Serial.print("m = "); Serial.print(m); Serial.print("  "); Serial.print("Input = "); Serial.println(gesamtInput_S);
            #endif
    Increment=1;
    n=1; k=1;
    counter=0;  // quittiert jedem Serial Input mit Vollausschlag, der aber nicht auf die Ausgabewerte geht
     Serial.flush();
  }
  
  parseCommand(com);
  Serial.flush();

  while (Increment > 0 && Increment < 2 ) {Increment=Increment+1; 
    Serial.println("");
    Serial.println("Current settings:");
    ParameterAusg();
    Serial.flush();

// **** Ausgabe der Spalten-Überschriften ********************************************************************************

    if (shortInput =="run") {AusgabeZeit=0; 

      SpaltenAusgabe(Col_00, Col_0);
      SpaltenAusgabe(Col_10, Col_1);
      SpaltenAusgabe(Col_20, Col_2);
      SpaltenAusgabe(Col_30, Col_3);
      SpaltenAusgabe(Col_40, Col_4);
      SpaltenAusgabe(Col_50, Col_5);
      SpaltenAusgabe(Col_60, Col_6);
      SpaltenAusgabe(Col_70, Col_7);
      SpaltenAusgabe(Col_80, Col_8);
      Serial.println();} 
      Serial.flush();  
  }; 

 Increment = Increment + 1;

// ******** Testfunktion *************************************************************************************************

    // Testfunktion1 = 200 * cos(0.001*Increment);
    // Testfunktion2 = 200 * cos(0.01*Increment);
    // if (cos(0.02*Increment) > 0) {Testfunktion3 = 200;} else {Testfunktion3 = 0;};
    
    // A0_F = Testfunktion1;
    // A1_F = Testfunktion2;
    // A2_F = Testfunktion3;
  
  A0_F = voltA0;
  A1_F = voltA1;
  A2_F = voltA2;
  A3_F = voltA3;
  
  Rs_F = dBmRSSI;
  Rm_F = dBmRSSImean;
  Rp_F = dBmRSSIpeak;

// *********** Zeiten und Korrektur **************************************************************************************
  
  TaktZeit = TaktZeit + MeasDelay;
  EchtZeit = EchtZeit + MeasDelay*Zeitkorr;
  

// ******** Ausgabe an Serial ********************************************************************************************
// ****** Ausgabe variabler Modus ****************************************************************************************
  
    analogWrite(Data_LED, 0);
    analogWrite(Data_LED_endless, 0);
     // digitalWrite(Data_LED_endless, LOW);
    Bri_LED = Brightness/7;
  
  if (shortInput =="run" && EchtZeit >= Interval) {
   if (n<Number+1){
    AusgabeZeit = AusgabeZeit + Interval;
     // digitalWrite(Data_LED_endless, HIGH);
    analogWrite(Data_LED, Bri_LED);
    Ausgabe();
    n=n+1; k=n;
    DutyCycle = 100*d/e; d=0; e=0;
    Dc_F = DutyCycle;
   //   Serial.print(d);Serial.print(" , ");Serial.print(e);Serial.print(" , ");Serial.println(DutyCycle);   // Test Print
    EchtZeit = 0;};
   if (n>Number) {shortInput = "stop";};
    };
    
// ****** Ausgabe Oszi Modus für Arduino Serial Plotter ******************************************************************
// ************* Mess Intervall berücksichtigt ***************************************************************************

// +++++++++ hier separate LED ansprechen für endlose Daten +++++++++++++++

  if (shortInput =="Rs_oscilloscope" && EchtZeit >= Interval) {
   // digitalWrite(Data_LED_endless, HIGH);
        analogWrite(Data_LED_endless, Bri_LED);
   Serial.print(Rs_F); Serial.print(" , ");
   Serial.print(Rm_F); Serial.print(" , ");
   Serial.print(Rp_F); Serial.print(" , ");
   Serial.println();
   EchtZeit = 0;  
   Serial.flush();};
  //  delay(500);

  if (shortInput =="Ax_oscilloscope" && EchtZeit >= Interval) {
   // digitalWrite(Data_LED_endless, HIGH);
        analogWrite(Data_LED_endless, Bri_LED);
   Serial.print(A0_F); Serial.print(" , ");
   Serial.print(A1_F); Serial.print(" , ");
   Serial.print(A2_F); Serial.print(" , ");
   Serial.print(A3_F); Serial.print(" , ");
   Serial.println();
   EchtZeit = 0;  
   Serial.flush();};
  //  delay(500);

// ****** Ausgabe Telemetry Modus **************************************************************************************** 
// ************* Mess Intervall berücksichtigt ***************************************************************************

if (shortInput =="telemetry" && EchtZeit >= Interval) {
   DutyCycle = 100*d/e; d=0; e=0;
   Dc_F = DutyCycle;
  // digitalWrite(Data_LED_endless, HIGH);
       analogWrite(Data_LED_endless, Bri_LED);
   Serial.print(Rs_F); Serial.print(",");
   Serial.print(Rm_F); Serial.print(",");
   Serial.print(Rp_F); Serial.print(",");
   Serial.print(Dc_F); Serial.print(",");
   Serial.print(A0_F); Serial.print(",");
   Serial.print(A1_F); Serial.print(",");
   Serial.print(A2_F); Serial.print(",");
   Serial.print(A3_F); Serial.print(",");  
   Serial.println();
   EchtZeit = 0; 
     Serial.flush();};

   delay(MeasDelay);   
}

// ************ Ende Hauptprogramm ***************************************************************************************
// ***********************************************************************************************************************


// ************************* Funktion Hilfe: Erklärung via Serial String *************************************************
// **** Achtung: Dieser Text war zu viel für das Setup ... führt zu Fehlern in der Ausgabe, daher als Funktion hier ******
// **** Speicherproblem mit Serial.println(F(" ...")) gelöst, 20% Speicher gespart ***************************************

void Help() {

 // Serial.flush();
  Serial.println(F(" "));
  Serial.println(F(" "));
  Serial.println(F("*** RSSI & Data Logger *** Verison P2_V2_4_3 (Old School Moving Coil Gauge) *** "));
  Serial.println(F(" "));
  Serial.println(F("  => => => =>  Help on Controls  <= <= <= <= "));
  Serial.println(F(" "));
  Serial.println(F("Short Controls:"));
  Serial.println(F("   Help:        'h'"));
  Serial.println(F("   Stop Data:   's'"));
  Serial.println(F("   Endless CSV: 'a'= oszi: A0, A1, A2, A3; 'o'= oszi: Rs, Rm, Rp; "));
  Serial.println(F("                't'= telem: All data; "));
  Serial.println(F("   Run def TXT: 'r'"));
  Serial.println(F(" "));
  Serial.println(F("Settings TXT (send String: eg 'Tn=500;'):"));
  Serial.println(F("   Tn = Interv/msec (25 .. arb); No = NumSampl (1 .. arb); "));
  Serial.println(F("   Av = #FloatAv; Pe = Peakhold/sec; Dl = DutyLev/dBm; "));
  Serial.println(F(" "));
  Serial.println(F("Settings TXT Columns def (send String: eg 'C0=Ti;'):"));
  Serial.println(F("   'C0, C1, ... = No, Ti, A0, A1, A2, Rs, Rm, Rp, Dc;'"));
  Serial.println(F("   'C0, C1, ... = xx;' neglect column"));
  // Serial.println(F(" "));
  // Serial.println(F("Settings General (send String: eg 'Br=500;'):"));
  // Serial.println(F("   'Br = 10 .. 500;' Brigthness Gauge & LEDs"));
  Serial.println(F(" ")); 
  Serial.println(F("Measurement TXT Columns/Units & Info:"));
  Serial.println(F("   No; Ti/s; A0/V; A1/V; A2/V; A3/V; Rs/dBm; Rm/dBm; Rp/dBm; Dc/%;"));
  Serial.println(F("   A0 for RSSI and Duty cycle;  A1, A2, A3, external inputs 0 .. 10V"));
  Serial.println(F(" "));
  Serial.println(F("  => => => => =>   End Help   <= <= <= <= <= "));
  Serial.println(F(" "));
 // Serial.println(F("Default settings:"));
 Serial.flush();
}

// ************* Ende Funktion Hilfe ... ********************************************************************************


// ********** Funktion: Umrechnung Ext A1 auf die Eingangsfunktion ******************************************************
// +++++ erweitert auf alle Eingfänge A1, A2, A3 mit rot, grün, blau ++++++++++++++++++++++++

void Ext_A1_Gauge() {
   
// **** Verarbeitung A1 ******************************************************************** 


  FunktionPeak_1 = 1.2 + Gain * E_FunktionMax/5 * 0.5* voltA1 * Anzahl_LED / E_FunktionMax;    // Faktor 0.5, voltA1 aufgrund Eingangsteiler mit 2 multipliziert wird
    
      if (FunktionPeak_1 < FunktionPeakAlt_1) {FunktionPeak_1 = FunktionPeakAlt_1 -(1/(0.9+Peakhold_time));};  // ** Langsamer Rückgang der Peak Anzeige
        if (FunktionPeak_1 >= Anzahl_LED) {FunktionPeak_1 = Anzahl_LED; FunktionPeakAlt_1 = Anzahl_LED; };       
    FunktionPeakAlt_1 = FunktionPeak_1;

 if (0 < counter && counter < 6.3) {FunktionPeak_1 = 1 + 0.5 * (Anzahl_LED-1) * (1.1 - cos(counter));};  // ** Start Test und Quittierung von Serial Input

 // if (FunktionPeak_1 < 0  || FunktionPeakAlt_1 < 0 ) {FunktionPeakAlt_1 = 0; FunktionPeak_1 =0;};


// ***** Ende A1 *****************************************************************************

// **** Verarbeitung A2 ******************************************************************** 

  FunktionPeak_2 = 1.2 + Gain * E_FunktionMax/5 * 0.5* voltA2 * Anzahl_LED / E_FunktionMax;    // Faktor 0.5, voltA1 aufgrund Eingangsteiler mit 2 multipliziert wird
    
      if (FunktionPeak_2 < FunktionPeakAlt_2) {FunktionPeak_2 = FunktionPeakAlt_2 -(1/(0.9+Peakhold_time));};  // ** Langsamer Rückgang der Peak Anzeige
        if (FunktionPeak_2 >= Anzahl_LED) {FunktionPeak_2 = Anzahl_LED; FunktionPeakAlt_2 = Anzahl_LED; };   
    FunktionPeakAlt_2 = FunktionPeak_2;
    
  if (6.3 < counter && counter < 12.6) {FunktionPeak_2 = 1 + 0.5 * (Anzahl_LED-1) * (1.1 - cos(counter));};       // ** Start Test und Quittierung von Serial Input

// ***** Ende A2 *****************************************************************************


// **** Verarbeitung A3 ******************************************************************** 

  FunktionPeak_3 = 1.2 + Gain * E_FunktionMax/5 * 0.5* voltA3 * Anzahl_LED / E_FunktionMax;    // Faktor 0.5, voltA1 aufgrund Eingangsteiler mit 2 multipliziert wird
    
      if (FunktionPeak_3 < FunktionPeakAlt_3) {FunktionPeak_3 = FunktionPeakAlt_3 -(1/(0.9+Peakhold_time));};  // ** Langsamer Rückgang der Peak Anzeige
        if (FunktionPeak_3 >= Anzahl_LED) {FunktionPeak_3 = Anzahl_LED; FunktionPeakAlt_3 = Anzahl_LED; };   
    FunktionPeakAlt_3 = FunktionPeak_3;

 if (12.6 < counter && counter < 18.9) {FunktionPeak_3 = 1 + 0.5 * (Anzahl_LED-1) * (1.1 - cos(counter));};       // ** Start Test und Quittierung von Serial Input

// ***** Ende A3 *****************************************************************************


    
// ************* Ansteuerung der NEO Pixel LEDs linksdrehend *****************

  pixels.begin();
  for (int j=0; j<16; j++) { 
 
  Bri = Brightness/50;
  
  if (Bri < 0.4) {Bri = 0.4;};  // Sicherungsfunktion!! damit Display nicht ganz ausgeht -> würde unnötige Fehlersuche verursachen
  if (Bri > 10) {Bri = 10;};   // Sicherungsfunktion!! damit Strom nicht zu hoch wird für Arduino bzw. USN

// ++++ Drei verschiedene Eingänge A1, A2, A3 müssen auf Farbenebene gesteuert werden ++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++ aber wie? 

 // if (Anzahl_LED-1-FunktionPeak < j) {j,pixels.setPixelColor(j, pixels.Color(Bri*PW_rot,Bri*PW_gruen,Bri*PW_blau)); } 
  //  else {pixels.setPixelColor(j, pixels.Color(Bri*HG_rot,Bri*HG_gruen,Bri*HG_blau));} 
 //    if (Anzahl_LED-1-FunktionPeak < j-1) {pixels.setPixelColor(j, pixels.Color(0,0,0));}; 
  //     if (Anzahl_LED-Funktion-1 < j) {pixels.setPixelColor(j, pixels.Color(Bri*VG_rot,Bri*VG_gruen,Bri*VG_blau));} // OK funktioniert

  if (Anzahl_LED-1-FunktionPeak_1 < j) {PW_rot = 3 * VG_rot;};
  if (Anzahl_LED-1-FunktionPeak_2 < j) {PW_gruen = 3 * VG_gruen;};
  if (Anzahl_LED-1-FunktionPeak_3 < j) {PW_blau = 3 * VG_blau;};
 
  //  else {pixels.setPixelColor(j, pixels.Color(Bri*HG_rot,Bri*HG_gruen,Bri*HG_blau));} 
     if (Anzahl_LED-1-FunktionPeak_1 < j-1 || FunktionPeak_1 <= 0) {PW_rot = 0;}; 
      if (Anzahl_LED-1-FunktionPeak_2 < j-1 || FunktionPeak_2 <= 0) {PW_gruen = 0;}; 
      if (Anzahl_LED-1-FunktionPeak_3 < j-1 || FunktionPeak_3 <= 0) {PW_blau = 0;}; 
     
    pixels.setPixelColor(j, pixels.Color(Bri*PW_rot,Bri*PW_gruen,Bri*PW_blau));
  }; 

  pixels.show();

// **************************************************************

}

// **************** Ende Funktion Umrechnung Ext A1 auf .. **************************************************************



// ********** Funktion: Umrechnung dBmRSSI auf die Eingangsfunktion ******************************************************

void dBmRSSI_Gauge() {
  
  E_Funktion = Gain*E_FunktionMax/70*(65 + dBmRSSI); 
  
  Funktion = 1 + 0.93 * E_Funktion * Anzahl_LED / E_FunktionMax;
  
   FunktionPeak = 1 + 0.93 * Gain*E_FunktionMax/70*(65 + dBmRSSI) * Anzahl_LED / E_FunktionMax;
    
 //  if (counter < 7) {FunktionPeak = Anzahl_LED;};                                   // ** Start Test und Quittierung von Serial Input
 
   FunktionPeak_Pix = FunktionPeak ;

   Peakhold_exp = 8 * pow(2, Peakhold_time/42) ;   // 
   PeakholdNo_Pix = 25 * Peakhold_exp / MeasDelay; // Zähler für die Peakhold Time
     
  if (FunktionPeak_Pix > FunktionPeakAlt_Pix) {FunktionPeakAlt_Pix = FunktionPeak_Pix; countPeak_Pix=0;};
  if (countPeak_Pix < PeakholdNo_Pix) {FunktionPeak = FunktionPeakAlt_Pix; } ;
  if (countPeak_Pix > PeakholdNo_Pix) {countPeak_Pix = 0; FunktionPeakAlt_Pix = 0; } ;

 /*
  Serial.print("PeakholdNo_Pix:  ") ; Serial.print(PeakholdNo_Pix) ;  
   Serial.print("  , Peakhold_time:  ") ; Serial.println(Peakhold_exp) ; 
  delay(30);
 */
 
//   if (FunktionPeak < FunktionPeakAlt) {FunktionPeak = FunktionPeakAlt - 0.4;};  // ** Langsamer Rückgang der Peak Anzeige

   if (FunktionPeak < FunktionPeakAlt) {FunktionPeak = FunktionPeakAlt - 2.0;};  // ** Slows down the analog hand of the gauge

    if (FunktionPeak < 0) {FunktionPeak = 0.0;}; 

    if (Funktion < 0) {Funktion = 0.0;}; 
   
   if (FunktionPeak >= Anzahl_LED) {FunktionPeak = Anzahl_LED; FunktionPeakAlt = Anzahl_LED; };  
 
   FunktionPeakAlt = FunktionPeak;
  //   FunktionPeakAlt_Pix = FunktionPeak_Pix;
    
 //  if (counter < 3.1415/2) {Funktion = 0.6 * Anzahl_LED * (1 - cos(counter));};       // ** Start Test und Quittierung von Serial Input
      
// ************* Ansteuerung der NEO Pixel LEDs linksdrehend *****************

  PW_rot=9; // verhindert Fehler, dass bei zu schnellem Umschalten der Peakwert verschwindet 
  PW_gruen=0; // oder die falsche Farbe auftritt
  PW_blau=0; // oder die falsche Farbe auftritt
  
  pixels.begin();
  for (int j=0; j<16; j++) { 
 
  Bri = Brightness/50;
  
  if (Bri < 0.4) {Bri = 0.4;};  // Sicherungsfunktion!! damit Display nicht ganz ausgeht -> würde unnötige Fehlersuche verursachen
  if (Bri > 10) {Bri = 10;};   // Sicherungsfunktion!! damit Strom nicht zu hoch wird für Arduino bzw. USN

  if (Anzahl_LED-1-FunktionPeak < j) {pixels.setPixelColor(j, pixels.Color(Bri*PW_rot,Bri*PW_gruen,Bri*PW_blau)); } 
     else {pixels.setPixelColor(j, pixels.Color(Bri*HG_rot,Bri*HG_gruen,Bri*HG_blau));}
       if (Anzahl_LED-1-FunktionPeak < j-1) {pixels.setPixelColor(j, pixels.Color(0,0,0));}; 
         if (Anzahl_LED-Funktion-1 < j) {pixels.setPixelColor(j, pixels.Color(Bri*VG_rot,Bri*VG_gruen,Bri*VG_blau));} // OK funktioniert
  }; 


/*
Serial.print("FunctionPeak:  ") ; Serial.println(FunktionPeak) ; 
Serial.print("inputA5:  ") ; Serial.println(inputA5) ; 
Serial.print("Peakhold_timeNo:  ") ; Serial.println(Peakhold_timeNo) ; 

delay(300);
*/


// analogWrite(Gauge_PWM_Output, 20);

// analogWrite(Gauge_PWM_Output, 13 * FunktionPeak);

// if (Switch_Status < 0.5) {analogWrite(Gauge_PWM_Output, 13 * FunktionPeak); } else {analogWrite(Gauge_PWM_Output, 100);};



  pixels.show();

// **************************************************************

}

// **************** Ende Funktion Umrechnung dBmRSSI auf .. **************************************************************


// ************ Funktion: Spalten Ausgabe Überschrift für variable Ausgabe getriggert mit "r" ****************************

void SpaltenAusgabe(String Spalte_Masseinheit, String Spalte) {

  Spalte_Masseinheit = Spalte;
  if (Spalte == "Ti") {Spalte_Masseinheit=Spalte+"/s";};
  if (Spalte =="A0" || Spalte =="A1" || Spalte =="A2"|| Spalte =="A3") {Spalte_Masseinheit = Spalte+"/V";};
  if (Spalte =="Rs" || Spalte =="Rm" || Spalte =="Rp") {Spalte_Masseinheit = Spalte+"/dBm";};
  if (Spalte =="Dc") {Spalte_Masseinheit = Spalte+"/%";};
  if (Spalte !="xx") {Serial.print(Spalte_Masseinheit); Serial.print("; ");};  
  Serial.flush();
}

// ********** Ende Funktion Ausgabe Überschrift ***************************************************************************************

// *********** Funktion: Parameter Ausgabe ****************************************************

void ParameterAusg() {
  Serial.flush();
  Serial.println(F(" "));
   
  Serial.print(F("   ")); Serial.print(F("Tn=")); Serial.print(Interval); Serial.print(F("; ")); 
                       Serial.print(F("No=")); Serial.print(Number);Serial.print(F("; ")); 
                       Serial.print(F("Av=")); Serial.print(AverageInt);Serial.print(F("; ")); 
                       Serial.print(F("Pe=")); Serial.print(Peakhold);Serial.print(F("; "));  
                       Serial.print(F("Dl=")); Serial.print(DutyLevel);Serial.println(F("; "));  
                       
                               
  Serial.print(F("   ")); Serial.print(F("C0=")); Serial.print(Col_0); Serial.print(F("; ")); 
                       Serial.print(F("C1=")); Serial.print(Col_1);Serial.print(F("; ")); 
                       Serial.print(F("C2=")); Serial.print(Col_2);Serial.print(F("; "));
                       Serial.print(F("C3=")); Serial.print(Col_3); Serial.print(F("; ")); 
                       Serial.print(F("C4=")); Serial.print(Col_4);Serial.print(F("; ")); 
                       Serial.print(F("C5=")); Serial.print(Col_5);Serial.print(F("; "));  
                       Serial.print(F("C6=")); Serial.print(Col_6);Serial.print(F("; ")); 
                       Serial.print(F("C7=")); Serial.print(Col_7);Serial.print(F("; "));                          
                       Serial.print(F("C8=")); Serial.print(Col_8);Serial.println(F("; "));  
                                 
  
  Serial.print(F("   ")); Serial.print(F("Br=")); Serial.print(Brightness);Serial.println(F("; ")); 
  Serial.println("");  
    Serial.flush();                    
};

// ******** Ende Funktion Parameter Ausgabe ******************************************************************************

// ****** Funktion Parser für die Controls, Übergabe von Control_1 für die Parameter *************************************************

String parseControl(String Control) {

         Serial.flush();
        com = gesamtInput_S;
        com1 = com.substring(com.indexOf(Control));    
       // Serial.print("com1 = "); Serial.println(com1);   
        Control_1 = com1.substring(com1.indexOf(Control)+3, com1.indexOf(";")); 
    //    com1 = ""; com = "";      
        return Control_1;
         Serial.flush();
}


// *********** Unterprogramm: Parsing und Umwandlung in Integer Variablen  *************************************************************

void parseCommand(String com)
 {

// ********** Parsing der Time Controls **********************************************************************************
// --> neu, funktioniert gut, reduziert dynamischen Speicherbedarf um 5%

  parseControl("Tn");
  Interval_S = String(Control_1);
  
  parseControl("No");
  Number_S = String(Control_1);
  
  parseControl("Av");
  AverageInt_S = String(Control_1);
  
  parseControl("Pe");
  Peakhold_S = String(Control_1);
  
  parseControl("Dl");
  DutyLevel_S = String(Control_1);
 
// ********** Parsing der Columns Controls *******************************************************************************
  
  parseControl("C0");
  Col_0_S = String(Control_1);
  
  parseControl("C1");
  Col_1_S = String(Control_1);
  
  parseControl("C2");
  Col_2_S = String(Control_1);
  
  parseControl("C3");
  Col_3_S = String(Control_1);
  
  parseControl("C4");
  Col_4_S = String(Control_1);
  
  parseControl("C5");
  Col_5_S = String(Control_1);
  
  parseControl("C6");
  Col_6_S = String(Control_1);
  
  parseControl("C7");
  Col_7_S = String(Control_1);
  
  parseControl("C8");
  Col_8_S = String(Control_1);

// ********** Parsing der Gauge Controls *********************************************************************************

  parseControl("Br");
  Brightness_S = String(Control_1);
  
   Serial.flush();
   
// *********** Prüfung, ob gültige Werte im Input ************************************************************************

  if (Interval_S !="")   {Interval = Interval_S.toInt();};  
  if (Number_S !="")     {Number = Number_S.toInt();};
  if (Peakhold_S !="")   {Peakhold = Peakhold_S.toFloat();};
  if (AverageInt_S !="") {AverageInt = AverageInt_S.toFloat();};
  if (DutyLevel_S !="")  {DutyLevel = DutyLevel_S.toFloat();};
  
  if (Col_0_S !="")      {Col_0 = Col_0_S;};
  if (Col_1_S !="")      {Col_1 = Col_1_S;};
  if (Col_2_S !="")      {Col_2 = Col_2_S;};
  if (Col_3_S !="")      {Col_3 = Col_3_S;};
  if (Col_4_S !="")      {Col_4 = Col_4_S;};
  if (Col_5_S !="")      {Col_5 = Col_5_S;};
  if (Col_6_S !="")      {Col_6 = Col_6_S;}; 
  if (Col_7_S !="")      {Col_7 = Col_7_S;}; 
  if (Col_8_S !="")      {Col_8 = Col_8_S;};  
  
  if (Brightness_S !="") {Brightness = Brightness_S.toInt();};

// ************* Kurze Steuerbefehle *************************************************************************************
// ****** s = stop, r = run, t = telemetry, o = oscilloscope; (evtl später m = minute_log, h = hour_log, d = day_log)
  
       com = gesamtInput_S;
       // com1 = com.substring(com.indexOf("Br="));    
       // Serial.print("com1 = "); Serial.println(com1);   
       // Brightness_S = com1.substring(com1.indexOf("Br=")+3, com1.indexOf(";")); 
       if (com =="s") {shortInput =""; shortInput = "stop";};
       if (com =="r") {shortInput =""; shortInput = "run";};
       if (com =="t") {shortInput =""; shortInput = "telemetry";};
       if (com =="o") {shortInput =""; shortInput = "Rs_oscilloscope";};
       if (com =="a") {shortInput =""; shortInput = "Ax_oscilloscope";};
       if (com =="h" && counter<0.1) {Help();}
   //     Serial.print("short input = "); Serial.println(shortInput);  
     //   com1 = ""; com = "";
 
  };
  
// ************ Ende Unterprogramm Parser ********************************************************************************

// ************* Funktion Spaltenausgabe mit Übergabe der Spalte *********************************************************

void SpaltenAusgabe(String Spalte) {

// ---> neu, funktioniert gut, reduziert dynamischen Speicherbedarf um 10%

    if (Spalte == "Ti") {Serial.print(AusgabeZeit_S); Serial.print("; ");};
    if (Spalte == "A0") {Serial.print(A0_F_S); Serial.print("; ");};
    if (Spalte == "A1") {Serial.print(A1_F_S); Serial.print("; ");};
    if (Spalte == "A2") {Serial.print(A2_F_S); Serial.print("; ");};
      if (Spalte == "A3") {Serial.print(A3_F_S); Serial.print("; ");};  
    if (Spalte == "Rs") {Serial.print(Rs_F_S); Serial.print("; ");};
    if (Spalte == "Rm") {Serial.print(Rm_F_S); Serial.print("; ");};
    if (Spalte == "Rp") {Serial.print(Rp_F_S); Serial.print("; ");};
    if (Spalte == "No") {Serial.print(k);   Serial.print("; ");};
    if (Spalte == "Dc") {Serial.print(Dc_F_S);   Serial.print("; ");};
   Serial.flush();
}

// ************* Unterprogramm Spaltensteuerung der Ausgabe **************************************************************

void Ausgabe()
{

// *******  Umwandlung in String und Austausch Punkt gegen Komma *************************

  AusgabeZeit_S = String (0.001*AusgabeZeit);
  AusgabeZeit_S.replace(".",",");

  A0_F_S = String (A0_F);
  A0_F_S.replace(".",",");

  A1_F_S = String (A1_F);
  A1_F_S.replace(".",",");

  A2_F_S = String (A2_F);
  A2_F_S.replace(".",",");

   A3_F_S = String (A3_F);
   A3_F_S.replace(".",",");
 
  Rs_F_S = String (Rs_F);
  Rs_F_S.replace(".",",");

  Rm_F_S = String (Rm_F);
  Rm_F_S.replace(".",",");

  Rp_F_S = String (Rp_F);
  Rp_F_S.replace(".",",");

  Dc_F_S = String (Dc_F);
  Dc_F_S.replace(".",",");


// **** Spalten und Ausgabe ****************

// ---> neu

    SpaltenAusgabe(Col_0);
    SpaltenAusgabe(Col_1);
    SpaltenAusgabe(Col_2);
    SpaltenAusgabe(Col_3);
    SpaltenAusgabe(Col_4);
    SpaltenAusgabe(Col_5);
    SpaltenAusgabe(Col_6);
    SpaltenAusgabe(Col_7);
    SpaltenAusgabe(Col_8);
    Serial.flush();
    Serial.println();
};

// ************ Ende Unterprogramm Spaltensteuerung **************************************************


  
