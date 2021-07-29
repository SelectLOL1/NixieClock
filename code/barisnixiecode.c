/*
Generated Main Source File

  www.studentcompanion.coza

  File Name: main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
       Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.77
        Device            :  PIC18F45K22
        Driver Version    :  2.00
MPLAB X IDE: v5.25
XC8: v2.05
MCC: v3.85.1

 Creator: Baris Catan, BCA 
 Data: Today
 Description: Nixie_Clock Firmware 1.1, never trust version 1.0
 */


#include "mcc_generated_files/mcc.h" //includes
#include "xc.h"
#include "pic18f45k22.h"
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#define _XTAL_FREQ 40000000
#define SHIFT_CLOCK LATCbits.LATC6 //Shift_Reg pins für Nixies
#define SHIFT_DATA LATCbits.LATC5
#define LATCH_NIXIE LATCbits.LATC7

#define SHIFT_CLOCK_LED LATDbits.LATD3 //Shift_Reg pins für LEDs
#define SHIFT_DATA_LED LATDbits.LATD2
#define LATCH_LED LATDbits.LATD4

#define PULS_TIME_NIXIE 1  //Dauer eines HIGH/LOW Pulses in us für shiftreg
#define PULS_TIME_LED 1  //Dauer eines HIGH/LOW Pulses in us shiftreg

unsigned long int longbyte = 0; //Hilfsvariabel für nixie_shifter
unsigned long int longbyte_led = 0; //Hilfsvariabel für led_shifter

unsigned long int nixie_zahl = 0; //variabel die auf Nixies angezeigt wird

unsigned char Pos1[6]; //Hilfsvariabel für nixie_shifter

char string[6]; //Hilfsvariabel für nixie_shifter

unsigned char posi1 = 0; //Hilfsvariabel für nixie_shifter
unsigned char posi2 = 0; //Hilfsvariabel für nixie_shifter
unsigned char posi3 = 0; //Hilfsvariabel für nixie_shifter
unsigned char posi4 = 0; //Hilfsvariabel für nixie_shifter
unsigned char posi5 = 0; //Hilfsvariabel für nixie_shifter
unsigned char posi6 = 0; //Hilfsvariabel für nixie_shifter

void interpreter_24bit(); //Hardwarefehler werden hier behoben      
void nixie_shifter_24bit(); //Shift_Reg für Nixies ansprechen   
void led_shifter_24bit(); //Shift_Reg für LEDs ansprechen  
void rtc_timereadout(); //RTC Zeit auslesen
void StateTemperature(void); //TEMP_SENS auslesen
void rtc_timesetting(); //RTC Zeit einstellen
void hum_sens(); //HUM_SENS auslesen, jedoch no ACK!

uint16_t I2C2_Read2ByteRegister(i2c2_address_t address, uint8_t reg); //I2C 2Bytes auslesen
void I2C2_Write1ByteRegister(i2c2_address_t address, uint8_t reg, uint8_t data); //I2C 1Byte schreiben
uint8_t I2C2_Read1ByteRegister(i2c2_address_t address, uint8_t reg); //I2C 1Byte auslesen

uint16_t returnValue; //Hilfsvariabel 
unsigned int helper1; //Hilfsvariabel 
unsigned int helper2; //Hilfsvariabel 
uint16_t setting_counter = 0; //Hilfsvariabel 
uint32_t sekunden_set = 0; //Hilfsvariabel 
uint32_t minuten_set = 0; //Hilfsvariabel 
uint32_t stunden_set = 0; //Hilfsvariabel 
uint8_t selector = 0; //Hilfsvariabel 

uint8_t TMP117_cmd = 0x00; //TEMP_SENS Speicheradresse für temp 2Byte gross
uint8_t HUM_SENS_cmd = 0xE3; //HUM_SENS Speicheradresse für feuchtig. 2Byte gross
uint8_t RTC_sec_cmd = 0x00; //RTC sekunden Speicheradresse 1Byte gross
uint8_t RTC_min_cmd = 0x01; //RTC minuten Speicheradresse 1Byte gross
uint8_t RTC_hour_cmd = 0x02; //RTC stunden Speicheradresse 1Byte gross
uint8_t RTC_wkday_cmd = 0x03; //RTC wochentag Speicheradresse 1Byte gross, wird nicht gebrauch
uint8_t RTC_date_cmd = 0x04; //RTC datum Speicheradresse 1Byte gross, wird nicht gebrauch
uint8_t RTC_mth_cmd = 0x05; //RTC monat Speicheradresse 1Byte gross, wird nicht gebrauch
uint8_t RTC_year_cmd = 0x06; //RTC jahr Speicheradresse 1Byte gross, wird nicht gebrauch
uint8_t RTC_battery_cmd = 0x03; //RTC batterie_backup aktivieren Speicheradresse 1Byte gross

i2c2_operations_t I2C_status = I2C2_STOP; //Ist so weil ist so
uint16_t I2C_Address_TMP117 = 0x48; // Slave Adresse TEMP_SENS
uint16_t I2C_Address_RTC = 0x6F; // Slave Adresse RTC
uint16_t I2C_Address_HUM_SENS = 0x40; // Slave Adresse HUM_SENS, no ACK
uint8_t rtc_battery_init = 0b00001000;
uint8_t switch_wert = 1; //Switch-Case werte, sollte am Anfang 1 sein!
unsigned char timekeeper = 0; //Hilfsvariabel für RTC

/*
   Main application
 */
void main(void) {
    // Initialize the device
    SYSTEM_Initialize();
    // If using interrupts in PIC18 High/Low Priority Mode you need to enable the Global High and Low Interrupts
    // If using interrupts in PIC Mid-Range Compatibility Mode you need to enable the Global and Peripheral Interrupts
    // Use the following macros to:

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();
    while (1) {
        if ((TASTER_LINKS_GetValue() == 1) && switch_wert != 4) { //Uhrzeit beobachten oder nach langem drücken Uhrzeit einstellen, solang Uhrzeit eingestellt wird, wird der modus nicht gewechselt
            switch_wert = 1;
            setting_counter++;
            if (setting_counter >= 60) {
                switch_wert = 4;
            }
        }
        if ((TASTER_RECHTS_GetValue() == 1) && switch_wert != 4) //Feuchtigkeit beobachten oder nach langem drücken Uhrzeit einstellen, solang Uhrzeit eingestellt wird, wird der modus nicht gewechselt
            switch_wert = 3;
        if ((TASTER_MITTE_GetValue() == 1) && switch_wert != 4) //Temperatur beobachten oder nach langem drücken Uhrzeit einstellen, solang Uhrzeit eingestellt wird, wird der modus nicht gewechselt
            switch_wert = 2;

        switch (switch_wert) {
            case 1: //LEDs auschalten und Zeit anzeigen und decimalpunkt ausschalten
                longbyte_led = 0;
                DP_MOSFET_SetLow();
                rtc_timereadout();
                break;
            case 2: //LEDs auschalten und Temperatur anzeigen
                longbyte_led = 0;
                StateTemperature();
                break;
            case 3://Feuchtigkeit anzeigen und decimalpunkt ausschalten
                DP_MOSFET_SetLow();
                hum_sens();
                break;
            case 4://LEDs auschalten und Zeiteinstellungen anzeigen
                DP_MOSFET_SetLow();
                rtc_timesetting();
                break;

        }
        for (unsigned char x = 0; x < 6; x++) { //char Array mit 0 füllen (leeren) für nixie Anzeige hilfsvariabel
            string[x] = 0;
        }
        sprintf(string, "%lu", nixie_zahl); //Hilfsvariabel nixie_zahl einzelne Chars in jede einzelne stringzeile füllen , %lu -> long unsigned
        interpreter_24bit(); //Hardwarefehler werden hier behoben
        nixie_shifter_24bit(); //Nixie Shift_Reg hier ansprechen
        led_shifter_24bit(); //LED Shift_Reg hier ansprechen
        __delay_ms(5);
    }
}

void StateTemperature(void) { //TEMP_SENS auslesen
    DP_MOSFET_SetHigh(); //Decimalpunkt einstellen
    // i2c master write transaction with the supplied parameters
    I2C2_Read2ByteRegister(I2C_Address_TMP117, TMP117_cmd); //I2C auslesen, dann parameter so abändern dasses für die Applikation sinn macht.
    helper1 = nixie_zahl;
    helper2 = nixie_zahl;
    helper1 = ((helper1 >> 8) &0b0000000011111111);
    helper2 = ((helper2 << 8) &0b1111111100000000);
    nixie_zahl = (helper1 + helper2);
    nixie_zahl = ((nixie_zahl * 78125) / 1000); //Tempsens formel -> 16 bit sensor
    __delay_ms(50); //delay für tempsens
}

void led_shifter_24bit() { //Shift_Reg für LEDs ansprechen  
    unsigned long int counter = longbyte_led;
    bool bb = 0;
    LATCH_LED = 1;
    for (unsigned char i = 0; i < 24; i++) { //Bit für bit wird rausgeschoben, dabei geht clock immer 1 mal HIGH, dann LOW und data macht was es muss.
        bb = counter & 0x01;
        SHIFT_DATA_LED = bb;
        SHIFT_CLOCK_LED = 1;
        __delay_ms(PULS_TIME_LED);
        SHIFT_CLOCK_LED = 0;

        counter = counter >> 1;
    }
    LATCH_LED = 0;
}

void nixie_shifter_24bit() { //Shift_Reg für Nixies ansprechen   
    unsigned long int bb = 0;
    LATCH_NIXIE = 1;
    for (unsigned char i = 0; i < 24; i++) { //Bit für bit wird rausgeschoben, dabei geht clock immer 1 mal HIGH, dann LOW und data macht was es muss.
        bb = longbyte & 1;
        SHIFT_DATA = bb;
        SHIFT_CLOCK = 1;
        __delay_us(PULS_TIME_NIXIE);
        SHIFT_CLOCK = 0;
        longbyte = longbyte >> 1;
    }
    LATCH_NIXIE = 0;
}

void interpreter_24bit() { //Durch Hardwarefehler müssen BCD-Dec Encoder anderst angschprochen werden, hier geschiet dies.
    unsigned long int shifthelp1 = 0; //Hilfsvariabel
    unsigned long int shifthelp2 = 0; //Hilfsvariabel
    char shift_counter = 0; //Hilfsvariabel

    if (nixie_zahl > 999999) //Falls Zahl nicht angezeigt werden kann, Zahl auf 0 setzen
        nixie_zahl = 0;

    unsigned char zahlen2[10] = {0b00001000, //0 Hier ist die Anschpreschsweise, die dem Input entsprechen
        0b00000101, //1
        0b00000100, //2    
        0b00000011, //3
        0b00000010, //4
        0b00000001, //5
        0b00000110, //6
        0b00000111, //7
        0b00000000, //8
        0b00001001}; //9 

    Pos1[0] = string[0];
    Pos1[1] = string[1];
    Pos1[2] = string[2];
    Pos1[3] = string[3];
    Pos1[4] = string[4];
    Pos1[5] = string[5];

    for (unsigned char i = 0; i < 6; i++) { //Aus einem char array wird eine Variabel, deren Werte so verändern worden sind, dass der Ausgang dem eigentlichen Eingang entspricht.
        switch (Pos1[i]) {
            case '0':
                if (i == 0)
                    posi1 = 0b00001000;
                if (i == 1)
                    posi2 = 0b00001000;
                if (i == 2)
                    posi3 = 0b00001000;
                if (i == 3)
                    posi4 = 0b00001000;
                if (i == 4)
                    posi5 = 0b00001000;
                if (i == 5)
                    posi6 = 0b00001000;
                break;
            case '1':
                if (i == 0)
                    posi1 = zahlen2[1];
                if (i == 1)
                    posi2 = zahlen2[1];
                if (i == 2)
                    posi3 = zahlen2[1];
                if (i == 3)
                    posi4 = zahlen2[1];
                if (i == 4)
                    posi5 = zahlen2[1];
                if (i == 5)
                    posi6 = zahlen2[1];
                break;
            case '2':
                if (i == 0)
                    posi1 = zahlen2[2];
                if (i == 1)
                    posi2 = zahlen2[2];
                if (i == 2)
                    posi3 = zahlen2[2];
                if (i == 3)
                    posi4 = zahlen2[2];
                if (i == 4)
                    posi5 = zahlen2[2];
                if (i == 5)
                    posi6 = zahlen2[2];
                break;
            case '3':
                if (i == 0)
                    posi1 = zahlen2[3];
                if (i == 1)
                    posi2 = zahlen2[3];
                if (i == 2)
                    posi3 = zahlen2[3];
                if (i == 3)
                    posi4 = zahlen2[3];
                if (i == 4)
                    posi5 = zahlen2[3];
                if (i == 5)
                    posi6 = zahlen2[3];
                break;
            case '4':
                if (i == 0)
                    posi1 = zahlen2[4];
                if (i == 1)
                    posi2 = zahlen2[4];
                if (i == 2)
                    posi3 = zahlen2[4];
                if (i == 3)
                    posi4 = zahlen2[4];
                if (i == 4)
                    posi5 = zahlen2[4];
                if (i == 5)
                    posi6 = zahlen2[4];
                break;
            case '5':
                if (i == 0)
                    posi1 = zahlen2[5];
                if (i == 1)
                    posi2 = zahlen2[5];
                if (i == 2)
                    posi3 = zahlen2[5];
                if (i == 3)
                    posi4 = zahlen2[5];
                if (i == 4)
                    posi5 = zahlen2[5];
                if (i == 5)
                    posi6 = zahlen2[5];
                break;
            case '6':
                if (i == 0)
                    posi1 = zahlen2[6];
                if (i == 1)
                    posi2 = zahlen2[6];
                if (i == 2)
                    posi3 = zahlen2[6];
                if (i == 3)
                    posi4 = zahlen2[6];
                if (i == 4)
                    posi5 = zahlen2[6];
                if (i == 5)
                    posi6 = zahlen2[6];
                break;
            case '7':
                if (i == 0)
                    posi1 = zahlen2[7];
                if (i == 1)
                    posi2 = zahlen2[7];
                if (i == 2)
                    posi3 = zahlen2[7];
                if (i == 3)
                    posi4 = zahlen2[7];
                if (i == 4)
                    posi5 = zahlen2[7];
                if (i == 5)
                    posi6 = zahlen2[7];
                break;
            case '8':
                if (i == 0)
                    posi1 = zahlen2[8];
                if (i == 1)
                    posi2 = zahlen2[8];
                if (i == 2)
                    posi3 = zahlen2[8];
                if (i == 3)
                    posi4 = zahlen2[8];
                if (i == 4)
                    posi5 = zahlen2[8];
                if (i == 5)
                    posi6 = zahlen2[8];
                break;
            case '9':
                if (i == 0)
                    posi1 = zahlen2[9];
                if (i == 1)
                    posi2 = zahlen2[9];
                if (i == 2)
                    posi3 = zahlen2[9];
                if (i == 3)
                    posi4 = zahlen2[9];
                if (i == 4)
                    posi5 = zahlen2[9];
                if (i == 5)
                    posi6 = zahlen2[9];
                break;
            case '\0':
                shift_counter = shift_counter + 4; //Bei jedem /0 wird einmal nach links geschoben
                if (i == 0)
                    posi1 = zahlen2[0];
                if (i == 1)
                    posi2 = zahlen2[0];
                if (i == 2)
                    posi3 = zahlen2[0];
                if (i == 3)
                    posi4 = zahlen2[0];
                if (i == 4)
                    posi5 = zahlen2[0];
                if (i == 5)
                    posi6 = zahlen2[0];
                break;
            default:
                if (i == 0)
                    posi1 = zahlen2[0];
                if (i == 1)
                    posi2 = zahlen2[0];
                if (i == 2)
                    posi3 = zahlen2[0];
                if (i == 3)
                    posi4 = zahlen2[0];
                if (i == 4)
                    posi5 = zahlen2[0];
                if (i == 5)
                    posi6 = zahlen2[0];
                break;
        }
    }
    //Ascii 48 -57
    longbyte = (((posi3 << 12) | (posi4 << 8)) | ((posi5 << 4) | (posi6))); //Die einzelnen Variabeln werden zu einer fusioniert
    longbyte = (longbyte & 0b00000000000000001111111111111111);
    shifthelp1 = (posi1 << 8);
    shifthelp1 = (shifthelp1 << 8);
    shifthelp1 = (shifthelp1 << 4);
    shifthelp2 = (posi2 << 8);
    shifthelp2 = (shifthelp2 << 8);
    longbyte = (longbyte | (shifthelp1 | shifthelp2));
    longbyte = longbyte >> shift_counter;

    switch (shift_counter / 4) { //Die gezählten (/0) werden durch (0) ersetzet oder besser gesagt, dass was die Nixie als(0) sieht. Die Variabel die entsteht heisst longyte.
        case 1:
            longbyte = ((longbyte & 0b00000000000011111111111111111111) | 0b00000000100000000000000000000000);
            break;
        case 2:
            longbyte = ((longbyte & 0b00000000000000001111111111111111) | 0b00000000100010000000000000000000);
            break;
        case 3:
            longbyte = ((longbyte & 0b00000000000000000000111111111111) | 0b00000000100010001000000000000000);
            break;
        case 4:
            longbyte = ((longbyte & 0b00000000000000000000000011111111) | 0b00000000100010001000100000000000);
            break;
        case 5:
            longbyte = ((longbyte & 0b00000000000000000000000000001111) | 0b00000000100010001000100010000000);
            break;
        case 6:
            longbyte = ((longbyte & 0) | 0b00000000100010001000100010001000);
            break;
    }
}

void rtc_timereadout() { //RTC Zeit auslesen
    uint32_t sekunden = 0; //Hilfsvariabel
    uint16_t sekunden_zehner = 0; //Hilfsvariabel
    uint16_t sekunden_einer = 0; //Hilfsvariabel
    uint32_t minuten = 0; //Hilfsvariabel
    uint16_t minuten_zehner = 0; //Hilfsvariabel
    uint16_t minuten_einer = 0; //Hilfsvariabel
    uint32_t stunden = 0; //Hilfsvariabel
    uint16_t stunden_zehner = 0; //Hilfsvariabel
    uint16_t stunden_einer = 0; //Hilfsvariabel
    timekeeper = 0;
    nixie_zahl = 0;

    I2C2_Read1ByteRegister(I2C_Address_RTC, RTC_sec_cmd); //Die sekunden kommen in binär an, was wiederum in eine Variabel umgerechnet werden muss, die für die Nixies sinn macht.
    sekunden = timekeeper;
    sekunden = sekunden & 0b01111111;
    sekunden_zehner = sekunden & 0b01110000;
    sekunden_zehner = (sekunden_zehner >> 4) * 10;
    sekunden_einer = sekunden & 0b00001111;
    sekunden = sekunden_zehner + sekunden_einer;
    I2C2_Read1ByteRegister(I2C_Address_RTC, RTC_min_cmd); //Die minuten kommen in binär an, was wiederum in eine Variabel umgerechnet werden muss, die für die Nixies sinn macht. 
    minuten = timekeeper;
    minuten = minuten & 0b01111111;
    minuten_zehner = minuten & 0b01110000;
    minuten_zehner = (minuten_zehner >> 4) * 10;
    minuten_einer = minuten & 0b00001111;
    minuten = minuten_zehner + minuten_einer;
    minuten = minuten * 100;
    I2C2_Read1ByteRegister(I2C_Address_RTC, RTC_hour_cmd); //Die stunden kommen in binär an, was wiederum in eine Variabel umgerechnet werden muss, die für die Nixies sinn macht. 
    stunden = timekeeper;
    stunden = stunden & 0b00111111;
    stunden_zehner = stunden & 0b00110000;
    stunden_zehner = (stunden_zehner >> 4) * 10;
    stunden_einer = stunden & 0b00001111;
    stunden = stunden_zehner + stunden_einer;
    stunden = stunden * 10000;
    nixie_zahl = stunden + minuten + sekunden; //Alles wird zusammengezählt und in eine Variabel verschachtelt.
}

void rtc_timesetting() { //RTC Zeit einstellen
    __delay_ms(5);
    if (selector == 0)
        longbyte_led = 0b00000000000000000000000000011011; //RGB Hintergrunbeleuchtung für jeweilige Zahlen
    if (selector == 1)
        longbyte_led = 0b00000000000000000000011011000000; //RGB Hintergrunbeleuchtung für jeweilige Zahlen
    if (selector == 2)
        longbyte_led = 0b00000000000000011011000000000000; //RGB Hintergrunbeleuchtung für jeweilige Zahlen
    if (TASTER_LINKS_GetValue() == 1) {//Die jeweilige Ziffer wird runtergezählt
        if (selector == 0) {
            __delay_ms(120);
            if (sekunden_set > 0)
                sekunden_set--;
        }
        if (selector == 1) {
            __delay_ms(120);
            if (minuten_set > 0)
                minuten_set--;
        }
        if (selector == 2) {
            __delay_ms(120);
            if (stunden_set > 0)
                stunden_set--;
        }
    }
    if (TASTER_RECHTS_GetValue() == 1) { //Die jeweilige Ziffer wird hochgezählt
        if (selector == 0) {
            __delay_ms(120);
            sekunden_set++;
            if (sekunden_set >= 60)
                sekunden_set = 0;
        }
        if (selector == 1) {
            __delay_ms(120);
            minuten_set++;
            if (minuten_set >= 60)
                minuten_set = 0;
        }
        if (selector == 2) {
            __delay_ms(120);
            stunden_set++;
            if (stunden_set >= 25)
                stunden_set = 0;
        }
    }
    nixie_zahl = sekunden_set + (minuten_set * 100) + (stunden_set * 10000); //Die Hilfvariabel werden in eine einzige verschachtelt
    if (TASTER_MITTE_GetValue() == 1) {//Auswahl der Ziffern und fertigstellung der Einstellungen
        selector++; //Hilfvariabel 
        __delay_ms(250);
        if (selector >= 3) { //Fals fertigeingstellt worden ist, werden die Daten zum RTC rübergeschickt
            sekunden_set = ((((sekunden_set - (sekunden_set % 10)) / 10) << 4) | (sekunden_set % 10)); //Die Daten werden so rumgerechnet, dass es für den RTC sinn macht
            sekunden_set = (sekunden_set | 0b10000000); //Quarz aktivieren
            I2C2_Write1ByteRegister(I2C_Address_RTC, RTC_sec_cmd, sekunden_set); //Quarz aktivieren am Anfang + zeit einstellen!
            __delay_ms(20);
            minuten_set = ((((minuten_set - (minuten_set % 10)) / 10) << 4) | (minuten_set % 10)); //Die Daten werden so rumgerechnet, dass es für den RTC sinn macht
            I2C2_Write1ByteRegister(I2C_Address_RTC, RTC_min_cmd, minuten_set); //Zeit einstellen
            __delay_ms(20);
            stunden_set = ((((stunden_set - (stunden_set % 10)) / 10) << 4) | (stunden_set % 10)); //Die Daten werden so rumgerechnet, dass es für den RTC sinn macht
            stunden_set = stunden_set & 0b00111111;
            I2C2_Write1ByteRegister(I2C_Address_RTC, RTC_hour_cmd, stunden_set); //Zeit einstellen
            __delay_ms(20);
            I2C2_Write1ByteRegister(I2C_Address_RTC, RTC_battery_cmd, rtc_battery_init); //Batterie_Backup aktivieren
            __delay_ms(20);
            switch_wert = 1; //Hilfsvariabel zurücksetzen
            setting_counter = 0; //Hilfsvariabel zurücksetzen
            selector = 0; //Hilfsvariabel zurücksetzen
            longbyte_led = 0; //Hilfsvariabel zurücksetzen
            sekunden_set = 0; //Hilfsvariabel zurücksetzen
            minuten_set = 0; //Hilfsvariabel zurücksetzen
            stunden_set = 0; //Hilfsvariabel zurücksetzen
        }
    }
}

void hum_sens() {
    I2C2_Read2ByteRegister(I2C_Address_HUM_SENS, HUM_SENS_cmd); //No ACK :()
    longbyte_led = 0b00000000000000100100100100100100; //Rote farbe einstellen auf LEDs
}




