/* This code uses WDT to reset the code whenever it get stuck.
 *  The timing is very important in this scenario when ever the 
 *  cod eget stuck the timing infromation is svaed in the   eeprom.
 *  All other codes are the same.However the initial time should be
 *  stored in the EEPROM before the code is running. */
 
 
#include <EEPROM.h>
#include <Time.h>
#include <avr/wdt.h>

double addr,DAY,HOUR,MIN,SEC;
float eeAddress = 0;

double YEAR = 2017;
double MON = 6;



void setup(){
EEPROM.get(eeAddress, DAY);
EEPROM.get(eeAddress+sizeof(double), HOUR);
EEPROM.get(eeAddress+2*sizeof(double), MIN);
EEPROM.get(eeAddress+3*sizeof(double), SEC);
setTime(HOUR,MIN,SEC,DAY,MON,YEAR);
watchdogSetup();
Serial.begin(9600);
}

void watchdogSetup(void)
{
cli();
wdt_reset(); // reset the WDT timer
// Enter Watchdog Configuration mode:
WDTCSR |= (1<<WDCE) | (1<<WDE);
// Set Watchdog settings:
WDTCSR = (1<<WDIE) | (1<<WDE) | (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);
sei();
}



void loop(){
wdt_reset();

DAY =  day();
HOUR = hour();
MIN = minute();
SEC = second();

Serial.println(DAY);
Serial.println(HOUR);
Serial.println(MIN);
Serial.println(SEC);

while(1);
  
// Write new values to the EEPROM. 
// write only when system hangs.

}


void EEPROM_writeDouble(int ee, double value)
{
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
       EEPROM.write(ee++, *p++);
}




//Interrupt Sub routine 
ISR(WDT_vect) // Watchdog timer interrupt.
{
Serial.println("Hi World");
addr = 0;
EEPROM_writeDouble(addr,day());
addr = addr + sizeof(double);
EEPROM_writeDouble(addr,hour());
addr = addr + sizeof(double);
EEPROM_writeDouble(addr,minute());
addr = addr + sizeof(double);
EEPROM_writeDouble(addr,second());
addr = addr + sizeof(double);
}
