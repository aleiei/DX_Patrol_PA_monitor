#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C MyLCD(0x27, 20, 4); // Creates I2C LCD Object With (Address=0x27, Cols=20, Rows=4)

float temp_voltage = 0.0;
float swr_voltage = 0.0;
float pwr_voltage = 0.0;


void setup()
{
   MyLCD.init();
   MyLCD.backlight();
   MyLCD.setCursor(0, 0);
   MyLCD.print("PA Monitor by IU0PXK");
}
void loop()
{

//Conversion formula for voltage
   
   long temp = analogRead(A0);
   temp_voltage = ((temp * (500000/1023L)) / 100); 
   temp_voltage = ((temp_voltage - 464)/6.25);


   long swr = analogRead(A1);
   swr_voltage = ((swr *(500000/1023L)) / 100)-1288;
   
      
   long pwr = analogRead(A2);
   pwr_voltage = ((pwr *(500000/1023L)) / 100)-298;
   
   swr_voltage = (pwr_voltage + swr_voltage ) / (pwr_voltage - swr_voltage);
   pwr_voltage = ((pwr_voltage + 0.5) * (pwr_voltage + 0.5))/1000000;

     
    MyLCD.setCursor(0, 1);
    MyLCD.print("TEMP C= ");
    MyLCD.print(temp_voltage);

    MyLCD.setCursor(0, 2);
  {  if(swr_voltage < 0)
   {
    MyLCD.print("SWR   = 0.00");
        }
        else
        {
    MyLCD.print("SWR   = ");
    MyLCD.print(swr_voltage);
    }
    }
    MyLCD.setCursor(0, 3);
     {  if(pwr_voltage < 1)
   {
    MyLCD.print("PWR   = 0.000");
        }
        else
        {
    MyLCD.print("PWR W = ");
    MyLCD.print(pwr_voltage);
     }
     }
    delay(300);
}
