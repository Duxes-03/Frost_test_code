/*

Project_name        : Frost_smart_water_bottle
Author              : Roushan
Project_description : 
Version             : V1.0
Change_history      : 
V1.0                : Initial File 

   Frost smart water bottle EOL test software program 

   Reads an analog input on pins , converts it to voltage and prints the result to serial monitor. 
   based on voltage, checks the voltage condition, 
   glow the LED by buzzing the buzzer and prints the corresponding warnings
   
  |LED    |  WARNINGS  |
   LED1    drink water 
   LED2    Battery low 
   LED3    Clean bottle
   LED4    Empty bottle
*/
const int led_1 = 9,led_2 = 10, led_3 = 11, led_4 = 12; 
unsigned int sensorvalue_1,sensorvalue_2,sensorvalue_3,sensorvalue_4;
float voltage_gyro, voltage_ldr, voltage_battery, voltage_pressure;

void Func_pot_1();             // Function protocol 
void Func_pot_2();
void Func_pot_3();
void Func_pot_4();
void Func_Buzzer();
void Func_check_gyro();
void Func_check_ldr();
void Func_check_battery();
void Func_check_pressure();

void setup() {
    // initialize serial communication at 9600 bits per second:
Serial.begin(9600);  
  // buzzer pin as output                        
pinMode(2, OUTPUT);                      // Buzzer pin mode 

}

void loop() {
  
    Func_check_gyro();                           // function call 
    Func_check_ldr();
    Func_check_battery();
    Func_check_pressure();
    Func_pot_1();
    Func_pot_2();
    Func_pot_3();
    Func_pot_4();
     
}
void Func_check_gyro()               // checking voltage for gyro sensor 
{    // read the input on analog pin 0:
     sensorvalue_1 = analogRead(A0);
     // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
     voltage_gyro = sensorvalue_1 * (5.0 / 1023.0);
     // print out the value you read:
     Serial.println("Voltage for GYRO = ");
     Serial.println(voltage_gyro);
}

void Func_check_ldr()                 // checking voltage for ldr sensor 
{
     sensorvalue_2 = analogRead(A1);
     voltage_ldr = sensorvalue_2 * (5.0 / 1023.0);
     Serial.println("Voltage for LDR = ");
     Serial.println(voltage_ldr);
}
void Func_check_battery()                     // checking voltage for battery 
{
     sensorvalue_3 = analogRead(A2);
     voltage_battery = sensorvalue_3 * (5.0 / 1023.0);
     Serial.println("Voltage of battery = ");
     Serial.println(voltage_battery);
}
void Func_check_pressure()                   // checking voltage for pressure sensor 
{
     sensorvalue_4 = analogRead(A4);
     voltage_pressure = sensorvalue_4 * (5.0 / 1023.0);
     Serial.println("Voltage for pressure sensor  = ");
     Serial.println(voltage_pressure);
}
void Func_Buzzer()                      // buzzer function defination 
{
  // Buzzer beeps 
  digitalWrite(2, HIGH);
  delay(1000);
  digitalWrite(2, LOW);
  delay(1000);
}

void Func_pot_1()                       // function gyro (POT1) defination
{
  if(voltage_gyro > 0 && voltage_gyro < 1.25) 
  {
    // led_1 to ON 
    digitalWrite(led_1, HIGH);
    delay(500);
    digitalWrite(led_1, LOW);
    delay(500);
    Func_Buzzer();
    Serial.println("Warning : Please drink water\n");
  }
    if(voltage_gyro > 1.25 && voltage_gyro < 2.5)
  {
    // led_2 to ON
    digitalWrite(led_2, HIGH);
    delay(500);
    digitalWrite(led_2, LOW);
    delay(500);
    Func_Buzzer();
    Serial.println("Warning : Battery is low \n");
  }
      if(voltage_gyro > 2.5 && voltage_gyro < 3.75)
  {
    // led_3 to ON
    digitalWrite(led_3, HIGH);
    delay(500);
    digitalWrite(led_3, LOW);
    delay(500);
    Func_Buzzer();
    Serial.println("Warning : Clean bottle \n");
  }
        if(voltage_gyro > 3.75 && voltage_gyro < 5)
  {
    // led_4 to ON
    digitalWrite(led_4, HIGH);
    delay(500);
    digitalWrite(led_4, LOW);
    delay(500);
    Func_Buzzer();
    Serial.println("Warning : Bottle is empty \n");
  }
}
void Func_pot_2()                              // function ldr (POT2) defination
{
  if(voltage_ldr > 0 && voltage_ldr < 1.25)
  {
    digitalWrite(led_1, HIGH);
    delay(500);
    digitalWrite(led_1, LOW);
    delay(500);
    Func_Buzzer();
    Serial.println("Warning : Please drink water\n");
  }
    if(voltage_ldr > 1.25 && voltage_ldr < 2.5)
  {
    digitalWrite(led_2, HIGH);
    delay(500);
    digitalWrite(led_2, LOW);
    delay(500);
    Func_Buzzer();
    Serial.println("Warning : Battery is low \n");
  }
      if(voltage_ldr > 2.5 && voltage_ldr < 3.75)
  {
    digitalWrite(led_3, HIGH);
    delay(500);
    digitalWrite(led_3, LOW);
    delay(500);
    Func_Buzzer();
    Serial.println("Warning : Clean bottle \n");
  }
        if(voltage_ldr > 3.75 && voltage_ldr < 5)
  {
    digitalWrite(led_4, HIGH);
    delay(500);
    digitalWrite(led_4, LOW);
    delay(500);
    Func_Buzzer();
    Serial.println("Warning : Bottle is empty \n");
  }
}
void Func_pot_3()                              // function battery (POT3) defination
{
  if(voltage_battery > 0 && voltage_battery < 1.25)
  {
    digitalWrite(led_1, HIGH);
    delay(500);
    digitalWrite(led_1, LOW);
    delay(500);
    Func_Buzzer();
    Serial.println("Warning : Please drink water\n");
  }
    if(voltage_battery > 1.25 && voltage_battery < 2.5)
  {
     digitalWrite(led_2, HIGH);
    delay(500);
    digitalWrite(led_2, LOW);
    delay(500);
    Func_Buzzer();
    Serial.println("Warning : Battery is low \n");
  }
      if(voltage_battery > 2.5 && voltage_battery < 3.75)
  {
    digitalWrite(led_3, HIGH);
    delay(500);
    digitalWrite(led_3, LOW);
    delay(500);
    Func_Buzzer();
    Serial.println("Warning : Clean bottle \n");
  }
        if(voltage_battery > 3.75 && voltage_battery < 5)
  {
    digitalWrite(led_4, HIGH);
    delay(500);
    digitalWrite(led_4, LOW);
    delay(500);
    Func_Buzzer();
    Serial.println("Warning : Bottle is empty \n");
  }
}
void Func_pot_4()                                   // function pressure sensor (POT4) defination
{
  if(voltage_pressure > 0 && voltage_pressure < 1.25)
  {
    digitalWrite(led_1, HIGH);
    delay(500);
    digitalWrite(led_1, LOW);
    delay(500);
    Func_Buzzer();
    Serial.println("Warning : Please drink water\n");
  }
    if(voltage_pressure > 1.25 && voltage_pressure < 2.5)
  {
    digitalWrite(led_2, HIGH);
    delay(500);
    digitalWrite(led_2, LOW);
    delay(500);
    Func_Buzzer();
    Serial.println("Warning : Battery is low \n");
  }
      if(voltage_pressure > 2.5 && voltage_pressure < 3.75)
  {
    digitalWrite(led_3, HIGH);
    delay(500);
    digitalWrite(led_3, LOW);
    delay(500);
    Func_Buzzer();
    Serial.println("Warning : Clean bottle \n");
  }
        if(voltage_pressure > 3.75 && voltage_pressure < 5)
  {
    digitalWrite(led_4, HIGH);
    delay(500);
    digitalWrite(led_4, LOW);
    delay(500);
    Func_Buzzer();
    Serial.println("Warning : Bottle is empty \n");
  }
}