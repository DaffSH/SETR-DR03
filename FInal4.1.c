#include <SoftwareSerial.h>
#include <LCD03.h>
#include <Wire.h>
#include "Time.h"
#include "TimeLib.h"
#include <EEPROM.h>
#include <avr/sleep.h>
#include <avr/power.h>  
#include <avr/wdt.h>


//MIPROGRAMA

#define SECS_PER_TIMER1_TICK (double(16e-6))
#define CMD            (byte)0x00  // MD49 command address of 0           
#define GET_SPEED1    0x21    //returns the current requested speed of motor 1
#define GET_SPEED2    0x22    //returns the current requested speed of motor 2
#define GET_ENC1      0x23    //motor 1 encoder count, 4 bytes returned high byte first (signed)
#define GET_ENC2      0x24    //motor 2 encoder count, 4 bytes returned high byte first (signed)
#define GET_ENCS      0x25    //returns 8 bytes -  encoder1 count, encoder2 count
#define SET_SPEED1    0x31    //set new speed1
#define SET_SPEED2    0x32    //set new speed2 or turn
#define SET_MOD       0x34    //set the mode
#define RESET_ENC     0x35    //zero both of the encoder counts
#define M_PI      3.14159265358979323846
#define TICSV       980
#define RADIO       0.0625
#define K 7


SoftwareSerial motors = SoftwareSerial(0x03, 0x04);    // Creates a serial port for the motors
volatile byte enc1a, enc1b, enc1c, enc1d, enc2a, enc2b, enc2c, enc2d, pin2 = 2;
volatile int32_t  en1=0, enAnt1=0, en2=0, enAnt2=0, tf1=0, tb1=0, tf2=0, tb2=0, tk1=0,tk2=0;
volatile int8_t aSpeedG1, aSpeedG2, aSpeed , aSpeed2;
volatile int temp1, temp2;
volatile uint64_t contador = 0,registro1 = 0, registro2 = 0;
const int32_t minimum = -2147483648;
const uint32_t maximum = 2147483647;
const uint16_t ticspv = 980; 
LCD03 lcd;
uint8_t muestra = 0;                                     
volatile uint8_t seconds = 00, minutes = 00, hours = 00, mierda = 0;  
volatile double vMotor1= 0, vMotorG1= 0,vMotor2= 0, vMotorG2= 0, velocidad1=0, velocidad2=0, velocidadG1, velocidadG2, diff=0, T1, T2, registro1Ant = 0.00,registro2Ant = 0.00;
boolean newTime=false, wD = false, dormido = false, updateV = false, stopped = false;
volatile time_t t;
int Month, Day, Hour, Minute, Second ,Year;
String command = "",tiempoT , velT1, velT2, htemp, mtemp, stemp;


void setup(){
  
  lcd.clear();
  
  pinMode(pin2, INPUT);
  tiempoT = String("");
  velT1 = String("");
  velT2 = String("");
  lcd.begin(16,2);
  lcd.backlight();  
  lcd.print("Starting...");  
  Wire.begin();
  Serial.begin(9600);
  Serial.println(F("Please set currect time in format: \nTdd/mm/yyyy hh:mm:ss, ex: T30/07/2018 17:15:00 \n"));
  motors.begin(9600);
  motors.write(CMD);   
  motors.write(0x34);
  motors.write(1);      //modo 1
  motors.write(CMD); 
  motors.write(0x35);
  TCCR1A = 0;       // set entire TCCR1A register to 0
  TCCR1B = 0;       // same for TCCR1B
  TCNT1  = 0;       //initialize counter value to 0
  TCCR1B |=  (1<<(WGM12));
  //TCCR1B |=  (1<<(WGM13));
  OCR1A = 3124;       // cada 50 ms
  TCCR1B |= (1<<(CS12));  //prescaler 256
  TIMSK1 |= (1<<(OCIE1A));  //por desbordamiento
  
  //velocidades maximas
  //setSpeed1(127);        //velocidad inicial 1
  //setSpeed2(-128);        //velocidad inicial 2
  
  while (timeStatus() == timeNotSet)
    {   
        if (Serial.available()){
            processSyncMessage();
        }
    }
  Serial.println(F("\nPara dormir: <duerme> . Se despertará pulsando el interruptor"));
  Serial.println(F("Para cambiar velocidad de ambos motres: <velocidad n> siendo n la velocidad en Km/h"));
  Serial.println(F("Para cambiar velocidad de un motor : <speedX n> siendo X 1 para el motor 1 y 2 para el motor 2; siendo n la velocidad en Km/h"));
  Serial.println(F("Para invertir velocidades: <back>"));  
  Serial.println(F("Para parar motores: <stop>"));
  Serial.println(F("Para resetear la fecha : <reset>"));
  Serial.println(F("Para guardar una nueva  fecha : <guardarFecha> \n"));
  
  lcd.clear();
  configureWatchdog(); 
   
}

void loop(){   
    
    wdt_reset();
    compareS();
    if(updateV){
      refreshTime();
      refreshVel();
      muestraLcd();
      updateV =  false;
 
    }


  if (newTime==true) {
    
    if (Serial.available()){
    
      processSyncMessage();
      newTime=false;
        
    }
  } else{
      wdt_reset();
      commands();
    }
  
  while(wD){ 
  
    muestraLcd();
    duerme();
    if (!wD){
      if (!stopped) resume();
	  dormido = false;
      lcd.backlight();
    }
  }
  leerEncoders();

}


ISR (TIMER1_COMPA_vect){
  contador += 3125;
  muestra ++;
  
 if ((muestra % 10) == 0){
    updateV = true;  
  }
  
  if(muestra >=20){
    update_clock();
    muestra=1;   
  }
}

void configureWatchdog() {
  cli();               // disable system interrupts during watchdog configuration
  wdt_reset();            // reset the watchdog timer
  WDTCSR |= (1<<WDCE) | (1<<WDE);   // follow unlocking procedure at the bottom of page 51 on the datasheet
  WDTCSR = 1<<WDP1 | 1<<WDP2;     // 1 seconds - Page 55 of the datasheet
  WDTCSR |= _BV(WDIE);        // Enable the WD interrupt (note no reset)
  sei();              // enable interrupts again, it's cool now
  wdt_reset();
}


ISR(WDT_vect){
  wD = true;
  update_clock();
  mierda ++;
 
}

//funcion para poner en hora
void processSyncMessage(){
  wdt_reset();  
  String aux = Serial.readStringUntil('\n');

    //char c = Serial.read();
    if (aux.substring(0,1)!="T"){
        if (newTime == true) {
          Serial.println(F("Error en el formato, vuelva a introducir el comando <guardarFecha>."));
        }else {

      Serial.println(F("Please set currect time in format: \nTdd/mm/yyyy hh:mm:ss, ex: T30/07/2018 17:15:00 \n"));
    }
    return;
    }

    wdt_reset();
    aux = aux.substring(1,aux.length());
    const char* str = aux.c_str();  
    sscanf(str, "%d/%d/%d %d:%d:%d", &Day, &Month, &Year, &Hour, &Minute, &Second);  
    setTime(Hour,Minute,Second,Day,Month,Year);                 // Sync Arduino clock to the time received on the serial port
    hours = hour();
    minutes=minute();
    seconds=second();
    EEPROM.put( 0, hours );           
    EEPROM.put( sizeof(int), minutes );                         
    EEPROM.put( 2 *sizeof(int) , seconds );         

}
void compareS(){
  
  if (vMotor1 == 0) setSpeed1(0);
  if (vMotor2 == 0) setSpeed2(0);
  
  diff = vMotor1 - velocidad1;
  if (diff >= 0.05 || diff <= -0.05) { 
  
    aSpeed = getSpeed1();
    temp1 = aSpeed + (K * diff);

    if((temp1 < 127) && (temp1 > -128)){ 
    
      aSpeed = temp1;
      setSpeed1(aSpeed); 
    }else{

      if (temp1 > 127)setSpeed1(127);
        
      else setSpeed1(-128);
    }
   
  }
  diff = vMotor2 - velocidad2 ;
  if (diff >= 0.05 || diff <= -0.05) {   
    aSpeed2 = getSpeed2();
    temp2 = aSpeed2 + (K * diff);
    
  if((temp2 < 127) && (temp2 > -128)){
    aSpeed2 = temp2;
    setSpeed2(aSpeed2);    
  }else{
    
    if (temp2 > 127) setSpeed2(127);
      
    else setSpeed2(-128);         
    } 
  }       
}

//comandos
void commands() {
   
    if(Serial.available()>0){    
      command=Serial.readStringUntil('\n');
      
      
      if(command.substring(0,6)=="speed1"){ 
      
        String numero=command.substring(6,command.length());
        vMotor1=numero.toDouble();
		stopped = false;
        return;
      }
        
      if(command.substring(0,6)=="speed2"){
        
        String numero=command.substring(6,command.length());
        vMotor2=numero.toDouble();
		stopped = false;
        return;
      }
      
      if(command.substring(0,9)=="velocidad"){
        
        String numero=command.substring(9,command.length());
        vMotor1=numero.toDouble();
        vMotor2=vMotor1;
		stopped = false;
        return;
      } 
      
      if(command == "back"){  
      
        //aSpeedG1 = getSpeed1();
        //aSpeedG2 = getSpeed2(); 
        vMotor1 = -vMotor1;
        vMotor2 = -vMotor2;
        //setSpeed1(-aSpeedG1);
        //setSpeed2(-aSpeedG2);
		stopped = false;
        return;
      }       
      
      if(command=="stop"){
        stop();
		stopped = true;
        return;
      } 
      if(command=="resume"){
      
        resume();
		stopped = false;
        return;
      } 
      
      if(command=="reset"){
        
        EEPROM.get( 0, hours);
        EEPROM.get( 0+sizeof(int), minutes);
        EEPROM.get( 0+sizeof(int)+sizeof(int), seconds);
        setTime(hours,minutes,seconds,day(),month(),year());
        wdt_reset();
        return;
      }
      
      if(command=="guardarFecha"){
      
        Serial.println(F("Please set currect time in format: \nTdd/mm/yyyy hh:mm:ss, ex: T19/07/2018 18:15:00 \n"));
        newTime=true;
        return;
      } 
      if(command=="duerme"){
		
        duerme();
      }
  
    }
}
void stop(){
    aSpeedG1 = getSpeed1();
    aSpeedG2 = getSpeed2();
	if (!dormido){
      vMotorG2 = vMotor2;
      vMotorG1 = vMotor1;
	}
    velocidadG1 = velocidad1;
    velocidadG2 = velocidad2;
    wdt_reset();
    setSpeed1(0);
    setSpeed2(0);
    vMotor1 = 0;
    vMotor2 = 0;
    
    if (!dormido) Serial.println(F("Para reanudar última velocidad tras stop: <resume> \n"));
	else Serial.println(F("Se ha dormido, para reanudar el programa, pulse el botón \n"));
    Serial.print(F("-Moto1: Actual : "));
    Serial.print(velocidadG1);
    Serial.print(F(" Km/h. "));
    Serial.print(F("Objetivo : "));
    Serial.print(vMotorG1);
    Serial.println(F(" Km/h. "));   
    Serial.print(F("-Moto2: Actual : "));
    Serial.print(velocidadG2);
    Serial.print(F(" Km/h. "));
    Serial.print(F("Objetivo : "));
    Serial.print(vMotorG2);
    Serial.println(F(" Km/h. \n"));
    wdt_reset();
}


void resume(){
  
  if ((getSpeed1() == 0) && (getSpeed2() == 0) ){
      vMotor1 = vMotorG1;
      vMotor2 = vMotorG2;
      setSpeed1(aSpeedG1);
      setSpeed2(aSpeedG2);
    }
  
}
 
void duerme(){
  if(!wD){
    dormido = true ;
    stop();
    lcd.noBacklight();  
  }  
    
  attachInterrupt(0, pin2Interrupt, LOW);
  delay(100);
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_mode();
  /* The program will continue from here. */
  /* First thing to do is disable sleep. */
  sleep_disable();
  refreshTime();
} 

  
void muestraLcd(){

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print (tiempoT);
  
  if(!dormido){
    lcd.setCursor(0,1);
    lcd.print (velT1);
    lcd.setCursor(8,1);
    lcd.print (velT2);
  }
 
    
}
void refreshTime(){
  t = now();    
  htemp="";
  mtemp="";
  stemp="";
  tiempoT = "";
  if (hours<10) htemp= htemp + "0" + hours;
  else htemp= htemp + hours;
  if (minutes<10) mtemp = mtemp + "0" + minutes;
  else mtemp= mtemp + minutes;
  if (seconds<10) stemp= stemp + "0" + seconds;
  else stemp= stemp +seconds;
  tiempoT = tiempoT + day(t)  + "/" + month(t) + "/" + (year(t) - 2000) + " " + htemp + ":" + mtemp + ":" + stemp;

}

void refreshVel(){

  velT1= "";
  velT1 = velT1 + "v1:"  + velocidad1 ;
  velT2= "";
  velT2 = velT2 + "v2:"  + velocidad2 ;
  
}
  
void update_clock(){
  
    seconds++;
    if (seconds == 60){
      seconds = 0;
      minutes++;
    }
    if(minutes==60){
    
      minutes=0;
      hours++;
    }
    if(hours>23){
    
      hours=0;
    }
}

void leerEncoders(){
  
  en1 = getEncoder1();
  cli();
  registro1 = contador + TCNT1;
  sei();
  
  en2 = getEncoder2();
  cli();
  registro2 = contador + TCNT1;
  sei();
 
  
  
  //Cálculo de la velocidad
    
  if(en1>=enAnt1){
  
    tf1 = en1 - enAnt1;
  }else{
  
    tf1 = maximum - minimum + en1 - enAnt1;
  }
  if(en2>=enAnt2){
  
    tf2 = en2 - enAnt2;
  }else{
  
    tf2 = maximum - minimum + en2 - enAnt2;
  }

  //BACKWARD
  if(en1<=enAnt1){
  
    tb1 = enAnt1 - en1;
  }else{
  
    tb1= maximum - minimum + enAnt1 - en1;
  }
  if(en2<=enAnt2){
  
    tb2 = enAnt2 - en2;
  }else{
  
    tb2 = maximum - minimum + enAnt2 - en2;
  }
  //OBTENER TICKS POR PERIODO
  if(tb1<tf1){
   
    tk1 = -tb1;
  }else{
  
    tk1 = tf1;
  }
  if(tb2<tf2){
   
    tk2 = -tb2;
  }else{
    
    tk2 = tf2;
  }
  T1 = registro1  - registro1Ant ;
  T1 *= SECS_PER_TIMER1_TICK; 
  T2 = registro2  - registro2Ant ;
  T2 *= SECS_PER_TIMER1_TICK; 

  velocidad1 = ( (tk1*2*M_PI/TICSV) / T1) * RADIO * 3.6;
  velocidad2 = ( (tk2*2*M_PI/TICSV) / T2) * RADIO * 3.6;
    
  registro1Ant = registro1; 
  registro2Ant = registro2; 
  enAnt1 = en1;
  enAnt2 = en2;
  }

    

// ########################Funciones dismponibles ##########################

void setSpeed1(int velocidad){
    
  motors.write(CMD); 
  motors.write(SET_SPEED1);
  motors.write(velocidad);
}

void setSpeed2(int velocidad){
    
  motors.write(CMD); 
  motors.write(SET_SPEED2);
  motors.write(velocidad);
}


void setMode(int mode){
  
  motors.write(CMD);  
  motors.write(SET_MOD);
  motors.write(mode);
}

int32_t getEncoder1(){
 
  motors.write(CMD);
  motors.write(GET_ENC1);
  
  while(motors.available() < 4){}

  enc1a = motors.read();
  enc1b = motors.read();
  enc1c = motors.read();
  enc1d = motors.read();

  en1 = (static_cast<uint32_t>(enc1a) << 24) + 
  (static_cast<uint32_t>(enc1b)<<16) + 
  (static_cast<uint32_t>(enc1c)<<8) + 
  static_cast<uint32_t>(enc1d);
  return en1;
}

int32_t getEncoder2(){
  

  motors.write(CMD);
  motors.write(GET_ENC2);

  while(motors.available() < 4){}
        
  enc2a = motors.read();
  enc2b = motors.read();
  enc2c = motors.read();
  enc2d = motors.read();
  
  en2 = (static_cast<uint32_t>(enc2a) << 24) + 
  (static_cast<uint32_t>(enc2b)<<16) + 
  (static_cast<uint32_t>(enc2c)<<8) + 
  static_cast<uint32_t>(enc2d);
  
  //en2=(enc2a << 24) | (enc2b <<16 ) | (enc2c <<8) | enc2d;
 
  return en2;
}

int getSpeed1(){
  
  int speed=0;

  motors.write(CMD);
  motors.write(GET_SPEED1);
  while(motors.available() < 1){}
    speed = motors.read();
  return speed; 
}

int getSpeed2(){
  
  int speed=0;
  motors.write(CMD);
  motors.write(GET_SPEED2);
  while(motors.available() < 1){} 
    speed = motors.read();
  return speed; 
}


void pin2Interrupt(void){
  wD = false;
  detachInterrupt(0);
  
}
void guardarFecha(){
  newTime=true;
}