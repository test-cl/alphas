//#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

// Set the LCD address to 0x20 for 8574 and 0x38 for 8574A
LiquidCrystal_I2C lcd(0x38, 20, 4);
#define Hematocrit    1
#define Immufuge      2
#define SerofugeLow   3
#define SerofugeHigh  4
#define ClinicalLow   5
#define ClinicalHigh  6
#define CardgelLow    7
#define Alphas        8
int type_product;

#define _1          0
#define _2          1
#define _F          2
#define _St         3
#define _Sp         7
#define _manual     8
#define _noSelect   9

#define _Setting    1
#define _Memory     2
#define _Manua      3
#define _Edit       4
#define _Run        5
#define _Stop       6
#define _Rebot      7

#define noPress     0
#define _ledPin     A2
#define _beepPin    8
#define _enPin      1
#define _doorPin    A3
#define _pwmPin     9
#define _fgPin      2
#define _testPin    3
#define _tempPin    A7

#define _minute     1
#define _second     0
#define _rpm        0
#define _rcf        1

#define memSetted   0xAA55

#define D1        (analogRead(A6)/1000)
#define D2        digitalRead(5)
#define D3        digitalRead(6)
#define D4        digitalRead(7)
#define LED_RUN   digitalWrite(_ledPin,LOW)
#define LED_STOP  digitalWrite(_ledPin,HIGH)
#define soundOn   digitalWrite(_beepPin,LOW)
#define soundOff  digitalWrite(_beepPin,HIGH)
#define beep      soundOn;delay(100);soundOff
#define _tempV    (analogRead(_tempPin)*5.0f*0.976f)
#define TEMP      map(_tempV,2100,997,0,100)

#define samplingTime  500

const char sw_pin[8][2]={{A1,10},{A1,11},{A1,12},{A1,13},
                  {A0,10},{A0,11},{A0,12},{A0,13},};
const int sw_val[8]={1,2,4,8,16,32,64,128};

int memStatus,setSpeed,time;
int buff_x,buff_min;
int count;
int RPM;
int max_rpm , min_rpm,N_pole ,defaultSetSpeed,defaultTime;
int t_start = 10;
char _mode,_menu;

const char *labelSw[9]={"1 ","2 ","F ","St","  ","  ","  ","Sp"};

void setup()
{
  pinMode(_testPin,OUTPUT);
  pinMode(_ledPin,OUTPUT);
  pinMode(_beepPin,OUTPUT);
  pinMode(_enPin,OUTPUT);
  pinMode(_pwmPin,OUTPUT);
  pinMode(_doorPin, INPUT_PULLUP);
  digitalWrite(_enPin,HIGH);
  digitalWrite(5,HIGH);digitalWrite(6,HIGH);digitalWrite(7,HIGH);   //init D1  D2  D3

  max_rpm = 4000;
  min_rpm = 500;
  defaultSetSpeed = 1800;
  defaultTime = 5;
  t_start = 20;
  N_pole = 2;

  setSpeed = defaultSetSpeed;
  time = defaultTime;
  _mode = _noSelect;
  _menu =_Setting;
  
  TCCR2A = (1 << WGM21);    // Set to CTC Mode  
  TCCR2B = (1 << CS22)|(1 << CS21)|(1 << CS20);    // set prescaler to 1024 and starts PWM
  OCR2A = 156;  //10 ms
  TIMSK2 = (1 << OCIE2A);   //Set interrupt on compare match

  EICRA |= (1 << ISC00)|(1 << ISC01);    // set INT0 to trigger on rissing
  EIMSK |= (1 << INT0);     // Turns on INT0
  sei();                      // turn on interrupts

	lcd.init();
  lcd.backlight();
  lcd.clear(); // clear display
  lcd.setCursor(7, 0);
  lcd.print("Welcome");
  lcd.setCursor(2, 1);
  lcd.print("ALPAS Centrifuge");
  lcd.setCursor(3, 2);
  lcd.print("Innovations in");
  lcd.setCursor(3, 3);
  lcd.print("Plasma Therapy");
  LED_STOP ;
  beep;
  delay(5000);
}

void loop()
{
  int buffSw;
 /* char buffChar[20];
  buffSw = getSwitch();
  sprintf(buffChar,"%d  ,  %d",buffSw,(sw_val[_1] | sw_val[_2] | sw_val[_F]));
   lcd.setCursor(0, 0);
  lcd.print(buffChar);*/
  if(_menu==_Setting){
    lcd.clear();
    lcd.setCursor(7, 0);
    lcd.print("Setting");  
    lcd.setCursor(1, 1);
    lcd.print("Please select Mode"); 
    lcd.setCursor(0, 2);
    lcd.print("Mode : "); 
    if(_mode == _noSelect){
      lcd.setCursor(7, 2);
      lcd.print("No selection");    
      //while(getSwitch()!=noPress);
      while(getSwitch()==noPress);
      buffSw = getSwitch();delay(50);
      if(buffSw == (sw_val[_1] | sw_val[_2] | sw_val[_F])) _menu = _Memory;
      else if(buffSw & sw_val[_1]){_mode = _1;getEEPROM(_mode);}
      else if(buffSw & sw_val[_2]){_mode = _2;getEEPROM(_mode);}
      else if(buffSw & sw_val[_F]){_mode = _F;getEEPROM(_mode);}
    }else{
      lcd.setCursor(7, 2);
      lcd.print(labelSw[_mode]); lcd.print("           ");   
      lcd.setCursor(0, 3);
      lcd.print("St:Start     Sp:Back");
      //while(getSwitch()!=noPress);
      while(getSwitch()==noPress);
      buffSw = getSwitch();delay(100);
      if(buffSw == (sw_val[_1] | sw_val[_2] | sw_val[_F])) _menu = _Memory;
      else if(buffSw & sw_val[_St]){
        if(memStatus != memSetted){
          beep;
          lcd.setCursor(9, 2);   lcd.print("           ");
          delay(100);
          lcd.setCursor(9, 2);   lcd.print("Err no set ");
          beep;delay(100);beep;delay(50);
          while(getSwitch()!=noPress);
        }else _menu = _Run;
      }
      else if(buffSw & sw_val[_Sp]) _mode = _noSelect;
      else if(buffSw & sw_val[_1]){_mode = _1;getEEPROM(_mode);}
      else if(buffSw & sw_val[_2]){_mode = _2;getEEPROM(_mode);}
      else if(buffSw & sw_val[_F]){_mode = _F;getEEPROM(_mode);}
    }
  }else if(_menu==_Memory){
    memory();
  }else if(_menu==_Edit){
    editProgram();
  }else if(_menu==_Manua){
    memStatus = memSetted;
    _mode = _manual;
    editProgram();
  }else if(_menu==_Run){
    runProgram();
  }
}

void runProgram(){
  unsigned long buff_millis ; 
  unsigned long nextSamplingTime = 0;
  unsigned long buffTime = time;
  int POW=0;
  int I=0,P=0,D=0,P_old=0;
  int buffSpeed = 0,a_start;
  int targetSpeed;
  unsigned long buffTimeF;
  //bool  _status = true;
  int _disDot;
  int buffMode = _mode;
  targetSpeed = setSpeed;
  
  lcd.clear();
  lcd.setCursor(2, 1);
  if(buffMode == _manual) lcd.print("Starting");
  else                    lcd.print("Waiting");
  disWait(); //แสดงจุดวิ่ง 7 จุด
  lcd.clear();
  if(buffMode == _manual) {
    lcd.setCursor(0, 0);
    lcd.print("Running: Manual Mode");
    lcd.setCursor(2, 1);
    lcd.print("Speed:       rpm");
  } else {
    lcd.setCursor(0, 0);
    lcd.print("Running: Mode  ");lcd.print(labelSw[buffMode]);
    lcd.setCursor(4, 1);
    lcd.print("Operating...");
  }
  lcd.setCursor(2, 3);
  lcd.print("DO NOT OPEN LID");

  buffTimeF = 60000;
  a_start = targetSpeed/(t_start*2);  //คูณ 2 เพราะอัตราการทำงานเป็น 500 ms
  buff_millis = millis(); 
  while(true){
    if(isPress(_Sp)) {delay(100);buffTime = 0;} //กดปุ่ม stop

    if(buffTime<=0){   //หมดเวลา
      brk(buffMode);   
      while(!isPress(_Sp)) ;
      beep;delay(100);beep;
      reboot();                      
      //_status = false;
    }
    
    if(digitalRead(_doorPin)==LOW){ //ถ้าฝาปิด
     // if(_status){
        lcd.setCursor(2, 3);
        lcd.print("DO NOT OPEN LID");soundOff;
        if(millis()>nextSamplingTime){
          if(buffMode == _manual) {
            char buffChar[4];
            sprintf(buffChar,"%4d",RPM);
            lcd.setCursor(9, 1);
            lcd.print(buffChar);
          }
          else{
            lcd.setCursor(13, 1);
            if(_disDot==0){lcd.print("   ");_disDot++;}
            else if(_disDot==1){lcd.print(".  ");_disDot++;}
            else if(_disDot==2){lcd.print(".. ");_disDot++;}
            else               {lcd.print("...");_disDot=0;}
          }
          /*char buffChar[40];
          sprintf(buffChar,"POW=%dP=%dI=%dD=%d ",POW,P,I,D);
          lcd.setCursor(0, 2);lcd.print(buffChar);*/
          digitalWrite(_enPin,LOW);
          nextSamplingTime = millis() + samplingTime;          
          LED_RUN ;
          if(buffSpeed<targetSpeed) buffSpeed+=a_start+200;  //+200 เป็นค่าเริ่มต้น
          if(buffSpeed>targetSpeed) buffSpeed = targetSpeed;

          P = buffSpeed - RPM;
          D = P-P_old;
          P_old = P;
          I=I+(P/10);        
          if(I>5600) I=5600;
          else if(I<-3600) I=-3600;          
          POW = 70+(P/120)+(I/40)+(D/250);
                    
          if(POW>255)     POW = 255;
          else if(POW<0)  POW = 0;

          analogWrite(_pwmPin,POW);    
                   
          if((millis()-buff_millis)>=buffTimeF)  //เวลาลดลง
          {
            buff_millis = millis(); 
            buffTime--;  
          }
        }
     // }
    }else{   //ฝาถูกเปิด
      LED_STOP ;
      digitalWrite(_enPin,HIGH);
      digitalWrite(_pwmPin,LOW);
      lcd.setCursor(2, 3);
      if(_disDot==0){lcd.print("DO NOT OPEN LID");/*soundOn;*/_disDot=1;}
      else          {lcd.print("               ");/*soundOff;*/_disDot=0;}
      beep;delay(100);//delay(500); 
    }
  }
}

void brk(int buffMode)
{ 
  int _disDot;
  char buffChar[4];
  digitalWrite(_pwmPin,LOW);
  if(buffMode != _manual){
    lcd.setCursor(4, 1);
    lcd.print("Stopping... ");
  }
  while(RPM>0) {
    if(buffMode == _manual) {
      sprintf(buffChar,"%4d",RPM);
      lcd.setCursor(9, 1);
      lcd.print(buffChar);
    }else{
    lcd.setCursor(12, 1);
      if(_disDot==0){lcd.print("   ");_disDot++;}
      else if(_disDot==1){lcd.print(".  ");_disDot++;}
      else if(_disDot==2){lcd.print(".. ");_disDot++;}
      else if(_disDot==3){lcd.print("...");_disDot=0;}
    }
    delay(500);     
  }
  soundOn; delay(1000); soundOff;  
  //beep;delay(100);
  if(buffMode == _manual) {
    sprintf(buffChar,"%4d",0);
    lcd.setCursor(9, 1);
    lcd.print(buffChar);
  }else{
    lcd.setCursor(4, 1);
    lcd.print("            ");
  }
  lcd.setCursor(0, 2);
  lcd.print(" Process completed  ");
  lcd.setCursor(0, 3);
  lcd.print("              Sp:END");
  LED_STOP ;
  digitalWrite(_enPin,HIGH);
}

void memory(){
  lcd.clear();
  lcd.setCursor(4, 0);
  lcd.print("Memory Mode");
  lcd.setCursor(2, 1);
  lcd.print("Waiting");
  disWait(); //แสดงจุดวิ่ง 7 จุด
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Memory Mode Setting");
  lcd.setCursor(1, 1);
  lcd.print("Please Select Mode");
  lcd.setCursor(0, 2);
  lcd.print("Mode : No selection");
  lcd.setCursor(13, 3);
  lcd.print("Sp:Back");
  while(true){
    int buffSw = getSwitch();
    if(buffSw!=noPress) {
      delay(100);
      if(!(buffSw & sw_val[_St]) && !(buffSw & sw_val[_Sp])){  //ถ้าปุ่มที่กดไม่ใช่ St และ Sp
        if(buffSw & sw_val[_1])       _mode = _1;
        else if(buffSw & sw_val[_2])  _mode = _2;
        else if(buffSw & sw_val[_F])  _mode = _F;
        lcd.setCursor(7, 2);
        lcd.print("            ");
        lcd.setCursor(7, 2);
        lcd.print(labelSw[_mode]);
        lcd.setCursor(1, 3);
        lcd.print("St:Confirm");        
      }else if(buffSw & sw_val[_St]) {
        if(_mode != _noSelect){
          _menu = _Edit;
          delay(100);while(getSwitch()!=noPress);delay(100);return;
        }
      }else if(buffSw & sw_val[_Sp]) {
        if(_mode == _noSelect){
          lcd.clear();
          lcd.setCursor(4, 0);
          lcd.print("Exit Memory Mode");
          lcd.setCursor(2, 1);
          lcd.print("Waiting");
          disWait(); //แสดงจุดวิ่ง 7 จุด
          _menu = _Setting;
          delay(100);while(getSwitch()!=noPress);delay(100);return;
        }else{
          lcd.setCursor(7, 2);
          lcd.print("No selection");
          lcd.setCursor(0, 3);
          lcd.print("             Sp:Back");
          _mode = _noSelect;
        }
      }
      delay(100);while(getSwitch()!=noPress);delay(100);
   }      
  }
}

void editProgram(){
  int buffSpeed,buffTime;
  int menu,buffDelay = 300;
  char buffChar[20];
  int buffMode = _mode;
  getEEPROM(buffMode); 
  buffSpeed = setSpeed;
  buffTime = time;
  menu = 0;
  if(buffMode == _manual){
    lcd.clear();
    lcd.setCursor(4, 0);
    lcd.print("Manual Mode");
    lcd.setCursor(2, 1);
    lcd.print("Waiting");
    disWait(); //แสดงจุดวิ่ง 7 จุด
  }
  lcd.clear();
  lcd.setCursor(0, 0);
  if(buffMode == _manual) {lcd.print("Manual Mode Setting");}
  else                    {lcd.print("Remembering: Mode "); lcd.print(labelSw[buffMode]);}
  lcd.setCursor(0, 2);
  lcd.print("1:UP          2:Down");
  lcd.setCursor(0, 3);
  lcd.print("St:Confirm   Sp:Back");
  delay(100);while(getSwitch()!=noPress);delay(100);
  while(true){
    int buffSw = getSwitch();
    if(buffSw & sw_val[_St]) {
      menu++;
      if(menu>1) {
        lcd.clear();
        lcd.setCursor(1, 0);
        if(buffMode == _manual) {lcd.print("Confirm Manual Run");}
        else                    {lcd.print("Confirm Mode: "); lcd.print(labelSw[buffMode]);}
        sprintf(buffChar,"Set Speed: %4d rpm ",buffSpeed);
        lcd.setCursor(0, 1);  
        lcd.print(buffChar);
        sprintf(buffChar,"Set Time:   %2d min ",buffTime);
        lcd.setCursor(0, 2);  
        lcd.print(buffChar);
        lcd.setCursor(0, 3);
        if(buffMode == _manual) lcd.print("St:Start     Sp:Back");
        else                    lcd.print("St:Confirm   Sp:Back");
        delay(100);while(getSwitch()!=noPress);
        do{buffSw = getSwitch();} while(!(buffSw & sw_val[_St]) && !(buffSw & sw_val[_Sp]));
        if(buffSw & sw_val[_St]) {
          if(buffMode == _manual) {
            setSpeed = buffSpeed;
            time = buffTime;
            _menu = _Run;
            delay(100);while(getSwitch()!=noPress);return;
          }else{
            saveEEPROM(buffMode,buffSpeed,buffTime);
            lcd.clear();
            lcd.setCursor(2, 0);
            lcd.print("Memory Completed");
            lcd.setCursor(2, 2);
            lcd.print("Waiting");
            disWait(); //แสดงจุดวิ่ง 7 จุด
            reboot();
          }
        }else if(buffSw & sw_val[_Sp]) {
          menu--; 
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Remembering: Mode "); lcd.print(labelSw[buffMode]);
          lcd.setCursor(0, 2);
          lcd.print("1:UP          2:Down");
          lcd.setCursor(0, 3);
          lcd.print("St:Confirm   Sp:Back");
        }
      }
      delay(100);while(getSwitch()!=noPress);
    }else if(buffSw & sw_val[_Sp]){ 
      delay(100);while(getSwitch()!=noPress);     
      menu--; 
      if(menu<0) {
        if(buffMode == _manual) {
          _menu = _Setting;_mode = _noSelect;
          lcd.clear();
          lcd.setCursor(2, 0);
          lcd.print("Exit Manual Mode");
          lcd.setCursor(2, 1);
          lcd.print("Waiting");
          disWait(); //แสดงจุดวิ่ง 7 จุด
        }
        else  _menu = _Memory;
        return;
      }
    }else if(buffSw & sw_val[_1]){
      if(menu==0){
        buffSpeed+=100;
        if(buffSpeed>max_rpm) buffSpeed = min_rpm;
      }else if(menu==1){
        buffTime++;
        if(buffTime>99) buffTime = 1;        
      }
      if(buffDelay>15) buffDelay = buffDelay - 15;
      delay(buffDelay);
    }else if(buffSw & sw_val[_2]){
      if(menu==0){  
          buffSpeed-=100;
          if(buffSpeed<min_rpm) buffSpeed = max_rpm;        
      }else if(menu==1){
        buffTime--;
        if(buffTime<1) buffTime = 99;        
      }
      if(buffDelay>15) buffDelay = buffDelay - 15;
      delay(buffDelay);
    }else if(getSwitch()==noPress) buffDelay = 300;
    
    if(menu==0){
      sprintf(buffChar,"Set Speed: %4d rpm",buffSpeed);
      lcd.setCursor(0, 1);  
      lcd.print(buffChar);
    }else if(menu==1){
      sprintf(buffChar,"Set Time:   %2d min ",buffTime);
      lcd.setCursor(0, 1);  
      lcd.print(buffChar);  
    }
  }
}

bool isPress(int _sw){
  pinMode(sw_pin[_sw][0],OUTPUT);
  pinMode(sw_pin[_sw][1],INPUT);
  digitalWrite(sw_pin[_sw][0],LOW);
  digitalWrite(sw_pin[_sw][1],HIGH);
  delay(1);
  bool _value = !digitalRead(sw_pin[_sw][1]);
  pinMode(sw_pin[_sw][0],INPUT);
  pinMode(sw_pin[_sw][1],INPUT);
  digitalWrite(sw_pin[_sw][0],HIGH);
  digitalWrite(sw_pin[_sw][1],HIGH);
  return(_value);
}

int getSwitch(){
  int buffSw = noPress;
  for(int i = 0; i<8 ; i++){
    if(isPress(i)) buffSw |= sw_val[i];
  }
  return buffSw;
}

void saveEEPROM(int buffMode,int buffSpeed,int buffTime){
  int eeAddress = buffMode*3*sizeof(int);
  EEPROM.put(eeAddress,0xAA55);
  EEPROM.put(eeAddress+sizeof(int),buffSpeed);
  EEPROM.put(eeAddress+(2*sizeof(int)),buffTime);
}

void getEEPROM(int buffMode){
  int eeAddress = buffMode*3*sizeof(int);
  EEPROM.get(eeAddress,memStatus);
  if(memStatus != memSetted) return;
  EEPROM.get(eeAddress+sizeof(int),setSpeed);
  EEPROM.get(eeAddress+(2*sizeof(int)),time);
  if(time<0||time>99) time = defaultTime;
  if(setSpeed<min_rpm||setSpeed>max_rpm) setSpeed = defaultSetSpeed;
}

void disWait()
{
  for(int i=0;i<7;i++) {delay(100);lcd.print(".");}
  delay(100);
}
void reboot() { asm volatile ("jmp 0"); }

ISR (INT0_vect)
{
    count++;
}

ISR (TIMER2_COMPA_vect)
{ 
  buff_x++;
  buff_min++;
  if(buff_x>=25)  {
    //RPM = ((count*4)*60)/(3*N_pole);
    RPM = count*(80/N_pole);
    count=0;
    buff_x=0;
    //digitalWrite(_testPin, digitalRead(_testPin) ^ 1);
  }
  /*if(buff_min>=99){
    buffTempAv[4] = buffTempAv[3];
    buffTempAv[3] = buffTempAv[2];
    buffTempAv[2] = buffTempAv[1];
    buffTempAv[1] = buffTempAv[0];
    buffTempAv[0] = TEMP;
    rTemp = (buffTempAv[0]+buffTempAv[1]+buffTempAv[2]+buffTempAv[3]+buffTempAv[4])/5;
    buff_min = 0;
  }*/
}
