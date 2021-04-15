//Modul Praktikum 4 - Sistem Kendali PWM
//Nama Tim : DuetMaut
//Nama Anggota 1 : Ihsan Maulana Alzidni 6702194020
//Nama Anggota 2 : Ajie Fauhad  6702194011
//Versi Program : 1.0

//pin buat sensor analog
int sensor1 = A0;
int sensor2 = A1;
int sensor3 = A2;
int sensor4 = A3;
int sensor5 = A4;
int sensor6 = A5;
int baca_sensor[6];

//pin buat enable
int pinEnable =4;
int pinEnable2 = 2;

//pin buat motor kiri
int motor_kiri1 = 5;
int motor_kiri2 = 6;

//pin buat motor kanan
int motor_kanan1 = 3;
int motor_kanan2 = 11;

//bantuan
int x;
char buffering [100];

//Error Code
int Last_Error = 0;
int error = 0;
int mtrSPDright, mtrSPDleft, setPointSPD = 255, PID_movement; 

void setup(){
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
  pinMode(sensor5, INPUT);
  pinMode(sensor6, INPUT);
   
  pinMode(pinEnable, OUTPUT);
  pinMode(pinEnable2, OUTPUT);
  pinMode(motor_kiri1, OUTPUT);
  pinMode(motor_kiri2, OUTPUT);
  pinMode(motor_kanan1, OUTPUT);
  pinMode(motor_kanan2, OUTPUT);
  
  Serial.begin(9600);
}

void readsensor() { 
  baca_sensor[0] = analogRead(sensor1);
  baca_sensor[1] = analogRead(sensor2);
  baca_sensor[2] = analogRead(sensor3);
  baca_sensor[3] = analogRead(sensor4);
  baca_sensor[4] = analogRead(sensor5);
  baca_sensor[5] = analogRead(sensor6);
    
  for (x = 0; x <= 5; x++){
    Serial.println (baca_sensor[x]);
  }  
}

void try_PID(){
  int kp = 30, ki = 30, kd = 0  ;
  int rate = error - Last_Error;
  Last_Error = error;
  PID_movement = (kp * error) + (kd * rate) + (ki / rate);
  mtrSPDright = setPointSPD - PID_movement;
  mtrSPDleft = setPointSPD + PID_movement;
  
  digitalWrite(pinEnable, HIGH);
  analogWrite(motor_kiri1, mtrSPDleft);
  analogWrite(motor_kiri2, 0);

  digitalWrite(pinEnable2, HIGH);
  analogWrite(motor_kanan1, mtrSPDright);
  analogWrite(motor_kanan2, 0);
}

void If_Error(){
//hanya sensor 1
if (baca_sensor[0] < 34 && baca_sensor[1] > 34 && 
    baca_sensor[2] > 34 && baca_sensor[3] > 34 && 
    baca_sensor[4] > 34 && baca_sensor[5] > 34){  

    Last_Error = -2;
    try_PID();
    Serial.print ("Error Sensor Detect : ");
    Serial.print (Last_Error);
    Serial.print ("\n");
   }

//hanya sensor 2
if (baca_sensor[0] > 34 && baca_sensor[1] < 34 && 
    baca_sensor[2] > 34 && baca_sensor[3] > 34 && 
    baca_sensor[4] > 34 && baca_sensor[5] > 34){
  
    Last_Error = -1;
    try_PID();
    Serial.print ("Error Sensor Detect : ");
    Serial.print (Last_Error);
    Serial.print ("\n");
   }

//hanya sensor 3  
if (baca_sensor[0] > 34 && baca_sensor[1] > 34 && 
    baca_sensor[2] < 34 && baca_sensor[3] > 34 && 
    baca_sensor[4] > 34 && baca_sensor[5] > 34){
  
    Last_Error = 0;
    try_PID();
    Serial.print ("Error Sensor Detect : ");
    Serial.print (Last_Error);
    Serial.print ("\n");
   }

//hanya sensor 4 
if (baca_sensor[0] > 34 && baca_sensor[1] > 34 && 
    baca_sensor[2] > 34 && baca_sensor[3] < 34 && 
    baca_sensor[4] > 34 && baca_sensor[5] > 34){
  
    Last_Error = 0;
    try_PID();
    Serial.print ("Error Sensor Detect : ");
    Serial.print (Last_Error);
    Serial.print ("\n");
   }

//hanya sensor 5 
if (baca_sensor[0] > 34 && baca_sensor[1] > 34 && 
    baca_sensor[2] > 34 && baca_sensor[3] > 34 && 
    baca_sensor[4] < 34 && baca_sensor[5] > 34){
  
    Last_Error = 1;
    try_PID();
    Serial.print ("Error Sensor Detect : ");
    Serial.print (Last_Error);
    Serial.print ("\n");
   }  

//hanya sensor 6 
if (baca_sensor[0] > 34 && baca_sensor[1] > 34 && 
    baca_sensor[2] > 34 && baca_sensor[3] > 34 && 
    baca_sensor[4] > 34 && baca_sensor[5] < 34){
  
    Last_Error = 2;
    try_PID();
    Serial.print ("Error Sensor Detect : ");
    Serial.print (Last_Error);
    Serial.print ("\n");
   }  
}

void loop(){
    
  readsensor();
  //Sensor 1 dan 2 mendeteksi gelap, sisanya terang >> Duty cycle 0% motor kiri, 50% motor kanan
  if (baca_sensor[0] < 34 && baca_sensor[1] < 34 && 
      baca_sensor[2] > 34 && baca_sensor[3] > 34 && 
      baca_sensor[4] > 34 && baca_sensor[5] > 34){

      Last_Error = -2;
      try_PID();
  }
  //Sensor 2 dan 3 mendeteksi gelap, sisanya terang >> Duty cycle 20% motor kiri, 50% motor kanan
  if (baca_sensor[0] > 34 && baca_sensor[1] < 34 && 
      baca_sensor[2] < 34 && baca_sensor[3] > 34 && 
      baca_sensor[4] > 34 && baca_sensor[5] > 34){

      Last_Error = -1;
      try_PID();
  }
  //Sensor 3 dan 4 mendeteksi gelap, sisanya terang >> Duty cycle 60% pada kedua motor (kedua motor aktif) 
  if (baca_sensor[0] > 34 && baca_sensor[1] > 34 && 
      baca_sensor[2] < 34 && baca_sensor[3] < 34 && 
      baca_sensor[4] > 34 && baca_sensor[5] > 34){

      Last_Error = 0;
      try_PID();
  }
  //Sensor 4 dan 5 mendeteksi gelap, sisanya terang >> Duty cycle 50% motor kiri, 20% motor kanan
  if (baca_sensor[0] > 34 && baca_sensor[1] > 34 && 
      baca_sensor[2] > 34 && baca_sensor[3] < 34 && 
      baca_sensor[4] < 34 && baca_sensor[5] > 34){

      Last_Error = 1;
      try_PID();
  }  
  //Sensor 5 dan 6 mendeteksi gelap, sisanya terang >> Duty cycle 50% Motor kiri, 0% motor kanan
  if (baca_sensor[0] > 34 && baca_sensor[1] > 34 && 
      baca_sensor[2] > 34 && baca_sensor[3] > 34 && 
      baca_sensor[4] < 34 && baca_sensor[5] < 34){

      Last_Error = 2;
      try_PID();
  }  
  //Semua sensor mendeteksi terang >> Duty cycle kedua motor 0% (semua motor mati)
  if (baca_sensor[0] > 34 && baca_sensor[1] > 34 && 
      baca_sensor[2] > 34 && baca_sensor[3] > 34 && 
      baca_sensor[4] > 34 && baca_sensor[5] > 34){
      //kedua motor mati
      digitalWrite(pinEnable, HIGH);
      analogWrite(motor_kiri1, 0);
      analogWrite(motor_kiri2, 0);

      digitalWrite(pinEnable2, HIGH);
      analogWrite(motor_kanan1, 0);
      analogWrite(motor_kanan2, 0);
  }
  If_Error();
}
