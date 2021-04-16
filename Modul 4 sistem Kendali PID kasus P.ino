// PRAKTIKUM 1 SISTEM KENDALI ON/OFF
// KELOMPOK 17 DUET MAUT
// ANGGOTA : AJIE FAUHAD FADHLULLAH (6702194011)
// ANGGOTA : IHSAN MAULANA (6702194020)

//Deklarasi Pin yang digunakan untuk sensor
int sensor1 = A0;
int sensor2 = A1;
int sensor3 = A2;
int sensor4 = A3;
int sensor5 = A4;
int sensor6 = A5;
int baca_sensor[6];
int sendat[6];
int sensorBit;
int but1 = 8;
int but2 = 9;

//Variable bantuan untuk menyimpan memory a.k.a Error Detection Memory(EDM)

//pin penggerak Motor
int ENA = 4; //Pin 1&2 harus HIGH
int ENB = 2; //Pin 3&4 harus HIGH

//pin Motorkiri 
int motorInput1 = 5; //input motor driver
int motorInput2 = 6; //input motor driver

//pin Motorkanan
int motorInput3 = 3;
int motorInput4 = 11;

//Initial Speed of Motor
int kecepatanSetPoint = 150;;

//Konstanta PID
float Kp = 5 ;
float Ki = 0 ; 
float Kd = 0 ;

float error = 0,P = 0, I = 0 , D = 0 , PID_value = 0;
float lastError = 0;

void setup(){
// Keenam Sensor Photodiode sebagai INPUT yaitu sensor cahaya
pinMode(sensor1, INPUT);
pinMode(sensor2, INPUT);
pinMode(sensor3, INPUT);
pinMode(sensor4, INPUT);
pinMode(sensor5, INPUT);
pinMode(sensor6, INPUT);

  //Motor sebagai OOUTPUT pengerak
  pinMode(motorInput1, OUTPUT);
  pinMode(motorInput2, OUTPUT);
  pinMode(motorInput3, OUTPUT);
  pinMode(motorInput4, OUTPUT);

pinMode(ENA, OUTPUT);
pinMode(ENB, OUTPUT);


Serial.begin(9600);
}

void loop(){
  read_sensor_values();
  Serial.print(error);
    hitungPID();
    motor_control();
}

//Membaca sinyal analog dari sensor
void read_sensor_values(){
    baca_sensor[0] = analogRead(sensor1); 
    baca_sensor[1] = analogRead(sensor2); 
    baca_sensor[2] = analogRead(sensor3);
    baca_sensor[3] = analogRead(sensor4); 
    baca_sensor[4] = analogRead(sensor5);
    baca_sensor[5] = analogRead(sensor6); 
 
  // 1 = Gelap 0 = Terang
  // Case 1 0 0 0 0 0
if (baca_sensor[0] < 34 && baca_sensor[1] > 34 && 
    baca_sensor[2] > 34 && baca_sensor[3] > 34 && 
    baca_sensor[4] > 34 && baca_sensor[5] > 34)
    {  
    error = 5;
  Serial.print("\n");
    }
  
  // Case 1 1 0 0 0 0
else if (baca_sensor[0] < 34 && baca_sensor[1] < 34 && 
    baca_sensor[2] > 34 && baca_sensor[3] > 34 && 
    baca_sensor[4] > 34 && baca_sensor[5] > 34){  

    error = 4;
  Serial.print("\n");
   }
  
  // Case 0 1 0 0 0 0
else if (baca_sensor[0] > 34 && baca_sensor[1] < 34 && 
    baca_sensor[2] > 34 && baca_sensor[3] > 34 && 
    baca_sensor[4] > 34 && baca_sensor[5] > 34){  

    error = 3;
  Serial.print("\n");
   }
  
  // Case 0 1 1 0 0 0
else if (baca_sensor[0] > 34 && baca_sensor[1] < 34 && 
    baca_sensor[2] < 34 && baca_sensor[3] > 34 && 
    baca_sensor[4] > 34 && baca_sensor[5] > 34){  

   error = 2;
  Serial.print("\n");
   }
  
  // Case 0 0 0 1 0 0
else if (baca_sensor[0] > 34 && baca_sensor[1] > 34 && 
    baca_sensor[2] < 34 && baca_sensor[3] > 34 && 
    baca_sensor[4] > 34 && baca_sensor[5] > 34){  

    error = 1;
  Serial.print("\n");
   }
  
  // Case 0 0 1 1 0 0
else if (baca_sensor[0] > 34 && baca_sensor[1] > 34 && 
    baca_sensor[2] < 34 && baca_sensor[3] < 34 && 
    baca_sensor[4] > 34 && baca_sensor[5] > 34){  
      
    error = 0;
  Serial.print("\n");
   }
  
  // Case 0 0 0 1 0 0
else if (baca_sensor[0] > 34 && baca_sensor[1] > 34 && 
    baca_sensor[2] > 34 && baca_sensor[3] < 34 && 
    baca_sensor[4] > 34 && baca_sensor[5] > 34){  

    error = -1;
  Serial.print("\n");
   }
  
  // Case 0 0 0 1 1 0
else if (baca_sensor[0] > 34 && baca_sensor[1] > 34 && 
    baca_sensor[2] > 34 && baca_sensor[3] < 34 && 
    baca_sensor[4] < 34 && baca_sensor[5] > 34){  

    error = -2;
  Serial.print("\n");
   }
  
  // Case 0 0 0 0 1 0
else if (baca_sensor[0] > 34 && baca_sensor[1] > 34 && 
    baca_sensor[2] > 34 && baca_sensor[3] > 34 && 
    baca_sensor[4] < 34 && baca_sensor[5] > 34){  

    error = -3;
  Serial.print("\n");
   }
  
  // Case 0 0 0 0 1 1
else if (baca_sensor[0] > 34 && baca_sensor[1] > 34 && 
    baca_sensor[2] > 34 && baca_sensor[3] > 34 && 
    baca_sensor[4] < 34 && baca_sensor[5] < 34){  

    error = -4;
  Serial.print("\n");
   }
  
  // Case 0 0 0 0 0 1
else if (baca_sensor[0] > 34 && baca_sensor[1] > 34 && 
    baca_sensor[2] > 34 && baca_sensor[3] > 34 && 
    baca_sensor[4] > 34 && baca_sensor[5] < 34){  

    error = -5;
  Serial.print("\n");
   }     

// Belokan kiri tajam
else if (baca_sensor[0] < 34 && baca_sensor[1] < 34 && 
    baca_sensor[2] < 34 && baca_sensor[3] < 34 && 
    baca_sensor[4] < 34 && baca_sensor[5] > 34){  

    error = 10;
  Serial.print("\n");
   }   

//Belokan kanan tajam
   else if (baca_sensor[0] > 34 && baca_sensor[1] < 34 && 
    baca_sensor[2] < 34 && baca_sensor[3] < 34 && 
    baca_sensor[4] < 34 && baca_sensor[5] < 34){  

    error = 11;
     Serial.print("\n");
   }   

//Putar Balik
   else if (baca_sensor[0] > 34 && baca_sensor[1] > 34 && 
    baca_sensor[2] > 34 && baca_sensor[3] > 34 && 
    baca_sensor[4] > 34 && baca_sensor[5] > 34){  

    error = 12;
     Serial.print("\n");
   }   

//Garis Finish diam
   else if (baca_sensor[0] < 34 && baca_sensor[1] < 34 && 
    baca_sensor[2] < 34 && baca_sensor[3] < 34 && 
    baca_sensor[4] < 34 && baca_sensor[5] < 34){  

    error = 13;
     Serial.print("\n");
   }   
                  
  //Menampilkan data sensor ke Serial Monitor
  //Data sensor 1-6
  // Formatting tampilkan sensor
  
  for(int i=0; i<=5; i++){
    Serial.print("Sensor");
    Serial.print(i+1);
    Serial.print(": ");
    Serial.print(baca_sensor[i]);
    Serial.print("\n");
    delay(1000);
  }

}


// Menghitung Nilai PID
void hitungPID(){
  P = error;
  I = error + lastError;
  D = error - lastError;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  lastError = error;
}

// Mengatur Kecepatan Motor
void motor_control()
{
  int kecepatanMotorKiri = kecepatanSetPoint - PID_value;
  int kecepatanMotorKanan = kecepatanSetPoint + PID_value;

  // Kecepetan Motor agar tidak melebihhi batas pwm
  kecepatanMotorKiri = constrain(kecepatanMotorKiri, 0, 255);
  kecepatanMotorKanan = constrain(kecepatanMotorKanan, 0, 255);

  Serial.print(" ");
  Serial.print(PID_value);
    Serial.print("\t");
    Serial.print(kecepatanMotorKiri);
    Serial.print("\t");
    Serial.println(kecepatanMotorKanan);
  digitalWrite(ENA, HIGH);
  analogWrite(motorInput1, kecepatanMotorKiri);
  analogWrite(motorInput2, 0);

  digitalWrite(ENB, HIGH);
  analogWrite(motorInput3, kecepatanMotorKanan);
  analogWrite(motorInput4, 0);
  
}
