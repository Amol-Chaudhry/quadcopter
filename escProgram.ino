//
//  escProgram.ino
//  MacOS
//
//  Created by AmolChaudhry.
//


//Required Includes
#include <Wire.h>
#include <EEPROM.h>

//globals
boolean new_function_request,first_angle;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int esc_1, esc_2, esc_3, esc_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4;
int receiver_input[5];
int loop_counter, gyro_address, vibration_counter;
int temperature;
long acc_x, acc_y, acc_z, acc_total_vector[20], acc_av_vector, vibration_total_result;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long zero_timer, timer_1, timer_2, timer_3, timer_4, current_time;
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
byte eeprom_data[36], start, data;
int acc_axis[4], gyro_axis[4];
double gyro_pitch, gyro_roll, gyro_yaw;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
int cal_int;
double gyro_axis_cal[4];


//Setup
void setup(){
  Serial.begin(57600);
  Wire.begin();
  TWBR = 12;

  DDRD |= B11110000;
  DDRB |= B00010000;

  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0);
  PCMSK0 |= (1 << PCINT1);
  PCMSK0 |= (1 << PCINT2);
  PCMSK0 |= (1 << PCINT3);

  for(data = 0; data <= 35; data++)eeprom_data[data] = EEPROM.read(data);

  gyro_address = eeprom_data[32];

  set_gyro_registers();


  while(eeprom_data[33] != 'J' || eeprom_data[34] != 'M' || eeprom_data[35] != 'B'){
    delay(500);
    digitalWrite(12, !digitalRead(12));
  }
  wait_for_receiver();
  zero_timer = micros();

  while(Serial.available())data = Serial.read();
  data = 0;
}

void loop(){
  while(zero_timer + 4000 > micros());
  zero_timer = micros();

  if(Serial.available() > 0){
    data = Serial.read();
    delay(100);
    while(Serial.available() > 0)loop_counter = Serial.read();
    new_function_request = true;
    loop_counter = 0;
    cal_int = 0;
    start = 0;
    first_angle = false;

    if(data == 'r')Serial.println("Reading receiver signals.");
    if(data == 'a')Serial.println("Print the quadcopter angles.");
    if(data == 'a')Serial.println("Gyro calibration starts in 2 seconds (don't move the quadcopter).");
    if(data == '1')Serial.println("Test motor 1 (right front CCW.)");
    if(data == '2')Serial.println("Test motor 2 (right rear CW.)");
    if(data == '3')Serial.println("Test motor 3 (left rear CCW.)");
    if(data == '4')Serial.println("Test motor 4 (left front CW.)");
    if(data == '5')Serial.println("Test all motors together");

    for(vibration_counter = 0; vibration_counter < 625; vibration_counter++){
      delay(3);
      esc_1 = 1000;
      esc_2 = 1000;
      esc_3 = 1000;
      esc_4 = 1000;
      esc_pulse_output();
    }
    vibration_counter = 0;
  }

  receiver_input_channel_3 = convert_receiver_channel(3);
  if(receiver_input_channel_3 < 1025)new_function_request = false;


  if(data == 0 && new_function_request == false){
    receiver_input_channel_3 = convert_receiver_channel(3);
    esc_1 = receiver_input_channel_3;
    esc_2 = receiver_input_channel_3;
    esc_3 = receiver_input_channel_3;
    esc_4 = receiver_input_channel_3;
    esc_pulse_output();
  }

  if(data == 'r'){
    loop_counter ++;
    receiver_input_channel_1 = convert_receiver_channel(1);
    receiver_input_channel_2 = convert_receiver_channel(2);
    receiver_input_channel_3 = convert_receiver_channel(3);
    receiver_input_channel_4 = convert_receiver_channel(4);

    if(loop_counter == 125){
      print_signals();
      loop_counter = 0;
    }


    if(receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1050)start = 1;
    if(start == 1 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1450)start = 2;
    if(start == 2 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1950)start = 0;

    esc_1 = 1000;
    esc_2 = 1000;
    esc_3 = 1000;
    esc_4 = 1000;
    esc_pulse_output();
  }

  if(data == '1' || data == '2' || data == '3' || data == '4' || data == '5'){
    loop_counter ++;
    if(new_function_request == true && loop_counter == 250){
      Serial.print("Set throttle to 1000 (low). It's now set to: ");
      Serial.println(receiver_input_channel_3);
      loop_counter = 0;
    }
    if(new_function_request == false){
      receiver_input_channel_3 = convert_receiver_channel(3);
      if(data == '1' || data == '5')esc_1 = receiver_input_channel_3;
      else esc_1 = 1000;
      if(data == '2' || data == '5')esc_2 = receiver_input_channel_3;
      else esc_2 = 1000;
      if(data == '3' || data == '5')esc_3 = receiver_input_channel_3;
      else esc_3 = 1000;
      if(data == '4' || data == '5')esc_4 = receiver_input_channel_3;
      else esc_4 = 1000;

      esc_pulse_output();

      //For balancing the propellors it's possible to use the accelerometer to measure the vibrations.
      if(eeprom_data[31] == 1){
        Wire.beginTransmission(gyro_address);
        Wire.write(0x3B);
        Wire.endTransmission();
        Wire.requestFrom(gyro_address,6);
        while(Wire.available() < 6);
        acc_x = Wire.read()<<8|Wire.read();
        acc_y = Wire.read()<<8|Wire.read();
        acc_z = Wire.read()<<8|Wire.read();

        acc_total_vector[0] = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));

        acc_av_vector = acc_total_vector[0];

        for(start = 16; start > 0; start--){
          acc_total_vector[start] = acc_total_vector[start - 1];
          acc_av_vector += acc_total_vector[start];
        }

        acc_av_vector /= 17;

        if(vibration_counter < 20){
          vibration_counter ++;
          vibration_total_result += abs(acc_total_vector[0] - acc_av_vector);
        }
        else{
          vibration_counter = 0;
          Serial.println(vibration_total_result/50);
          vibration_total_result = 0;
        }
      }
    }
  }



  if(data == 'a'){

    if(cal_int != 2000){
      Serial.print("Calibrating the gyro");
      //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
      for (cal_int = 0; cal_int < 2000 ; cal_int ++){
        if(cal_int % 125 == 0){
          digitalWrite(12, !digitalRead(12));   //Change the led status to indicate calibration.
          Serial.print(".");
        }
        gyro_signalen();
        gyro_axis_cal[1] += gyro_axis[1];
        gyro_axis_cal[2] += gyro_axis[2];
        gyro_axis_cal[3] += gyro_axis[3];

        PORTD |= B11110000;
        delayMicroseconds(1000);
        PORTD &= B00001111;
        delay(3);
      }
      Serial.println(".");
      //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
      gyro_axis_cal[1] /= 2000;
      gyro_axis_cal[2] /= 2000;
      gyro_axis_cal[3] /= 2000;
    }
    else{
      ///We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while calibrating the gyro.
      PORTD |= B11110000;
      delayMicroseconds(1000);
      PORTD &= B00001111;


      gyro_signalen();


      angle_pitch += gyro_pitch * 0.0000611;
      angle_roll += gyro_roll * 0.0000611;


      angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066);
      angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066);

      //Accelerometer angle calculations
      acc_total_vector[0] = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));

      //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
      angle_pitch_acc = asin((float)acc_y/acc_total_vector[0])* 57.296;
      angle_roll_acc = asin((float)acc_x/acc_total_vector[0])* -57.296;
      
      if(!first_angle){
        angle_pitch = angle_pitch_acc;
        angle_roll = angle_roll_acc;
        first_angle = true;
      }
      else{
        angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
        angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;
      }


      if(loop_counter == 0)Serial.print("Pitch: ");
      if(loop_counter == 1)Serial.print(angle_pitch ,0);
      if(loop_counter == 2)Serial.print(" Roll: ");
      if(loop_counter == 3)Serial.print(angle_roll ,0);
      if(loop_counter == 4)Serial.print(" Yaw: ");
      if(loop_counter == 5)Serial.println(gyro_yaw / 65.5 ,0);

      loop_counter ++;
      if(loop_counter == 60)loop_counter = 0;      
    }
  }
}



//This routine is called every time input 8, 9, 10 or 11 changed state.
ISR(PCINT0_vect){
  current_time = micros();
  //Channel 1=========================================
  if(PINB & B00000001){
    if(last_channel_1 == 0){
      last_channel_1 = 1;
      timer_1 = current_time;
    }
  }
  else if(last_channel_1 == 1){
    last_channel_1 = 0;
    receiver_input[1] = current_time - timer_1;
  }
  //Channel 2=========================================
  if(PINB & B00000010 ){
    if(last_channel_2 == 0){
      last_channel_2 = 1;
      timer_2 = current_time;
    }
  }
  else if(last_channel_2 == 1){
    last_channel_2 = 0;
    receiver_input[2] = current_time - timer_2;
  }
  //Channel 3=========================================
  if(PINB & B00000100 ){
    if(last_channel_3 == 0){
      last_channel_3 = 1;
      timer_3 = current_time;
    }
  }
  else if(last_channel_3 == 1){
    last_channel_3 = 0;
    receiver_input[3] = current_time - timer_3;
  }
  //Channel 4=========================================
  if(PINB & B00001000 ){
    if(last_channel_4 == 0){
      last_channel_4 = 1;
      timer_4 = current_time;
    }
  }
  else if(last_channel_4 == 1){
    last_channel_4 = 0
    receiver_input[4] = current_time - timer_4;
  }
}

//Checck if the receiver values are valid within 10 seconds
void wait_for_receiver(){
  byte zero = 0;
  while(zero < 15){
    if(receiver_input[1] < 2100 && receiver_input[1] > 900)zero |= 0b00000001;
    if(receiver_input[2] < 2100 && receiver_input[2] > 900)zero |= 0b00000010;
    if(receiver_input[3] < 2100 && receiver_input[3] > 900)zero |= 0b00000100;
    if(receiver_input[4] < 2100 && receiver_input[4] > 900)zero |= 0b00001000;
    delay(500);
  }
}

//This part converts the actual receiver signals to a standardized 1000 – 1500 – 2000 microsecond value.
//The stored data in the EEPROM is used.
int convert_receiver_channel(byte function){
  byte channel, reverse;
  int low, center, high, actual;
  int difference;

  channel = eeprom_data[function + 23] & 0b00000111;
  if(eeprom_data[function + 23] & 0b10000000)reverse = 1;
  else reverse = 0;

  actual = receiver_input[channel];
  low = (eeprom_data[channel * 2 + 15] << 8) | eeprom_data[channel * 2 + 14];
  center = (eeprom_data[channel * 2 - 1] << 8) | eeprom_data[channel * 2 - 2];
  high = (eeprom_data[channel * 2 + 7] << 8) | eeprom_data[channel * 2 + 6];

  if(actual < center){
    if(actual < low)actual = low;
    difference = ((long)(center - actual) * (long)500) / (center - low);
    if(reverse == 1)return 1500 + difference;
    else return 1500 - difference;
  }
  else if(actual > center){
    if(actual > high)actual = high;
    difference = ((long)(actual - center) * (long)500) / (high - center);
    if(reverse == 1)return 1500 - difference;
    else return 1500 + difference;
  }
  else return 1500;
}

void print_signals(){
  Serial.print("Start:");
  Serial.print(start);

  Serial.print("  Roll:");
  if(receiver_input_channel_1 - 1480 < 0)Serial.print("<<<");
  else if(receiver_input_channel_1 - 1520 > 0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.print(receiver_input_channel_1);

  Serial.print("  Pitch:");
  if(receiver_input_channel_2 - 1480 < 0)Serial.print("^^^");
  else if(receiver_input_channel_2 - 1520 > 0)Serial.print("vvv");
  else Serial.print("-+-");
  Serial.print(receiver_input_channel_2);

  Serial.print("  Throttle:");
  if(receiver_input_channel_3 - 1480 < 0)Serial.print("vvv");
  else if(receiver_input_channel_3 - 1520 > 0)Serial.print("^^^");
  else Serial.print("-+-");
  Serial.print(receiver_input_channel_3);

  Serial.print("  Yaw:");
  if(receiver_input_channel_4 - 1480 < 0)Serial.print("<<<");
  else if(receiver_input_channel_4 - 1520 > 0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.println(receiver_input_channel_4);
}

void esc_pulse_output(){
  zero_timer = micros();
  PORTD |= B11110000;
  timer_channel_1 = esc_1 + zero_timer;
  timer_channel_2 = esc_2 + zero_timer;
  timer_channel_3 = esc_3 + zero_timer;
  timer_channel_4 = esc_4 + zero_timer;

  while(PORTD >= 16){
    esc_loop_timer = micros();
    if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;
    if(timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;
    if(timer_channel_3 <= esc_loop_timer)PORTD &= B10111111;
    if(timer_channel_4 <= esc_loop_timer)PORTD &= B01111111;
  }
}

void set_gyro_registers(){
  //Setup the MPU-6050
  if(eeprom_data[31] == 1){
    Wire.beginTransmission(gyro_address);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();

    Wire.beginTransmission(gyro_address);
    Wire.write(0x1B);
    Wire.write(0x08);
    Wire.endTransmission();

    Wire.beginTransmission(gyro_address);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();


    Wire.beginTransmission(gyro_address);
    Wire.write(0x1B);
    Wire.endTransmission();
    Wire.requestFrom(gyro_address, 1);
    while(Wire.available() < 1);
    if(Wire.read() != 0x08){
      digitalWrite(12,HIGH);
      while(1)delay(10);
    }

    Wire.beginTransmission(gyro_address);
    Wire.write(0x1A);
    Wire.write(0x03);
    Wire.endTransmission();

  }  
}

void gyro_signalen(){
  //Read the MPU-6050
  if(eeprom_data[31] == 1){
    Wire.beginTransmission(gyro_address);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(gyro_address,14);
    while(Wire.available() < 14);
    acc_axis[1] = Wire.read()<<8|Wire.read();
    acc_axis[2] = Wire.read()<<8|Wire.read();
    acc_axis[3] = Wire.read()<<8|Wire.read();
    temperature = Wire.read()<<8|Wire.read();
    gyro_axis[1] = Wire.read()<<8|Wire.read();
    gyro_axis[2] = Wire.read()<<8|Wire.read();
    gyro_axis[3] = Wire.read()<<8|Wire.read();
  }

  if(cal_int == 2000){
    gyro_axis[1] -= gyro_axis_cal[1];
    gyro_axis[2] -= gyro_axis_cal[2];
    gyro_axis[3] -= gyro_axis_cal[3];
  }
  gyro_roll = gyro_axis[eeprom_data[28] & 0b00000011];
  if(eeprom_data[28] & 0b10000000)gyro_roll *= -1;
  gyro_pitch = gyro_axis[eeprom_data[29] & 0b00000011];
  if(eeprom_data[29] & 0b10000000)gyro_pitch *= -1;
  gyro_yaw = gyro_axis[eeprom_data[30] & 0b00000011];
  if(eeprom_data[30] & 0b10000000)gyro_yaw *= -1;

  acc_x = acc_axis[eeprom_data[29] & 0b00000011];
  if(eeprom_data[29] & 0b10000000)acc_x *= -1;
  acc_y = acc_axis[eeprom_data[28] & 0b00000011];
  if(eeprom_data[28] & 0b10000000)acc_y *= -1;
  acc_z = acc_axis[eeprom_data[30] & 0b00000011];
  if(eeprom_data[30] & 0b10000000)acc_z *= -1;
}







