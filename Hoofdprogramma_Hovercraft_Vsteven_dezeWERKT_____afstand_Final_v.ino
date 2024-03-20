// Libraries
#include <VL53L0X.h>
#include <Wire.h>
#include "MPU9250.h"


MPU9250 mpu;
VL53L0X TOF1;
VL53L0X TOF2;
VL53L0X TOF3;


// Variable
const long SENSOR1CYCLUSTIJD = 10;
const long SENSOR2CYCLUSTIJD = 50;
const long SENSOR3CYCLUSTIJD = 60;
const long IMUCYCLUSTIJD = 70;
const long REGELCYCLUSTIJD = 100;
const long PICYCLUSTIJD = 500;
long tijd;
long oudeSensor1tijd = 0;
long oudeSensor2tijd = 0;
long oudeSensor3tijd = 0;
float X = 0;
float Y = 0;
float Z = 0;
float hoek = 0;
const float FmaxL = 0.14715, FmaxR = 0.14715, FminL = -0.16677, FminR = -0.17658;


// Variable regelaar picam
long cyclustijd = 10;
float Fx, Fy;
const float x_kp_pi = 30, x_kd_pi = 30, x_ki_pi = 1, x_Fmin = -0.1, x_Fmax=0.1;
const float y_kp_pi = 30, y_kd_pi = 30, y_ki_pi = 1, y_Fmin = -0.1, y_Fmax=0.1;
float x_error_pi, x_error_oud_pi, x_int_Error_pi, y_error_pi, y_error_oud_pi, y_int_Error_pi;
const float x_sp_pi = 0, y_sp_pi = 0;
long oudeimu3tijd = 0;
long oudePiTijd = 0;
long oudeRegeltijd = 0;
int regelaarnummer = 1;

uint8_t t[8]={};//array waarin de chars vanuit de pi opgeslagen worden

long t_oud, t_nw;
float dt;

//TOF
int afstandTOF1;
int afstandTOF2;
int afstandTOF3;
#define adres_TOF1 0x33  // vookant
#define adres_TOF2 0x36  // zijkant voor
#define adres_TOF3 0x35  // zijkant achter
#define pin_TOF1 12
#define pin_TOF2 11
#define pin_TOF3 10

//Motoren
int PWM_links = 0;
int PWM_rechts = 0;
int PWM_midden = 0;
int forward_links = true;
int forward_rechts = true;
int forward_midden = true;
float rechtermotorforce;
float linkermotorforce;
float zijmotorforce;

// Pins
int motor_left_Lpwm = 4;
int motor_left_Rpwm = 5;
int motor_right_Lpwm = 6;
int motor_right_Rpwm = 7;
int motor_side_Lpwm = 2;
int motor_side_Rpwm = 3;
#define safety_relay_blowers_pin 22
#define safety_relay_actuatoren_pin 23
#define stroom_meter A3
#define pin_accu1 A0
#define pin_accu12 A1
#define pin_accu123 A2



// Setupfuncties
void setupMotoren(){
  pinMode(motor_left_Lpwm, OUTPUT);
  pinMode(motor_left_Rpwm, OUTPUT);
  pinMode(motor_right_Lpwm, OUTPUT);
  pinMode(motor_right_Rpwm, OUTPUT);
  pinMode(motor_side_Lpwm, OUTPUT);
  pinMode(motor_side_Rpwm, OUTPUT);
  }
void setupTOFSensor1()
{
  // voorkant
  Serial.println("TOF1 init");
  digitalWrite(pin_TOF1, HIGH);
  delay(10);
  TOF1.init(true);
  delay(10);
  TOF1.setAddress(adres_TOF1);
  delay(10);
}
void setupTOFSensor2()
{
  // zijkant voor
  Serial.println("TOF2 intit");
  digitalWrite(pin_TOF2, HIGH);
  delay(10);
  TOF2.init(true);
  delay(10);
  TOF2.setAddress(adres_TOF2);
  delay(10);
}
void setupTOFSensor3()
{
  // zijkant achter
  Serial.println("TOF3 init");
  digitalWrite(pin_TOF3, HIGH);
  delay(10);
  TOF3.init(true);
  delay(10);
  TOF3.setAddress(adres_TOF3);
  delay(10);
}


void mpu_setup(){
   if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }
    mpu.verbose(true);
    mpu.calibrateAccelGyro(); 
    mpu.verbose(false);
}


// Alle functies

int pwm_check_big(int current_pwm)
{
  if (current_pwm >= 0 && current_pwm <= 255)
  {
    return current_pwm;
  }
  else if (current_pwm > 255)
  {
    return 255;
  }
  else
  {
    return 0; //kan niet
  }
}
int pwm_check_klein(int current_pwm)
{
  if (current_pwm >= 0 && current_pwm <= 109)
  {
    return current_pwm;
  }
  else if (current_pwm > 109)
  {
    return 109;
  }
  else
  {
    return current_pwm*-1;
  }
}

void aansturingMotoren(float x, float y){ //functie voor het bewegen van de UAV over de x en y as
  Serial.print("x: ");
  Serial.println(x);
  Serial.print("y: ");
  Serial.println(y);
  //kracht y achteruit
  if (y <= 0){
    
    rechtermotorforce = (y/2) * -1;
    linkermotorforce = (y/2) * -1;
    //kracht naar pwm
    Serial.print("PWM -Y: \t");
    //rechtermotor linksom
    int PWMMotorRL = pwm_check_big((1003.7*rechtermotorforce)+76.24);
    Serial.print(PWMMotorRL);
    Serial.print("\t");
    //linkermotor linksom
    int PWMMotorLL = pwm_check_big((1263.1*linkermotorforce)+58.58);
    if (y = 0){
      PWMMotorLL = 0;
      PWMMotorRL = 0;
    }
    Serial.print(PWMMotorLL);
    Serial.print("\n");
    analogWrite(motor_right_Lpwm, PWMMotorRL);
    analogWrite(motor_left_Lpwm, PWMMotorLL);
    analogWrite(motor_right_Rpwm, 0);
    analogWrite(motor_left_Rpwm, 0);
  }
  //kracht y vooruit
  else if (y > 0){
    rechtermotorforce = y/2;
    linkermotorforce = y/2;
    //kracht naar pwm
    Serial.print("PWM Y: \t");
    //rechtermotor rechtsom
    int PWMMotorRR= pwm_check_big((1353.4*rechtermotorforce)+72.89);
    Serial.print(PWMMotorRR);
    Serial.print("\t");
    //linkermotor Rechtsom
    int PWMMotorLR = pwm_check_big((1020.4*linkermotorforce)+113.37);
    if (y = 0){
      PWMMotorLR = 0;
      PWMMotorRR = 0;
    }
    Serial.print(PWMMotorLR);
    Serial.print("\n");
    analogWrite(motor_right_Rpwm, PWMMotorRR);
    analogWrite(motor_left_Rpwm, PWMMotorLR);
    analogWrite(motor_right_Lpwm, 0);
    analogWrite(motor_left_Lpwm, 0);  
  }
  else{
    Serial.println("PWM Y: \t 0 \t 0");
    analogWrite(motor_right_Rpwm, 0);
    analogWrite(motor_left_Rpwm, 0);
  }

  //kracht x achteruit
  if (x < 0){
    zijmotorforce = x/2;
    //kracht naar pwm 
    Serial.print("PWM -X: \t");
    //linksom
    int PWMMotorZL = pwm_check_klein((5828.1*zijmotorforce)+17.326);
    Serial.println(PWMMotorZL);
    analogWrite(motor_side_Lpwm, PWMMotorZL);
  }
  //kracht x vooruit
  else if (x > 0){
     zijmotorforce = x/2;
    //kracht naar pwm
    Serial.print("PWM X: \t");
    //Rechtsom
    int PWMMotorZR = pwm_check_klein((9688.7*zijmotorforce)+0);
    Serial.println(PWMMotorZR);
    analogWrite(motor_side_Rpwm, PWMMotorZR);
  }
  else{
    zijmotorforce = x/2;
    //kracht naar pwm
    Serial.println("PWM X: \t 0");
    analogWrite(motor_side_Rpwm, 0);
  }
  Serial.print("Motorforce:  R: ");
  Serial.print(rechtermotorforce);
  Serial.print("\t  L: ");
  Serial.print(linkermotorforce);
  Serial.print("\t  Z: ");
  Serial.println(zijmotorforce);
  
}
  

float leesTOFSensor1() {
  return TOF1.readRangeContinuousMillimeters();
  }
float leesTOFSensor2() {
  return TOF2.readRangeContinuousMillimeters();
  }
float leesTOFSensor3() {
  return TOF3.readRangeContinuousMillimeters();
  }

int leesPi(String gevraagdeWaarde)
{
  Wire.beginTransmission(9); // transmit to device #9
  Wire.write(gevraagdeWaarde.c_str()); // sends one bytes
  Wire.endTransmission(); // stop transmitting
  delay(15); // Maak deze niet te klein! Maar geen delay gebruiken.
  Wire.requestFrom(9, 8); // request 8 bytes from slave device #9

  if (Wire.available()) { 
    for ( int i = 0; i < 4; ++i ){
      t[i] = Wire.read(); // elke char naar array t
      Serial.print(t[i],HEX);
      }
      Serial.print("\t");

    // converteren naar little endien
    int x = 0;
    x |= t[0] << 24;
    x |= t[1] << 16;
    x |= t[2] << 8;
    x |= t[3] << 0;

    Serial.print(x,HEX);
    Serial.print("\t");
    
    Serial.print("i2c waarden: ");
    Serial.println(x);
    return x;
    }
}

void lees_stroommeter()
{
  int waarde1 = analogRead(stroom_meter);
  float a = (waarde1 * 9.5 / 1024);
  if (a >= 7.0) {
    digitalWrite(safety_relay_actuatoren_pin, LOW);
    digitalWrite(safety_relay_blowers_pin, LOW);
    Serial.println("Stroom te hoog");
  }
}

void check_accu(){
  Serial.print("battery 1: ");
  Serial.println(analogRead(pin_accu1));
  int s1= analogRead(pin_accu1);
   //cel 1
  Serial.print("battery 2: ");
  Serial.println(analogRead(pin_accu12));
  int s2= analogRead(pin_accu1+2);
   //cel 1+2
  Serial.print("battery 3: ");
  Serial.println(analogRead(pin_accu123));
  int s3= analogRead(pin_accu1+2+3);
   //cel 1+2+3
  Serial.println(); 
  float cel1gemiddelde = 3.7;
  float cel2gemiddelde = 3.7;
  float cel3gemiddelde = 3.7;

  float a1 = (s1 * 4.1) / 895;
  float a2 = (s2 * 4.1) / 895;
  float a3 = (s3 * 4.1) / 895;

  cel1gemiddelde = (cel1gemiddelde + a1) / 2;
  cel2gemiddelde = (cel2gemiddelde + a2) / 2;
  cel3gemiddelde = (cel3gemiddelde + a3) / 2;
  
  if ((cel1gemiddelde <= 3.3) || (cel1gemiddelde <= 3.3) || (cel3gemiddelde <= 3.3)){
//    digitalWrite(safety_relay_actuatoren_pin, LOW);
    digitalWrite(safety_relay_blowers_pin, LOW);
    Serial.println("Spanning laag");
  }
 }

// Regelaars
void afstand_tot_muur(){    //1 (Steven)
  Serial.println("regelaar Steven");
  // Instellingen regelaar
  const float afstand_Kp = 0.5, afstand_Kd = .5 ; // Regelaarparameters
  //const float afstand_Kp = 0.4, afstand_Kd = 4., afstand_ki = 0.01; // Regelaarparameters
  
  float s, v;
  const float sp = 300;       // setpoint = 30 cm

  float error, error_oud, d_error, cumerror;
  const float m = 1.206;      // kg
  float afstand_F, Fclipped;

  const float Fmax = min(FmaxL, FmaxR), Fmin = max(FminL, FminR);
  
  
  
  t_oud = t_nw;
  // Wacht tot de cyclustijd bereikt is:
  while (t_nw - t_oud < cyclustijd) t_nw = millis();
  dt = (t_nw - t_oud) / .001; // omzetten ms => s
  
  s = afstandTOF1; //Afstand TOF1 opvragen

  // Regelaar
  error_oud = error;
  error = sp - s;
  cumerror += error * dt;
  d_error = error - error_oud;
  //afstand_F = error * afstand_Kp + afstand_ki * cumerror + d_error / dt * afstand_Kd;
  afstand_F = error * afstand_Kp + d_error / dt * afstand_Kd;

  afstand_F = afstand_F / 1000;
  
  Fclipped = max(min(afstand_F, Fmax), Fmin); // Fmin < F < +Fmax
  Serial.println("error");
  Serial.println(error);
  //Serial.println("afstand");
  //Serial.println(afstand_F);

  Fclipped = Fclipped*-1;
  Serial.println("Fclipped");
  Serial.println(Fclipped);

  Serial.println("afstand_F");
  Serial.println(afstand_F);
  aansturingMotoren(0, Fclipped);
  }
void langs_muur(){          //2 (Vincent)
  }
void hoeksensor(){          //3 (Aron)
  }
void picam(float x, float y){             //4 (Duco)
  t_nw = millis();
  if (t_nw - t_oud > cyclustijd) {
    dt = (t_nw - t_oud) * .001;
    t_oud = t_nw;

    Serial.println("regelaar Duco");
    x_error_pi = x_sp_pi - x;                      // error bepalen
    Serial.print("error x: ");
    Serial.println(x_error_pi);
    x_int_Error_pi += x_error_pi * dt;             // integraal bepalen
    Fx = x_kp_pi * x_error_pi + x_ki_pi * x_int_Error_pi + x_kd_pi * (x_error_pi - x_error_oud_pi)/dt;
    Fx = constrain(Fx, x_Fmin, x_Fmax);
    Serial.print("Fx: ");
    Serial.println(Fx);
    x_error_oud_pi = x_error_pi;

    y_error_pi = y_sp_pi - y;                      // error bepalen
    Serial.print("error y: ");
    Serial.println(y_error_pi);
    y_int_Error_pi += y_error_pi * dt;             // integraal bepalen
    Fy = y_kp_pi * y_error_pi + y_ki_pi * y_int_Error_pi + y_kd_pi * (y_error_pi - y_error_oud_pi)/dt;
    Fy = constrain(Fy, y_Fmin, y_Fmax);
    Serial.print("Fy: ");
    Serial.println(Fy);
    y_error_oud_pi = y_error_pi;

    aansturingMotoren(Fx, Fy);
  }
}   
void hybride(){             //5 (Andreas)
  }       



void setup()
{
  Wire.begin();
  Serial.begin(115200);
  Serial.println("Setup");
  pinMode(pin_TOF1, OUTPUT);
  pinMode(pin_TOF2, OUTPUT);
  pinMode(pin_TOF3, OUTPUT);
  digitalWrite(pin_TOF1, LOW);
  digitalWrite(pin_TOF2, LOW);
  digitalWrite(pin_TOF3, LOW);
  setupTOFSensor1();
  setupTOFSensor2();
  setupTOFSensor3();
  TOF1.startContinuous();
  TOF2.startContinuous();
  TOF3.startContinuous();
  setupMotoren();
  mpu_setup();
  pinMode(safety_relay_blowers_pin, OUTPUT);
  pinMode(safety_relay_actuatoren_pin, OUTPUT);
}

void loop()
{
  tijd = millis();
  if (tijd - oudeSensor1tijd > SENSOR1CYCLUSTIJD) // 10 ms
  {
    oudeSensor1tijd = tijd;
    afstandTOF1 = leesTOFSensor1();
    Serial.print("Tof1: ");
    Serial.println(afstandTOF1);
  }
  if (tijd - oudeSensor2tijd > SENSOR2CYCLUSTIJD) // 50 ms
  {
    oudeSensor2tijd = tijd;
    afstandTOF2 = leesTOFSensor2();
    Serial.print("Tof2: ");
    Serial.println(afstandTOF2);
  }
  if (tijd - oudeSensor3tijd > SENSOR3CYCLUSTIJD) // 60 ms
  {
    oudeSensor3tijd = tijd;
    afstandTOF3 = leesTOFSensor3();
    Serial.print("Tof3: ");
    Serial.println(afstandTOF3);
  }
  if (tijd - oudeimu3tijd > IMUCYCLUSTIJD) // 70 ms
  {
    if (mpu.update()) {
      static uint32_t prev_ms = millis();
      if (millis() > prev_ms + 25) {
        Serial.print("Yaw, Pitch, Roll: ");
        Serial.print(mpu.getYaw());
        Serial.print(", ");
        Serial.print(mpu.getPitch());
        Serial.print(", ");
        Serial.println(mpu.getRoll());
        //print_accel();
        //print_gyro();
        prev_ms = millis();
      }
    }
  }
  if (tijd - oudePiTijd > PICYCLUSTIJD) // 500 ms
  {
    oudePiTijd = tijd;
    // Vision
    X = leesPi("X");
    Y = leesPi("Y");
    Z = leesPi("Z");
    hoek = leesPi("H");
    // HMI
    //regelaarnummer = leesPi("R");
    Serial.print("Regelaar: ");
    Serial.println(regelaarnummer);
  }
  // Selecteer het practicum
  switch (regelaarnummer)
  {
    case 1:
      Serial.println("R1");
      afstand_tot_muur();
      break;
    case 2:
      Serial.println("R2");
      langs_muur();
      break;
    case 3:
      Serial.println("R3");
      hoeksensor();
      break;
    case 4:
      Serial.println("R4");
      picam(X,Y);
      break;
    case 5:
      Serial.println("R5");
      hybride();
      break;
    default:
      //C Statements
      ;
  }
  // Stuur alle actuatoren aan en check de accu
  if (tijd - oudeRegeltijd > REGELCYCLUSTIJD) // 100 ms
  {
    oudeRegeltijd = tijd;
    Serial.println("Accu check");
    lees_stroommeter();
    check_accu();
  }
}
