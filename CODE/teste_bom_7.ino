//teste bom 7: pid boa, integral com limite, derivada funcionando

//sensor + controle de motores com pid aplicado 

//integrando derividada do erro e integral do erro, e ajustndo os parametros
//testar derivada, pis esta dando problema, vou usar um metodo de quanod ela subir muito, o seu valor vai diminuir aos poucos ou ficar gravada
#include "Wire.h"
#include <MPU6050_light.h>
#include <AccelStepper.h>
#include <AFMotor.h>
#define dirPin 5
#define dirPin2 2
#define stepPin 6
#define stepPin2 3
#define enablePin 7
#define enablePin2 4
#define stepDelay 1000
#define stepsToRevolution 1000

AccelStepper motor1(1, stepPin, dirPin);
AccelStepper motor2(1, stepPin2, dirPin2);

MPU6050 mpu(Wire);  
double error;  //valor desejado menos valor real
int targetValue = 1.5;   //depende da inclinacao da pcb e em qual lado o robo tende cair 


double result;
double derivative;
double integral;
double lastValue;
unsigned long currentTime;
unsigned long lastTime;
float kp = 100;      //ganhos parametros p i d  
float kd = 40;
float ki = 1500;  
                 
 
unsigned long microsValue;
unsigned long millisValue;
unsigned long timeMicros;
unsigned long timeMillis;
unsigned long timeMillis2;

boolean stepState;
boolean dirState;
int speed;
int derivativeCount;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  
  byte status = mpu.begin();

  pinMode(dirPin, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(enablePin2, OUTPUT);
  

  digitalWrite(enablePin, LOW);
  digitalWrite(enablePin2, LOW);

  motor1.setMaxSpeed(20000);	    
  motor1.setAcceleration(1000);  //cmudar aceleracao, qutno miasor a acelrarcao 
  motor2.setMaxSpeed(20000); 
  motor2.setAcceleration(1000);
}

void loop() {

  millisValue = millis();
  currentTime = micros();

  if((millisValue - timeMillis2) >=  10){  // prisr ter um prip loop mais devagar para a leitura do sensor 
    mpu.update();
    timeMillis2 = millisValue;
  }
  error = targetValue - mpu.getAngleY();  //calculo do erro

  //pid objetivo: deixar  erro como zero
  double dt = (currentTime - lastTime) / 1000000.0;  //segundos

  if(dt > 0){
    if(abs(derivative) > 5000){
      derivativeCount++;
      if(derivativeCount == 70){
        derivativeCount = 0;
        derivative = (error - lastValue) / dt;
      }
    }else{
      derivative = (error - lastValue)/dt;
    }
  }
  
  integral += (error + lastValue) * 0.5 * dt;
  integral >= 3000? integral == 3000 : true;
  result = kp*error + kd*derivative + ki*integral;   //o resultado sera a velocidade do motor 

  lastTime = currentTime;
  lastValue = error;
  //controle dos motore: direcao e velocidade
  speed = result; //velocidade dos motores, precisa ser quanto menos o valor maior a velocidade
  
    motor1.setSpeed(-speed);
    motor2.setSpeed(speed);
    if(abs(speed) > 50){ 
      motor1.runSpeed();
      motor2.runSpeed(); 
    }
   
  //debug dos valores
    if((millisValue - timeMillis) >= 1000){ 
    Serial.print("erro: ");   //debug do erro
    Serial.println(error);

    //Serial.print("resultado PID: ");        //debug do resultado
    //Serial.println(result);

    Serial.print("velocidade motores: ");  //debug velocidade
    Serial.println(speed);

    timeMillis = millisValue;   //  e 

    }
}


