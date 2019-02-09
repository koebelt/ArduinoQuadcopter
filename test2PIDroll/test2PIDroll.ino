
/////////  Controller de vol d'un quadricopter basé sur arduino UNO
/////////  Centrale inertielle :MPU6050 :Utilisation du DMP en interruption et lecture du FIFO par le DMP
/////////  Détermine les mesures en angles par float à 100 Hz
/////////  la lecture de la RX par int à 50 Hz. lecture en µs des sticks, conversions en angles en float degrés précision 1*16 éme
/////////  et zône morte centrale

#include "Config.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include "PID_v1.h"
#include "Servo.h"


//////////  =====================================  concerne la lecture de la RX  =============================  //////////////
volatile unsigned long current_time = 0;
volatile unsigned long timer[5] = {0,0,0,0,0};
volatile byte          previous_state[5] = {0,0,0,0,0};  // Previous state of each channel (HIGH or LOW)
volatile unsigned int  pulse_duration[5] = {1500, 1500, 1000,1500,1000}; //  pulse Duration on each channel in µs (1000µs..2000µs)
volatile boolean       channel1Changed = false;
volatile boolean       channel3Changed = false;
volatile boolean       channel5Changed = false;

float                  consigneROLL = 0.0; //en degrés >0 et <0
int                    consigneGAZ = 1000; //en microsecondes de 1060µs à 1800µs
int                    consigneETAT = ARMED;

//////////  =====================================  concerne l'IMU  =============================  //////////////

MPU6050 mpu(0x68);//instantiation MPU a l'adresse I2C 0x68

bool          dmpReady = false;  // set true if DMP init was successful
uint8_t       mpuIntStatus = 0;   // holds actual interrupt status byte from MPU
uint8_t       devStatus = 1;      // return status after each device operation (0 = success, !0 = error)
uint16_t      packetSize = 0;    // expected DMP packet size (default is 42 bytes)
uint16_t      fifoCount = 0;     // count of all bytes currently in FIFO
uint8_t       fifoBuffer[64]; // FIFO storage buffer
Quaternion    q;           // [w, x, y, z]         quaternion container
VectorFloat   gravity;    // [x, y, z]            gravity vector
float         yrp[3] {0.0,0.0,0.0};           // [yaw, roll, pitch]   yaw/pitch/roll container as radians
int16_t       gx, gy, gz;
VectorInt16   gyroRates;   // [x, y, z]           Gyro sensor measurements par DMP
float         mesureROLL = 0.0; //en degrés >0 et <0
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

//////////  =====================================  concerne le vol  =============================  //////////////
int           EtatDuVol = ARMED;

//////////  =====================================  concerne PID  =============================  //////////////

// PID ROLL Taux angulaire

double      Roll1_PID_input = 0.0;  // entrée PID
double      Roll1_PID_setpoint = 0.0; //consigne PID
double      Roll1_PID_output = 0.0; // sortie PID
int         sampleTimeRTA = SAMPLE_TIMERTA; // échantillonnage valeur par défaut
double      RtaKp=ROLL_TANGLEX_KP,  RtaKi=ROLL_TANGLEX_KI, RtaKd=ROLL_TANGLEX_KD; // valeurs par defaut: 2 0 0
int         sensCorrectionRTA = DIRECT; //valeur par défaut
// reverse : erreur >0 -> sortie PID <0
double      Roll1outPutLimits = 200; // valeur par défaut, sera fixé en outPutLimits*-1 .. outPutLimits
boolean     roll_taux_angle_pidOK = false;

// instanciation objet ROLL
PID         roll_taux_angle_pid (&Roll1_PID_input, &Roll1_PID_output, &Roll1_PID_setpoint, RtaKp, RtaKi, RtaKd, P_ON_E, sensCorrectionRTA);

// PID ROLL acceleration

double      Roll2_PID_input = 0.0;
double      Roll2_PID_setpoint = 0.0;
double      Roll2_PID_output = 0.0;
int         sampleTimeRAC = SAMPLE_TIMERAC; // échantillonnage valeur par défaut
double      RacKp=ROLL_ACCEL_KP,  RacKi=ROLL_ACCEL_KI, RacKd=ROLL_ACCEL_KD; // valeurs par defaut: 2 0 0
int         sensCorrectionRAC = DIRECT; //valeur par défaut
double      Roll2outPutLimits = 400; // valeur par défaut, sera fixé en outPutLimits*-1 .. outPutLimits
boolean     roll_accel_pidOK = false;



PID         roll_accel_pid (&Roll2_PID_input, &Roll2_PID_output, &Roll2_PID_setpoint, RacKp, RacKi, RacKd, P_ON_E, sensCorrectionRAC);

//////////  =====================================  concerne Moteurs  =============================  //////////////
Servo         motA, motB; // instanciation   pour la séquence de démarrage
unsigned long ESC_A = 0;
unsigned long ESC_B = 0;
unsigned long loop_timer, timerMotA,timerMotB,esc_loop_timer;
// ------------------------concerne l'UI dialogue---------------------------------------------------

String        inputString=""; // chaine de caractères pour contenir les données
boolean       stringComplete=false ; // pour savoir si la chaine est complète
String        command="";    // init
String        argument="";   // init





/*  =================================================================================================================  *\
 *                                                        SETUP                                                        *
\*  =================================================================================================================  */
/*    Initiallisation : Des Variables                                                                                  *\
 *                      Des ESC                                                                                        *
 *                      Des pins                                                                                       *
 *                      Du Récépteur                                                                                   *
 *                      De l'IMU                                                                                       *
 *                      Des PID                                                                                        *
 *                                                                                                                     *
 *                                                                                                                     *
\*  =================================================================================================================  */
 
  void setup() {                               
     initVars();
     Serial.begin(115200);
     startESC(); // utilise classe SERVO pour envoyer pulse start puis detach: SERVO restera inutilisé par après
     configPins();
     initRXread();
     initImu();
     initPID();
               }  // Fin du SETUP





/*  =================================================================================================================  *\
 *                                                        LOOP                                                         *
\*  =================================================================================================================  */
/*    On s'occupe de : Réccupérer et convertir les informations de l'IMU                                               *\
 *                     Réccupérer et convertir les informations de la RC                                               *
 *                     Calculer les PID                                                                                *
 *                     Calculer les durées à envoyer aux ESC                                                           *
 *                     Piloter les ESC                                                                                 *
 *                                                                                                                     * 
 *                                                                                                                     *
\*  =================================================================================================================  */


void loop() {
      
    if (!dmpReady) return;  //Si il n'y a pas d'interruptions donc sortir de la loop: PLUS RIEN ne se passe


    while (!mpuInterrupt && fifoCount < packetSize) {    //  Si il n'y a pas d'interruptions mpu ET que le FIFO n'est pas rempli
                                                        //  càd pas de données brutes et pas ( ni) de donnée traitées par DMP
                                                       //  Durée 5 ms MAX !
       
        pulseDurationToAngles();  //---- conversion des données foirnies par la RC
   
                                                    } // fin du while


               /*                                                                                                                   *\
                * maintenant il y a interruption >> réagir : préparer la prochaine int et vérifier !!: int OK? ET FIFO données OK ? *
               \*                                                                                                                   */
                       
     
     mpuInterrupt = false;              // reset l'interruption de l'IMU
     mpuIntStatus = mpu.getIntStatus(); 
     fifoCount = mpu.getFIFOCount();                   
    
     
     if ((mpuIntStatus & 0x10) || fifoCount == 1024) {  // Verifier si il n'y a pas un débordement sur la mémoire du FIFO (this should never happen unless our code is too inefficient)
         mpu.resetFIFO();  // reset le fifo pour pouvoir continuer
         Serial.println(F("FIFO overflow!"));
                                                     }                                                                                            
     else if (mpuIntStatus & 0x02) {    // Si le fifo est OK, verifier si DMP data ready interrupt (this should happen frequently)
        
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();    // wait for correct available data length, should be a VERY short wait
        
        mpu.getFIFOBytes(fifoBuffer, packetSize); // read a packet from FIFO
        fifoCount -= packetSize;  // INUTILISE


               /*                                                       *\
                * tout est OK il faut utiliser ce qui est dans le FIFO  *
               \*                                                       */
               
        mpu.dmpGetQuaternion(&q, fifoBuffer);      // mpu dit à DMP de lire la partie quaterion contenue ds le FIFO  et mettre dans q
        mpu.dmpGetGravity(&gravity, &q);           //mpu dit a DMP de calcler la gravitée a partir du quaternion et mettre ds gravity
        mpu.dmpGetYawRollPitch(yrp, &q, &gravity); //mpu dit a dmp de calculer yrp a partir de la gravitée et du quaternion et de mettre dans ypr
                                                   // le résultat est les angles en RADIANS
                                                        
               /*                                                                                                        *\
                * l'imu commute roll et pitch du fait de son montage mécanique; donc rectifications et sortie en degrés  *
               \*                                                                                                        */
   
        mesureROLL =  0-(yrp[2] * radToDegrees);
        mpu.dmpGetGyro(&gyroRates, fifoBuffer); 
        gx = gyroRates.x;
               
               /*                  *\
                *  Calcul Des PID  *
               \*                  */
        
        //Compute Roll PID taux angulaire
        Roll1_PID_setpoint = mesureROLL;          //consigne
        Roll1_PID_input = consigneROLL;
        roll_taux_angle_pidOK = roll_taux_angle_pid.Compute();  // resultat ds pid_output borné à  +-200 (défaut)

        //compute Roll PID acceleration   
        Roll2_PID_setpoint = Roll1_PID_output;    //consigne
        Roll2_PID_input = gx;
        roll_accel_pidOK = roll_accel_pid.Compute();  // resultat ds pid_output borné à  +-200 (défaut)  


        if(roll_taux_angle_pidOK && roll_accel_pidOK){
          // Serial.print( mesureROLL); Serial.print("\t");
          //Serial.print (Roll1_PID_output);Serial.print("\t");
          computeESC();
        //  Serial.print (consigneROLL);Serial.print("\t");Serial.print(ESC_A);Serial.print("\t");Serial.print(ESC_B);Serial.print("\t");Serial.println(Roll2_PID_output);
           
          }; 

        if (consigneETAT == STABILIZE){
         commandAllMotors();
            };

    }; // fin du else mpuIntStatus cad tous data prêts et donc actions de corrections

// ============================================= ds seconde partie de loop éxécutée qd int imu et data ok rythme 10 msec  et si  il ya du temps de libre
////////////////////////////////////////////////////////////////// UI monitor  protocole Dcmd argF  ////////////////
//// éventuellement mettre ds loop partie 1 qd attend int et data , partie déclenchée par int IMU ttes les 5 msec

//POUR PID ROLL TAUX ANGULAIRE
if (command=="RTP"){ // fixe new KP
  RtaKp = argument.toFloat(); // conversion string to float set ds globale
  // un float se met à la place d'un double
    roll_taux_angle_pid.SetTunings(RtaKp,RtaKi,RtaKd);
   };
if (command=="RTI"){ // fixe new KI
  RtaKi = argument.toFloat(); // conversion string to float
  // un float se met à la place d'un double
    roll_taux_angle_pid.SetTunings(RtaKp,RtaKi,RtaKd);
   };
if (command=="RT3"){ // fixe new KD
  RtaKd = argument.toFloat(); // conversion string to float
  // un flota se met à la place d'un double
    roll_taux_angle_pid.SetTunings(RtaKp,RtaKi,RtaKd);
   };
if (command=="RTE"){ // fixe new échantillonage
  sampleTimeRTA = argument.toInt(); // conversion string to int
    roll_taux_angle_pid.SetSampleTime(sampleTimeRTA);
   };
if (command=="RTS"){ // fixe new sens direction
  sensCorrectionRTA = argument.toInt(); // conversion string to int
    roll_taux_angle_pid.SetControllerDirection(sensCorrectionRTA);
   };
if (command=="RTL"){ // fixe new output limits
  Roll1outPutLimits = argument.toFloat();  // conversion string to float
  // un float se met à la place d'un double
    roll_taux_angle_pid.SetOutputLimits(Roll1outPutLimits*-1,Roll1outPutLimits);
   };

if (command=="RTR"){ // RESET PID to start
  sampleTimeRTA = SAMPLE_TIMERTA;   // restore defaults
  RtaKp=ROLL_TANGLEX_KP,  RtaKi=ROLL_TANGLEX_KI, RtaKd=ROLL_TANGLEX_KD; 
  sensCorrectionRTA = DIRECT;
  Roll1outPutLimits = 200;
  initPID();
   };

// POUR PID ROLL ACCELERATION
   if (command=="RAP"){ // fixe new KP
  RacKp = argument.toFloat(); // conversion string to float set ds globale
  // un float se met à la place d'un double
    roll_accel_pid.SetTunings(RacKp,RacKi,RacKd);
   };
if (command=="RAI"){ // fixe new KI
  RacKi = argument.toFloat(); // conversion string to float
  // un float se met à la place d'un double
    roll_accel_pid.SetTunings(RacKp,RacKi,RacKd);
   };
if (command=="RA3"){ // fixe new KD
  RacKd = argument.toFloat(); // conversion string to float
  // un flota se met à la place d'un double
    roll_accel_pid.SetTunings(RacKp,RacKi,RacKd);
   };
if (command=="RAE"){ // fixe new échantillonage
  sampleTimeRAC = argument.toInt(); // conversion string to int
    roll_accel_pid.SetSampleTime(sampleTimeRAC);
   };
if (command=="RTS"){ // fixe new sens direction
  sensCorrectionRAC = argument.toInt(); // conversion string to int
    roll_accel_pid.SetControllerDirection(sensCorrectionRAC);
   };
if (command=="RAL"){ // fixe new output limits
  Roll2outPutLimits = argument.toFloat();  // conversion string to float
  // un float se met à la place d'un double
    roll_accel_pid.SetOutputLimits(Roll2outPutLimits*-1,Roll2outPutLimits);
   };
if (command=="RAR"){ // RESET PID to start
  sampleTimeRAC = SAMPLE_TIMERAC;   // restore defaults
  RacKp=ROLL_ACCEL_KP,  RacKi=ROLL_ACCEL_KI, RacKd=ROLL_ACCEL_KD; 
  sensCorrectionRTA = DIRECT;
  Roll1outPutLimits = 200;
  initPID();
   };










if (command=="STOP"){ // stop motors
  StopMotors();
      };

  command="";    //re init
  argument="";

 
}  // fin de loop


void pulseDurationToAngles() {
  int consigneAsInteger = 0;
  if(channel1Changed) {
  pulse_duration[CHANNEL1] = constrain(pulse_duration[CHANNEL1], RC_CH1_IN_MIN, RC_CH1_IN_MAX );  // élimine les écarts trop importants
  pulse_duration[CHANNEL1] = map(pulse_duration[CHANNEL1] , RC_CH1_IN_MIN, RC_CH1_IN_MAX , 1000 ,2000); //pulse_duration[CHANNEL1] normalisé
 //   Serial.print(pulse_duration[CHANNEL1]);Serial.print("\t");

  if((pulse_duration[CHANNEL1] > 1480) && (pulse_duration[CHANNEL1] < 1520)) {pulse_duration[CHANNEL1] = 1500 ;}; // création de la zone morte
  consigneAsInteger = map(pulse_duration[CHANNEL1] , 1000 , 2000 , RC_CH1_OUT_MAX , RC_CH1_OUT_MIN ); //consigne en 16e de degré
//    Serial.print(consigneAsInteger);Serial.print("\t");

  consigneROLL = (float)consigneAsInteger / 16 ; //conversion Consigne en degrés dans un float
//    Serial.println(consigneROLL);
  channel1Changed = false;
  };
  
    
  pulse_duration[CHANNEL3] = constrain(pulse_duration[CHANNEL3], RC_CH3_IN_MIN, RC_CH3_IN_MAX );
  consigneGAZ = map(pulse_duration[CHANNEL3] , RC_CH3_IN_MIN , RC_CH3_IN_MAX , RC_CH3_OUT_MIN , RC_CH3_OUT_MAX );


  pulse_duration[CHANNEL5] = constrain(pulse_duration[CHANNEL5], RC_CH5_IN_MIN, RC_CH5_IN_MAX );
  consigneETAT = map(pulse_duration[CHANNEL5] , RC_CH5_IN_MIN , RC_CH5_IN_MAX , RC_CH5_OUT_MIN , RC_CH5_OUT_MAX ); 
}

///// routine d'interruption RX
ISR(PCINT0_vect)
{
    current_time = micros();

    // Channel 1 -------------------------------------------------
    if (PINB & B00000001) {                                        // Is input 8 high ?
        if (previous_state[CHANNEL1] == LOW) {                     // Input 8 changed from 0 to 1 (rising edge)
            previous_state[CHANNEL1] = HIGH;                       // Save current state
            timer[CHANNEL1]          = current_time;               // Start timer
        }
    } else if(previous_state[CHANNEL1] == HIGH) {                  // Input 8 changed from 1 to 0 (falling edge)
        previous_state[CHANNEL1] = LOW;                            // Save current state
        pulse_duration[CHANNEL1] = current_time - timer[CHANNEL1]; // Stop timer & calculate pulse duration     
        channel1Changed = true;
    };

    // Channel 3 -------------------------------------------------
    if (PINB & B00000100) {                                        // Is input 10 high ?
        if (previous_state[CHANNEL3] == LOW) {                     // Input 10 changed from 0 to 1 (rising edge)
            previous_state[CHANNEL3] = HIGH;                       // Save current state
            timer[CHANNEL3]          = current_time;               // Start timer
        }
    } else if(previous_state[CHANNEL3] == HIGH) {                  // Input 10 changed from 1 to 0 (falling edge)
        previous_state[CHANNEL3] = LOW;                            // Save current state
        pulse_duration[CHANNEL3] = current_time - timer[CHANNEL3]; // Stop timer & calculate pulse duration
        channel3Changed = true;
    };

      // Channel 5 -------------------------------------------------
    if (PINB & B00010000) {                                        // Is input 12 high ?
        if (previous_state[CHANNEL5] == LOW) {                     // Input 12 changed from 0 to 1 (rising edge)
            previous_state[CHANNEL5] = HIGH;                       // Save current state
            timer[CHANNEL5]          = current_time;               // Start timer
        }
    } else if(previous_state[CHANNEL5] == HIGH) {                  // Input 12 changed from 1 to 0 (falling edge)
        previous_state[CHANNEL5] = LOW;                            // Save current state
        pulse_duration[CHANNEL5] = current_time - timer[CHANNEL5]; // Stop timer & calculate pulse duration
        channel5Changed = true;
    };
    /*
   int pulse = 0;
   if (pulse_duration[4] < 1600){ 
      pulse = 0;} else {
      pulse = 1;};
     
     Serial.println (pulse);
*/
     
}

///// routine d'interruption IMU
void dmpDataReady() {
    mpuInterrupt = true;
}

void initRXread() {
 PCICR  |= (1 << PCIE0);  //Set PCIE0 to enable PCMSK0 scan.
    PCMSK0 |= (1 << PCINT0); //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
    PCMSK0 |= (1 << PCINT2); //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
    PCMSK0 |= (1 << PCINT4); //Set PCINT4 (digital input 12)to trigger an interrupt on state change.     
}

void initImu() {
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties      
      mpu.initialize();
      pinMode(INTERRUPT_PIN, INPUT);
      Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
      devStatus = mpu.dmpInitialize();
      mpu.setXGyroOffset(GYRO_X_OFFSET);
      mpu.setYGyroOffset(GYRO_Y_OFFSET);
      mpu.setZGyroOffset(GYRO_Z_OFFSET);
      mpu.setXAccelOffset(ACC_X_OFFSET);
      mpu.setYAccelOffset(ACC_Y_OFFSET);
      mpu.setZAccelOffset(ACC_Z_OFFSET);

      if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        // enable Arduino interrupt detection
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        } else {   // ERROR!
          // 1 = initial memory load failed
          // 2 = DMP configuration updates failed
          // (if it's going to break, usually the code will be 1)
          Serial.print(F("DMP Initialization failed (code "));
          Serial.print(devStatus);
          Serial.println(F(")"));
                }// fin else
} // fin initImu

void initVars(){
  for (int i = 0;i<64; i++){
    fifoBuffer[i] = 0;
  } ;
}

void initPID()
{ // set les propriétés initiales du PID à partir des défauts 
  ///////////// +++++++++++++++++++++++++++++++++++++++++++++++CAS ROLL
 // PID taux angulaire
  roll_taux_angle_pid.SetMode(AUTOMATIC);
  roll_taux_angle_pid.SetSampleTime(sampleTimeRTA);
  roll_taux_angle_pid.SetTunings(RtaKp,RtaKi,RtaKd,P_ON_E);    // 
  roll_taux_angle_pid.SetControllerDirection(sensCorrectionRTA);
  roll_taux_angle_pid.SetOutputLimits(-Roll1outPutLimits,Roll1outPutLimits); 

 // PID acceleration
  roll_accel_pid.SetMode(AUTOMATIC);
  roll_accel_pid.SetSampleTime(sampleTimeRAC);
  roll_accel_pid.SetTunings(RacKp,RacKi,RacKd,P_ON_E);    // 
  roll_accel_pid.SetControllerDirection(sensCorrectionRAC);
  roll_accel_pid.SetOutputLimits(-Roll2outPutLimits,Roll2outPutLimits); 

  
    ///////////// +++++++++++++++++++++++++++++++++++++++++++++++CAS ROLL
}

void StopMotors(){
   PORTD |= B11110000;            //Set digital port 4, 5, 6 and 7 high.
    delayMicroseconds(1000);       //Wait 1000us.
    PORTD &= B00001111;            //Set digital port 4, 5, 6 and 7 low.
}

void startESC(){
    motA.attach(4, 1000, 2000);
    motB.attach(5, 1000, 2000);
//indispensable pour signaler à l'ESC que un démarrage "normal" va se faire
//après on peut mettre le courant en route (connecter la batterie) 
     
     Serial.println(" TURN LIPO ON !!!: 10 sec pour le faire !!!");
     delay (5000);
     motA.writeMicroseconds(1000); // pour arrêter les bips
     motB.writeMicroseconds(1000);
     delay (5000);
     motA.detach(); // pour libérer les broches de la classe SERVO et libérer le timer millis() 
     motB.detach(); // pour libérer les broches de la classe SERVO et libérer le timer millis()
}

void computeESC() {  // maitien d'assiette sans changer l'altitude : Mode Auto level
int correctionROLL = Roll2_PID_output  ;
// 
 ESC_A = consigneGAZ + correctionROLL;
if(ESC_A < 1090){ESC_A = 1090;};
if(ESC_A > 2000){ESC_A = 2000;};
 ESC_B = consigneGAZ - correctionROLL;  
if(ESC_B < 1090){ESC_B = 1090;};
if(ESC_B > 2000){ESC_B = 2000;};
};

void commandAllMotors()   // créer le pulse de la largeur décidée par les PID Mixés pour chaque ESC
{
  //La cadence de la RC est 50Hz: Il est donc inutile de cadencer autrement les ESC puisque les nouvelles consignes ne peuvent arriver plus vite
  //D'où: :les ESC ont besoin d'un pulse toutes les 20 msec maxi mais vu la position de cette fonction après venue et traitement Int IMU 
  //la cde moteur se fait au rythme nouvelle donnée FIFO cad 100 Hz; donc sur 2 consignes identiques mais 2 mesures différentes
  loop_timer = micros();                                                    //Set the timer for the next loop.

  PORTD |= B11110000;                                                       //Set digital outputs 4,5,6 and 7 high.
  timerMotA = ESC_A + loop_timer;                                     //Calculate the time of the falling edge of the esc-A pulse.
  timerMotB = ESC_B + loop_timer;                                     //Calculate the time of the falling edge of the esc-B pulse.

//pour garder PORTD >= 16 .... force les pins 6 et 7 à low
   PORTD &= B10111111;                                               //Set digital output 6 to low
   PORTD &= B01111111;                                               //Set digital output 7 to low  
    
  while(PORTD >= 16){                                                 //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                        //Read the current time.
    if(timerMotA <= esc_loop_timer) {
          PORTD &= B11101111;  //Set digital output 4 to low if the time is expired.
                };                    
    if(timerMotB <= esc_loop_timer){
          PORTD &= B11011111;                //Set digital output 5 to low if the time is expired.
              };
                  } ;// fin while
};

// ------------------------------Moniteur de dialogue "Dcmd argumentRC------------------------------
void serialEvent() {
  while (Serial.available()) {
  char inChar = (char)Serial.read();  // récupérer le prochain octet (byte ou char) et l'enlever
  inputString += inChar;              // concaténation des octets reçus
      if (inChar == '\n') {       // caractère de fin pour notre chaine
        stringComplete = true;                         }
  }
// seconde partie, le dialogue est terminé, il faut analyser ce qui est dit: "analyse syntaxique: to PARSE en aglais"
// vérifs: si string complet & commence par 'D' => analyse, sinon sortie directe sans rien garder
// c'est une analyse minimale ...
 if (stringComplete){
    inputString.toUpperCase() ;    // tout en majuscules
 //       Serial.print("inputString =   ");
 //       Serial.println(inputString); 
        int indexD= inputString.indexOf('D');
        int indexSpace= inputString.indexOf(' ');
       command=inputString.substring((indexD+1),indexSpace);
 //     Serial.print("command="); Serial.println(command);  
             argument=inputString.substring(indexSpace+1);
  //    Serial.print("argument=");    
   //   Serial.println(argument);  
      
      inputString = ""; 
      stringComplete = false;
                    }        
}
