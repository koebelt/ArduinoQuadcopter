/////////  Expérience : ETUDE D'ASSERVISSEMT PID POUR DRONE  
////////   OBJECTIF : étude du CTR de Vol par PID sans pbs de LOOP Arduino interrompue par RC; cad rythme PID est rythme DMP
/////////  sans RC: Consignes directes par MAC IHM en liaison série
/////////  imitant RXsérie 3 canaux en angles (Yaw Roll Pitch); 1 canal en µs (GAZ), 2 canaux en 0/1 pour mode de vol et URGENCE
/////////  les  calibrations et emplacement IMU sont ceux de la bascule
/////////  l'altitude initiale ( AVANT stabilisation) est donnée par la consigne GAZ de 1200 µs initiale qui s'ajoute aux corrections de vol
/////////  Stabilisation étudiée sur 1 cascades de PID: cad PID-Angle qui donne une consigne "rate" au PID-Rate qui aliment un ESC 
/////////  qui fonctionne par accélération sur 1 seule cascade ...ROLL

/////////  Squelette d'un Controlleur de vol d'un quadricopter basé sur arduino UNO
/////////  Centrale inertielle :MPU6050 :Utilisation du DMP en interruption et lecture du FIFO par le DMP
/////////  Détermine les mesures en angles par float à 100 Hz

/////////  ATTENTION: au départ les conditions initiales sont 
/////////                 gx =0 / consigneROLL = 0.0 / consigneGAZ = 1000; / consigneETAT = UNARMED / consigneURGENCE = FLYING
/////////  ET la bascule de préférence à plat pour avoir une mesure initiale d'angle 0  
/////////  POUR DEMARRER il faut avoir le moniteur en route ; répondre à la demande de branchement LIPO et quand les esc ont fait leur selfTest
/////////  l'IMU démarre et se signale; puis les PID démarrent leur initialisation standartd et la loop commnence
/////////  elle part sur l'int IMU, sort immédiatement et éxécute l'IHM des commandes. Si celui)ci ne spécifie encore rien, la loop reprend sur l'int FIFO OK 
/////////  et les PID asservissent sur les consignes et mesure initiales
/////////  puis toutes les 10 ms l'asservissement recommence à corriger
/////////  il faut alors taper des commandes: d'abord ARM pour lancer les moteurs ( env 1200 µs) sans correction; puis VOL pour passer en stabilisation d'assiette
/////////  puis des consignes diverses qui sont appliquées en stabilisation ... 
/////////  et des TESTS et ajustages par des commandes de changement des paramètres ... ( ajouter aussi des print pour voir (des courbes))
/////////  !!! la commande STOP coupe les moteurs !!!!

/////////  A VOIR ????.... zône morte centrale équivalente à +-20µs ????


#include    "Config.h"
#include    "I2Cdev.h"
#include    "MPU6050_6Axis_MotionApps20.h"
#include    "PID_v1.h"
#include    "Servo.h"

//////////  =====================================  concerne l'IMU  =============================  //////////////

MPU6050       mpu(0x68);//instantiation MPU a l'adresse I2C 0x68

bool          dmpReady = false;  // set true if DMP init was successful
uint8_t       mpuIntStatus = 0;   // holds actual interrupt status byte from MPU
uint8_t       devStatus = 1;      // return status after each device operation (0 = success, !0 = error)
uint16_t      packetSize = 0;    // expected DMP packet size (default is 42 bytes)
uint16_t      fifoCount = 0;     // count of all bytes currently in FIFO
uint8_t       fifoBuffer[64]; // FIFO storage buffer
Quaternion    q;           // [w, x, y, z]         quaternion container
VectorFloat   gravity;    // [x, y, z]            gravity vector
int16_t       gx, gy, gz; // raw gyro rates
VectorInt16   gyroRates;   // [x, y, z]  Gyro sensor measurements par DMP

float         yrp[3] {0.0,0.0,0.0};           // [yaw, roll, pitch]   yaw/pitch/roll container as radians
float         mesureROLL = 0.0; //en degrés >0 et <0
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

//////////  =====================================  concerne le vol  =============================  //////////////
float         consigneROLL = 0.0; //en degrés >0 et <0
int           consigneGAZ = 1000; //en microsecondes de 1060µs à 1800µs
int           consigneETAT = UNARMED;
int           consigneURGENCE = FLYING;

/////  =====================================  concerne PID  =============================  //////////////
//////////  Dénomination: le PID est nommé selon sa fonction et sa sortie =>ROLL_TAUX_ANGLE_PID   ROLL_ACCEL_PID   
//////////  =====================================  concerne l'entrée des PID  =============================  //////////////

///////////////////////////////////////////////// PID ROLL_TAUX_ANGLE_PID <=> sortie accélération  PID externe//////////////////////////////////
double        Roll1_PID_input = 0.0;  // entrée PID
double        Roll1_PID_setpoint = 0.0; //consigne PID
double        Roll1_PID_output = 0.0; // sortie PID
int           sampleTimeRTA = SAMPLE_TIMERTA; // échantillonnage valeur par défaut
double        RtaKp=ROLL_TANGLEX_KP,  RtaKi=ROLL_TANGLEX_KI, RtaKd=ROLL_TANGLEX_KD; // valeurs par defaut: 2 0 0
int           RtaPOn = P_ON_E ;  // mode du PID P_ON_E 1 ou P_ON_M 0 valeur par défaut
int           sensCorrectionRTA = DIRECT; //valeur par défaut
// reverse : erreur >0 -> sortie PID <0      
double        Roll1OutPutLimits = 200; // valeur par défaut, sera fixé en outPutLimits*-1 .. outPutLimitsboolean     
boolean       roll_taux_angle_pidOK = false;

// instanciation objet ROLL
PID           ROLL_TAUX_ANGLE_PID (&Roll1_PID_input, &Roll1_PID_output, &Roll1_PID_setpoint, RtaKp, RtaKi, RtaKd, RtaPOn, sensCorrectionRTA);

///////////////////////////////////////////////// ROLL_ACCEL_PID <=> sortie Taux angulaire  PID interne//////////////////////////////////

// PID ROLL acceleration

double        Roll2_PID_input = 0.0;
double        Roll2_PID_setpoint = 0.0;
double        Roll2_PID_output = 0.0;
int           sampleTimeRAC = SAMPLE_TIMERAC; // échantillonnage valeur par défaut
double        RacKp=ROLL_ACCEL_KP,  RacKi=ROLL_ACCEL_KI, RacKd=ROLL_ACCEL_KD; // valeurs par defaut: 2 0 0
int           RacPOn = P_ON_E ;  // mode du PID P_ON_E 1 ou P_ON_M 0 valeur par défaut
int           sensCorrectionRAC = DIRECT; //valeur par défaut
double        Roll2OutPutLimits = 400; // valeur par défaut, sera fixé en outPutLimits*-1 .. outPutLimits
boolean       roll_accel_pidOK = false;

PID           ROLL_ACCEL_PID (&Roll2_PID_input, &Roll2_PID_output, &Roll2_PID_setpoint, RacKp, RacKi, RacKd, RacPOn, sensCorrectionRAC);

//////////  =====================================  concerne Moteurs  =============================  //////////////
Servo         motA, motB; // instanciation   pour la séquence de démarrage
unsigned long ESC_A = 0;
unsigned long ESC_B = 0;
unsigned long loop_timer, timerMotA,timerMotB,esc_loop_timer;

//////////  =====================================  concerne l'UI dialogue  =============================  //////////////
String        inputString=""; // chaine de caractères pour contenir les données
boolean       stringComplete=false ; // pour savoir si la chaine est complète
String        command="";    // init
String        argument="";   // init

//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void setup() {
  initVars();
  Serial.begin(115200);
  startESC(); // utilise classe SERVO pour envoyer pulse start puis detach: SERVO restera inutilisé par après
  configPins();
  initImu();
  initPID();
  };
// ============================================================================================================

void pid_loop()
{
  if (!dmpReady) {return;};  //Si il n'y a pas d'interruptions donc sortir de cette loop: PLUS RIEN ne se passe

  // wait for MPU interrupt or extra packet(s) available
  if (!mpuInterrupt && fifoCount < packetSize) {return;};
// si il n'y a pas d'interruptions mpu ET que le FIFO n'est pas rempli cad pas de données brutes ni de donnée traitées par DMP : cad pas de mesure valable
// NE concerne pas l'asservissement   = sortir
        
////////////             maintenant il y a interruption >> réagir : préparer la prochaine int et vérifier !!: int OK? ET FIFO données OK ?
   // reset interrupt flag and get INT_STATUS byte
     mpuInterrupt = false;
     mpuIntStatus = mpu.getIntStatus();
     fifoCount = mpu.getFIFOCount(); // get current FIFO count
     if ((mpuIntStatus & 0x10) || fifoCount == 1024) {// check for overflow (this should never happen unless our code is too inefficient)
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;  // INUTILISE
////////////////////////////////////////////////////////  =====================================================  //////////////////////////////////
///////////////               tout est OK il faut utiliser ce qui est dans le FIFO 
        mpu.dmpGetQuaternion(&q, fifoBuffer);  // mpu dit à DMP de lire la partie quaternion contenue ds le FIFO  et mettre dans q
        mpu.dmpGetGravity(&gravity, &q);  //mpu dit a DMP de calculer la gravitée a partir du quaternion et mettre ds gravity
        mpu.dmpGetYawRollPitch(yrp, &q, &gravity);    //mpu dit a dmp de calculer ypr a partir de la gravitée et du quaternion et de mettre dans yrp
                                                      // ordre YAW PITCH ROLL en est le résultat : les angles en RADIANS
//////////////// l'imu montée sur la bascule commute r et p du fait de son montage mécanique; donc rectifications et sortie en degrés                                                                 
        mesureROLL =  0-(yrp[2] * radToDegrees);  // on veut ROLL qui est le dernier cad le second, axe de roll est y     
        mpu.dmpGetGyro(&gyroRates, fifoBuffer);// choix: utiliser dmp et fifo: synchronise meure roll angle et roll taux
        gx = gyroRates.x;
        gy = gyroRates.y; // seul utilisé pour roll
        gz = gyroRates.z;
        /*
Serial.print("gyroRates\t"); Serial.print("axeX\t"); Serial.print(gyroRates.x); Serial.print("\t"); Serial.print("axeY\t"); Serial.print(gyroRates.y);
Serial.print("\t"); Serial.print("axeZ\t"); Serial.println(gyroRates.z);
            */
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////  ================================ consignes et mesures en angles sont disponibles  ============================= 
////////////////  lancer l'asservissement //////////////////////////////////////////////////////////////////////////////////////////////////////       
        
        //Compute Roll PID taux angulaire
        Roll1_PID_setpoint = consigneROLL; // consigne ==setPoint
        Roll1_PID_input = mesureROLL; // entrée == mesure
        roll_taux_angle_pidOK = ROLL_TAUX_ANGLE_PID.Compute();  // resultat ds pid_output borné à  +-200 (défaut)
        //Compute Roll PID sortie en accélération vers ESC
        Roll2_PID_setpoint = Roll1_PID_output;    // LA CONSIGNE EST LA SORTIE du PID précédent qui décide par la consigne de son 'opérateur
        Roll2_PID_input = gy;  // l'entrée est la mesure, cad le taux de rotation selon axe y donné par IMU
        roll_accel_pidOK = ROLL_ACCEL_PID.Compute();  // resultat ds pid_output borné à  +-200 (défaut)  

        if(roll_taux_angle_pidOK && roll_accel_pidOK){ //si l'asservissement est OK Afficher des données  MIXER
///Serial.print( mesureROLL);Serial.print("\t");Serial.print (consigneROLL); Serial.print("\t");Serial.print (Roll1_PID_output);Serial.print("\t");Serial.println(Roll2_PID_output);
           computeESC();  // calculer les données ESC à appliquer (MIX)
          }; 
        // Lancer les moteurs pour un décollage 
        if (consigneETAT == ARMED){   // ATTENTION n'est valable que pour la bascule: pour le drone il faut asservir la hauteur et enchaîner de suite sur stabilize
         startMotors();   // moteurs passent en décollage suffisant pour que stabilisation fonctionne (+200 µs)
            };
        // éxécuter la stabilisation d'assiette
        if (consigneETAT == STABILIZE){   // c'est un fonctionnement TOTALEMENT MANUEL  !!! à tester en essayant de bouger à la main la bascule qui doit résister
         commandAllMotors();
            };

    }; // fin du else mpuIntStatus cad tous data prêts et donc actions de corrections
 }; // fin de pid_loop
  
void loop() {
      
  pid_loop();
     
/////////////////// ===================================================   IHM Moniteur de comandes  =================================  ///////////////
// ============================================= ds seconde partie de loop éxécutée qd int imu et data ok rythme 10 msec 
////////////////////////////////////////////////////////////////// UI monitor  protocole Dcmd argF  ////////////////
//// éventuellement mettre ds loop partie 1 qd attend int et data , partie déclenchée par int IMU ttes les 5 msec
/////////////////// ===================================================   IHM Moniteur de comandes  =================================  ///////////////
                                                  //  COMMANDES DE VOL
if (command=="ARM"){ // démarre motors
  consigneETAT = ARMED;
  consigneURGENCE = FLYING;
      };     
if (command=="VOL"){ // démarre motors
  consigneETAT = STABILIZE;
  consigneURGENCE = FLYING;
      };  
if (command=="STOP"){ // stop motors
  consigneETAT = UNARMED;
  consigneURGENCE = EMERGENCY;
      };     
if (command=="ROLL"){ // incliner par roll  remplace la manette
  float value = argument.toFloat();
  consigneROLL = value;
      }; 
if (command=="GAZ"){ //   remplace la manette
  int value = argument.toInt();
  consigneGAZ = value;
      };
          
                                                    //POUR PID ROLL TAUX ANGULAIRE  <=> PID externe
if (command=="RTP"){ // fixe new KP
  RtaKp = argument.toFloat(); // conversion string to float set ds globale
  // un float se met à la place d'un double
    ROLL_TAUX_ANGLE_PID.SetTunings(RtaKp,RtaKi,RtaKd);
   };
if (command=="RTI"){ // fixe new KI
  RtaKi = argument.toFloat(); // conversion string to float
  // un float se met à la place d'un double
    ROLL_TAUX_ANGLE_PID.SetTunings(RtaKp,RtaKi,RtaKd);
   };
if (command=="RT3"){ // fixe new KD
  RtaKd = argument.toFloat(); // conversion string to float
  // un flota se met à la place d'un double
    ROLL_TAUX_ANGLE_PID.SetTunings(RtaKp,RtaKi,RtaKd);
   };
if (command=="RTE"){ // fixe new échantillonage
  sampleTimeRTA = argument.toInt(); // conversion string to int
    ROLL_TAUX_ANGLE_PID.SetSampleTime(sampleTimeRTA);
   };
if (command=="RTS"){ // fixe new sens direction
  sensCorrectionRTA = argument.toInt(); // conversion string to int
    ROLL_TAUX_ANGLE_PID.SetControllerDirection(sensCorrectionRTA);
   };
if (command=="RTL"){ // fixe new output limits
  Roll1OutPutLimits = argument.toFloat();  // conversion string to float
  // un float se met à la place d'un double
    ROLL_TAUX_ANGLE_PID.SetOutputLimits(Roll1OutPutLimits*-1,Roll1OutPutLimits);
   };
if (command=="RTM"){ // fixe mode PID
  RtaPOn = argument.toInt(); // conversion string en int; 
    ROLL_TAUX_ANGLE_PID.SetTunings(RtaKp,RtaKi,RtaKd,RtaPOn);
   };
if (command=="RTR"){ // RESET PID to start
  sampleTimeRTA = SAMPLE_TIMERTA;   // restore defaults
  RtaKp=ROLL_TANGLEX_KP,  RtaKi=ROLL_TANGLEX_KI, RtaKd=ROLL_TANGLEX_KD; 
  RtaPOn = P_ON_E; 
  sensCorrectionRTA = DIRECT;
  Roll1OutPutLimits = 200;
  initPID();
   };

                                                // POUR PID ROLL ACCELERATION <=> PID Interne
   if (command=="RAP"){ // fixe new KP
  RacKp = argument.toFloat(); // conversion string to float set ds globale
  // un float se met à la place d'un double
    ROLL_ACCEL_PID.SetTunings(RacKp,RacKi,RacKd);
   };
if (command=="RAI"){ // fixe new KI
  RacKi = argument.toFloat(); // conversion string to float
  // un float se met à la place d'un double
    ROLL_ACCEL_PID.SetTunings(RacKp,RacKi,RacKd);
   };
if (command=="RA3"){ // fixe new KD
  RacKd = argument.toFloat(); // conversion string to float
  // un flota se met à la place d'un double
    ROLL_ACCEL_PID.SetTunings(RacKp,RacKi,RacKd);
   };
if (command=="RAE"){ // fixe new échantillonage
  sampleTimeRAC = argument.toInt(); // conversion string to int
    ROLL_ACCEL_PID.SetSampleTime(sampleTimeRAC);
   };
if (command=="RAS"){ // fixe new sens direction
  sensCorrectionRAC = argument.toInt(); // conversion string to int
    ROLL_ACCEL_PID.SetControllerDirection(sensCorrectionRAC);
   };
if (command=="RAL"){ // fixe new output limits
  Roll2OutPutLimits = argument.toFloat();  // conversion string to float
  // un float se met à la place d'un double
    ROLL_ACCEL_PID.SetOutputLimits(Roll2OutPutLimits*-1,Roll2OutPutLimits);
   };
if (command=="RAM"){ // fixe mode PID
  RacPOn = argument.toInt(); // conversion string en int; 
    ROLL_ACCEL_PID.SetTunings(RacKp,RacKi,RacKd,RacPOn);
   };
if (command=="RAR"){ // RESET PID to start
  sampleTimeRAC = SAMPLE_TIMERAC;   // restore defaults
  RacKp=ROLL_ACCEL_KP,  RacKi=ROLL_ACCEL_KI, RacKd=ROLL_ACCEL_KD; 
  RacPOn = P_ON_E; 
  sensCorrectionRTA = DIRECT;
  Roll1OutPutLimits = 200;
  initPID();
   };

if (command=="STOP"){ // stop motors
  StopMotors();
      };

  command="";    //re init
  argument="";


 Serial.print(consigneGAZ);Serial.print("      ");Serial.print(consigneROLL);
 Serial.print("      ");Serial.print(Roll2_PID_output);Serial.print("      ");Serial.print(consigneETAT);Serial.print("       ");Serial.print(ESC_A);Serial.print("      ");Serial.println(ESC_B);
  
};  // fin de loop



void initVars(){
  for (int i = 0;i<64; i++){
    fifoBuffer[i] = 0;
  } ;
  for (int i=0; i<3; i++){
    yrp[i] = 0.0;
  }
  consigneROLL = 0.0; //en degrés >0 et <0
  consigneGAZ = 1000; //en microsecondes de 1060µs à 1800µs
  consigneETAT = UNARMED;
  consigneURGENCE = FLYING;
  
  gyroRates.x = 0.0;
  gyroRates.y = 0.0;
  gyroRates.z = 0.0; 
  
};

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
}; // fin initImu

void initPID()
{ // set les propriétés initiales du PID à partir des défauts 
  ///////////// +++++++++++++++++++++++++++++++++++++++++++++++CAS ROLL
 // PID taux angulaire
  ROLL_TAUX_ANGLE_PID.SetMode(AUTOMATIC);
  ROLL_TAUX_ANGLE_PID.SetSampleTime(sampleTimeRTA);
  ROLL_TAUX_ANGLE_PID.SetTunings(RtaKp,RtaKi,RtaKd,RtaPOn);    // 
  ROLL_TAUX_ANGLE_PID.SetControllerDirection(sensCorrectionRTA);
  ROLL_TAUX_ANGLE_PID.SetOutputLimits(-Roll1OutPutLimits,Roll1OutPutLimits); 

 // PID acceleration
  ROLL_ACCEL_PID.SetMode(AUTOMATIC);
  ROLL_ACCEL_PID.SetSampleTime(sampleTimeRAC);
  ROLL_ACCEL_PID.SetTunings(RacKp,RacKi,RacKd,RacPOn);    // 
  ROLL_ACCEL_PID.SetControllerDirection(sensCorrectionRAC);
  ROLL_ACCEL_PID.SetOutputLimits(-Roll2OutPutLimits,Roll2OutPutLimits); 
};

void StopMotors(){
   PORTD |= B11110000;            //Set digital port 4, 5, 6 and 7 high.
    delayMicroseconds(1000);       //Wait 1000us.
    PORTD &= B00001111;            //Set digital port 4, 5, 6 and 7 low.
};

void startESC(){
    motA.attach(4, 1000, 2000);
    motB.attach(5, 1000, 2000);
//indispensable pour signaler à l'ESC que un démarrage "normal" va se faire
//après on peut mettre le courant en route (connecter la batterie) 
     
     Serial.println(" TURN LIPO ON !!!: 5 sec pour le faire !!!");
     delay (5000);
     motA.writeMicroseconds(1000); // pour arrêter les bips
     motB.writeMicroseconds(1000);
     delay (5000);
     motA.writeMicroseconds(1000); // pour arrêter les bips
     motB.writeMicroseconds(1000);
     motA.detach(); // pour libérer les broches de la classe SERVO et libérer le timer millis() 
     motB.detach(); // pour libérer les broches de la classe SERVO et libérer le timer millis()
};

/*void computeESC() {  // maintien d'assiette sans changer l'altitude : Mode Auto level
int correctionROLL = Roll2_PID_output  ;
    if(consigneGAZ <= consigneGAZ + MOT_START_MINIMUM ) {
    consigneGAZ = consigneGAZ + MOT_START_MINIMUM;
    };
    
    if(consigneGAZ >= MOT_MAX - MOT_START_MINIMUM ) {
    consigneGAZ = MOT_MAX - MOT_START_MINIMUM ;
    };
    
    ESC_A = consigneGAZ + correctionROLL;
    ESC_B = consigneGAZ - correctionROLL;  
};*/
void computeESC() {  // maitien d'assiette sans changer l'altitude : Mode Auto level
int correctionROLL = Roll2_PID_output  ;
// 
 ESC_A = consigneGAZ - correctionROLL;
if(ESC_A < 1090){ESC_A = 1090;};
if(ESC_A > 2000){ESC_A = 2000;};
 ESC_B = consigneGAZ + correctionROLL ;  
if(ESC_B < 1103){ESC_B = 1103;};
if(ESC_B > 2000){ESC_B = 2000;};
};


void startMotors()   // créer le pulse de largeur suffisante pour décoller et autoriser la correction d'assiette: 1200 µs ??
{
  loop_timer = micros();                                                             //Set the timer for the next loop.
  PORTD |= B11110000;                                                                //Set digital outputs 4,5,6 and 7 high.
  timerMotA = MOT_START_MINIMUM + loop_timer;                          //Calculate the time of the falling edge of the esc-A pulse.
  timerMotB = MOT_START_MINIMUM + loop_timer + 13 ;  //Calculate the time of the falling edge of the esc-B pulse.

//pour garder PORTD >= 16 .... force les pins 6 et 7 à low
   PORTD &= B10111111;                                               //Set digital output 6 to low
   PORTD &= B01111111;                                               //Set digital output 7 to low  
    
  while(PORTD >= 16){                                                 //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                        //Read the current time.
    if(timerMotA <= esc_loop_timer) {
          PORTD &= B11101111;             //Set digital output 4 to low if the time is expired.sans toucher aux pins 6 et 7
                };                    
    if(timerMotB <= esc_loop_timer){
          PORTD &= B11011111;            //Set digital output 5 to low if the time is expired sans toucher aux pins 6 et 7
              };
                  } ;// fin while                 
};

void commandAllMotors()   // créer le pulse de la largeur décidée par les PID Mixés pour chaque ESC
{
  //la cde moteur se fait au rythme nouvelle donnée FIFO cad 100 Hz; donc ICI sur 2 consignes identiques mais 2 mesures différentes
  loop_timer = micros();                                              //Set the timer for the next loop.

  PORTD |= B11110000;                                                 //Set digital outputs 4,5,6 and 7 high.
  timerMotA = ESC_A + loop_timer;                                     //Calculate the time of the falling edge of the esc-A pulse.
  timerMotB = ESC_B + loop_timer;                                     //Calculate the time of the falling edge of the esc-B pulse.

//pour garder PORTD >= 16 .... force les pins 6 et 7 à low
   PORTD &= B10111111;                                               //Set digital output 6 to low
   PORTD &= B01111111;                                               //Set digital output 7 to low  
    
  while(PORTD >= 16){                                                //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                       //Read the current time.
    if(timerMotA <= esc_loop_timer) {
          PORTD &= B11101111;             //Set digital output 4 to low if the time is expired.sans toucher aux pins 6 et 7
                };                    
    if(timerMotB <= esc_loop_timer){
          PORTD &= B11011111;            //Set digital output 5 to low if the time is expired sans toucher aux pins 6 et 7
              };
                  } ;// fin while
};


///// routine d'interruption IMU
void dmpDataReady() {
    mpuInterrupt = true;
};


// ------------------------------Moniteur de dialogue "Dcmd argumentRC------------------------------
void serialEvent() {
  while (Serial.available()) {
  char inChar = (char)Serial.read();  // récupérer le prochain octet (byte ou char) et l'enlever
  inputString += inChar;              // concaténation des octets reçus
      if (inChar == '\n') {       // caractère de fin pour notre chaine
        stringComplete = true;                         }
  }
// seconde partie, le dialogue est terminé, il faut analyser ce qui est dit: "analyse syntaxique: to PARSE en anglais"
// vérifs: si string complet & commence par 'D' => analyse, sinon sortie directe sans rien garder
// c'est une analyse minimale ...
 if (stringComplete){
    inputString.toUpperCase() ;    // tout en majuscules
        Serial.print("inputString =   ");
        Serial.println(inputString); 
        int indexD= inputString.indexOf('D');
        int indexSpace= inputString.indexOf(' ');
       command=inputString.substring((indexD+1),indexSpace);
     Serial.print("command="); Serial.println(command);  
             argument=inputString.substring(indexSpace+1);
      Serial.print("argument=");    
      Serial.println(argument);  
    //  Serial.print(consigneETAT);
      inputString = ""; 
      stringComplete = false;
                    }        
}
