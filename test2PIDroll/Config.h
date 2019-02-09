/////////  Quadricopter composé de : Moteur A :Avant gauche, sens horraire           -ESC A :Pin 4 
/////////                          - Moreur B :Avant droit, sens anti horraire       -ESC B :Pin 5
/////////                          - Moteur C :Arriere gauche, sens anti horraire    -ESC C :Pin 6
/////////                          - Moteur D :Arriere droit:sens horraire           -ESC D :Pin 7
/////////  Radiocommande FS-T6 6 Cannaux : Canal 1 :Stick droit axe horizontal :Roll        RX réception : Canal 1 :Pin 8
/////////                                - Canal 2 :Stick droit axe vertical :Pitch         RX réception : Canal 2 :Pin 9
/////////                                - Canal 3 :Stick gauche axe vertical :Gaz          RX réception : Canal 3 :Pin 10
/////////                                - Canal 4 :Stick gauche axe horizontal :Yaw        RX réception : Canal 4 :Pin 11
/////////                                - Canal 5 :SwB :off==0 :Moteurs inactifs           RX réception : Canal 5 :Pin 12
/////////                                - Canal 5 :SwB :on==1 :Moteurs actifs
/////////                                - Canal 6 :SwA :off==0 :                           RX réception : Canal 6 :Pin 13
/////////                                - Canal 6 :SwA :on==1 :
// -----Other Config---------

#define ARMED     0
#define STABILIZE 1   // flag de mode de contrôle de vol


// ==========================================     RX PINS    =========================================================================
// define  le nom du canal sur la RC en un index de tableau
#define CHANNEL1 0      //ROLL
#define CHANNEL2 1     // PITCH
#define CHANNEL3 2     // GAZ
#define CHANNEL4 3     // YAW
#define CHANNEL5 4     // DRONE STATE

//-------RC_Channels Config---------

#define  RC_CH1_IN_MIN   992
#define  RC_CH1_IN_MAX   1984
#define  RC_CH1_OUT_MIN  -15 * 16 //l'unitée : 16e de degré
#define  RC_CH1_OUT_MAX  15 *  16

#define  RC_CH2_IN_MIN   992
#define  RC_CH2_IN_MAX   1988
#define  RC_CH2_OUT_MIN  -15 * 16 //l'unitée : 16e de degré
#define  RC_CH2_OUT_MAX  15 * 16

#define  RC_CH3_IN_MIN   992
#define  RC_CH3_IN_MAX   1988
#define  RC_CH3_OUT_MIN  1080  // Rotation minimale des hélices
#define  RC_CH3_OUT_MAX  1800 // laisse marge de 200 pour correction par PID

#define  RC_CH4_IN_MIN   992
#define  RC_CH4_IN_MAX   1988
#define  RC_CH4_OUT_MIN  -15 * 16 //l'unitée : 16e de degré
#define  RC_CH4_OUT_MAX  15 * 16

#define  RC_CH5_IN_MIN  1200
#define  RC_CH5_IN_MAX  1700
#define  RC_CH5_OUT_MAX 1
#define  RC_CH5_OUT_MIN 0

// define  la fonction d'un canal en un numéro équivalent
#define ROLL     2
#define THROTTLE 1

//==========================================   PID Config   =========================================================================
 #define P_ON_M 0
 #define P_ON_E 1
 
///////////////// ======================= CAS ROLL 
#define ROLL_TANGLEX_KP 2.0
#define ROLL_TANGLEX_KI 0.0
#define ROLL_TANGLEX_KD 0.0

#define ROLL_ACCEL_KP 1.0
#define ROLL_ACCEL_KI 0.0
#define ROLL_ACCEL_KD 0.0
///////////////// ======================= CAS ROLL 

#define PLAGE_MIN_ANGLE -15.0 *16  //  -15 degrés *16
#define PLAGE_MAX_ANGLE 15.0 * 16  //  15 degrés *16
#define SAMPLE_TIMERTA 3  //millisecondes
#define SAMPLE_TIMERAC 3  //millisecondes

//=======================================    IMU Config      =========================================================================
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno for extInt 0


#define ACC_X_OFFSET  632   // valeurs de l'IMU "flottant GM" valeurs par DEFAUT
#define ACC_Y_OFFSET  169
#define ACC_Z_OFFSET  885

#define GYRO_X_OFFSET  55
#define GYRO_Y_OFFSET  -11
#define GYRO_Z_OFFSET  -3

#define  radToDegrees  180.0/PI 

//======================================   MOTOR CONFIG    ==============================================================================

void configPins(){
  pinMode(INTERRUPT_PIN, INPUT); // set pin digital 2 to Interrupt mode
  //Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs.
  DDRD |= B11110000;       //Configure digital port 4, 5, 6 and 7 as output.
}




