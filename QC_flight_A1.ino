/* FLIGHT CODE FOR QUADDY MCQUADCOPTER (AKA H.M.S. DAN SISSON) */
// by Brad Danielson
// ***************************************************************
/* 
 *  THINGS TO RESEARCH:
 *  
 * Magnetic Declination Required?
 * 
 * 
 * 
 * 
 * 
 * 
*/ //**************************************************************

#include <PID_v1.h>
#include <i2c_t3.h>
#include <XYZ_BNO055.h>
#include <Motors.h>
#include <math.h>
#include <FlightControl.h>

/* General Definitions 
   * COMMENT OUT:
   * Serial  > if using Bluetooth device
   * Serial2 > if hardwired via USB using COM port
*/
#define BT         Serial
//#define BT       Serial2
/* */
//#define BTCS       Serial3 // Bluetooth with Charging Station **************************
//#define GPSserial  Serial1 // Rx/Tx with Emlid Reach ****************************
#define TimeIntervalMilliSeconds 10  // IMU Update Period Milliseconds 100 Hz
#define TimeInterval10HzTask 100 // GPS UpdatePeriod Milliseconds 10 Hz
#define ms2us 1000 // us per ms 
#define u2deltaPWM_R 1
#define u2deltaPWM_P 1
#define u2deltaPWM_Y -1 
#define PID_MAX_VALUE 100
#define PID_MAX_ANGLE 15 // Maximum Angle Roll and Pitch will be commanded
#define bitRes 65535 // PWM Bit Resolution
#define LEDPIN 13
#define displayPeriod 3 // seconds per debugging display print
#define deltaKP .01
#define deltaKI .0001
#define deltaKD .001
#define deltaBase 0.25
#define deltaSP 0.5
#define deg2rad 0.0174533
#define magneticDec 14.79 // Degrees


double OutputE, OutputN, OutputZ ;
double InputE_cons = 0.0, InputN_cons = 0.0, InputZ; /* Zero (constants) due to rotateVector() algorithm */
double InputP = 0.0, OutputP ;
double InputR = 0.0, OutputR ;
double InputY, OutputY ; 
float basePWM = 0 ;
float ypr[3] ;
float rollOffset, pitchOffset, yawOffset;
double Roll_Setpoint = 0.0, Pitch_Setpoint = 0.0, Yaw_Setpoint, Yaw_Setpoint_cons = 180.0;
double N_Setpoint = 0.0, E_Setpoint = 0.0, Z_Setpoint;
double SpeedControl = 1.0, ZControl = 1.0  ;
int cnt = 0 ;
volatile int newDataIMU = FALSE, newDataGPS = FALSE ; /* Volatile Interrupt Variables */



/* Object Creation */
XYZ_BNO055 imu ;
PID PID_Z(&InputZ, &OutputZ, &Z_Setpoint, KpZ, KiZ, KdZ, DIRECT) ;
PID PID_N(&InputN_cons, &OutputN, &N_Setpoint, KpN, KiN, KdN, DIRECT) ;
PID PID_E(&InputE_cons, &OutputE, &E_Setpoint, KpE, KiE, KdE, DIRECT) ;
PID PID_P(&InputP, &OutputP, &Pitch_Setpoint, KpP, KiP, KdP, DIRECT);
PID PID_R(&InputR, &OutputR, &Roll_Setpoint, KpR, KiR, KdR, DIRECT);
PID PID_Y(&InputY, &OutputY, &Yaw_Setpoint_cons, KpY, KiY, KdY, DIRECT);
Motors motorControl ;
FlightControl fcontrol ;
IntervalTimer IMUUpdate ;
IntervalTimer TenHzTask ;
/* End Object Creation */


/* BEGIN SETUP */
void setup() {
  char go ;
  pinMode(LEDPIN, OUTPUT) ;
  /* Setup The BNO055, Serial, and I2C ports */
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);
  delay(1500) ;
  BT.begin(115200) ;
  delay(10) ;
  //BTCS.begin(115200) ; 
  /* ************************* */
  delay(10) ;
  //GPSserial.begin(38400) ;
  /* *************************************** */
  delay(10) ;
  Serial.begin(115200) ;
  delay(150) ;
  BT.println("BNO055 TEST");
  while (!imu.setup(BNO055_ADDRESS_B))
  {
    BT.println("No BNO055 found");
    delay(100);
  }
  BT.println("BNO055 found") ;
  delay(1000) ;
  imu.setMode(XYZ_BNO055::NDOF) ;
  calibrateIMU();
  BT.println("IMU Calibration Success");
  delay(1000);
  digitalWrite(LEDPIN,HIGH);
  /* End IMU/BT setup */
  
  /* Setup The Motors */
  basePWM = 40.0 ;
  motorControl.initMotors() ;
  /* End Motor Setup */

   /* Setup The PID */
  // Z AXIS
  PID_Z.SetMode(AUTOMATIC) ;
  PID_Z.SetOutputLimits(+10, -10) ;
  PID_Z.SetSampleTime(TimeInterval10HzTask) ;
  PID_Z.SetTunings(KpZ, KiZ, KdZ) ;
  
  /* PID_MAX_ANGLE is 10 degrees */
  //N (NORTH/SOUTH AXIS) N = -S, E X N = Z
  PID_N.SetMode(AUTOMATIC);
  PID_N.SetOutputLimits(-PID_MAX_ANGLE, PID_MAX_ANGLE);
  PID_N.SetSampleTime(TimeInterval10HzTask);
  PID_N.SetTunings(KpN, KiN, KdN);

  //E (EAST/WEST AXIS) E = -W
  PID_E.SetMode(AUTOMATIC);
  PID_E.SetOutputLimits(-PID_MAX_ANGLE, PID_MAX_ANGLE);
  PID_E.SetSampleTime(TimeInterval10HzTask);
  PID_E.SetTunings(KpE, KiE, KdE);
  
  //Pitch
  PID_P.SetMode(AUTOMATIC);
  PID_P.SetOutputLimits(-PID_MAX_VALUE, PID_MAX_VALUE);
  PID_P.SetSampleTime(TimeIntervalMilliSeconds);
  PID_P.SetTunings(KpP, KiP, KdP);

  //Roll
  PID_R.SetMode(AUTOMATIC);
  PID_R.SetOutputLimits(-PID_MAX_VALUE, PID_MAX_VALUE);
  PID_R.SetSampleTime(TimeIntervalMilliSeconds);
  PID_R.SetTunings(KpR, KiR, KdR);

  //Yaw
  PID_Y.SetMode(AUTOMATIC);
  PID_Y.SetOutputLimits(-PID_MAX_VALUE/10, PID_MAX_VALUE/10);
  PID_Y.SetSampleTime(TimeIntervalMilliSeconds);
  PID_Y.SetTunings(KpY, KiY, KdY);
  /* End PID Setup */  

  /* PWM Setup */
  analogWriteResolution(16) ;
  /* End PWM Setup */
  
  /* Get ready to start acquiring GPS */
  BT.println("'g' to begin GPS acuisition, Tone when complete");
  while(true) {
    if(BT.available()) {
        go = (char)BT.read() ;
        if (go == 'g')
          break ;
    }  
    Serial.println("waiting for 'g'...");
    
    delay(1000);
  }
  //FUNCTION TO SEARCH FOR GPS FIX AND SOUND TONE
  //fcontrol.initFlight() ; 
  /************************************************************************** */
  /* ONCE TONE HAS SOUNDED GPS FIX ACQUIRED*/
  go = 'n' ;
  BT.println("GPS FIX ACQUIRED. 'g' to begin flight sequence");
  while(true) {
    if(BT.available()) {
        go = (char)BT.read() ;
        if (go == 'g')
          break ;
    }  
    Serial.println("waiting for 'g'...");
    
    delay(1000);
  } 
  /* SETUP THE INITIAL STATE */
  //initialPosition = getGPSData() ; 
  /**********************************/
  flightCoors[flightModeIndex].x = initialPosition.x ;
  flightCoors[flightModeIndex].y = initialPosition.y ;
  flightCoors[flightModeIndex].z = initialPosition.z ;
  imu.readYPR(ypr) ;
  Yaw_Setpoint = ypr[0] ;
  InputY = fcontrol.rotateAxes(Yaw_Setpoint, Yaw_Setpoint) ;
  flightModeIndex++; // NEXT FLIGHT MODE (Take Off)
  flightCoors[flightModeIndex].x= initialPosition.x ;
  flightCoors[flightModeIndex].y = initialPosition.y ;
  flightCoors[flightModeIndex].z = flightCoors[flightModeIndex].z ; // Set target altitude
  flightCoors[flightModeIndex+1].x = initialPosition.x ;
  flightCoors[flightModeIndex+1].y = initialPosition.y ;
  flightCoors[flightModeIndex+1].z = flightCoors[flightModeIndex].z ; // Set target altitude
  flightCoors[flightModeIndex+2].x = initialPosition.x ;
  flightCoors[flightModeIndex+2].y = initialPosition.y ;
  flightCoors[flightModeIndex+2].z = initialPosition.z ; // Set landing altitude
  flightCoors[flightModeIndex+3].x = initialPosition.x ;
  flightCoors[flightModeIndex+3].y = initialPosition.y ;
  flightCoors[flightModeIndex+3].z = initialPosition.z ; // Set landing altitude
  /* INITIAL STATE SET, READY TO FLY */
  
  for (int i = 0; i <= 10 ; i++) {
    BT.print("T minus ");BT.print(10-i);BT.print(" seconds\n");
    if(i != 0) delay(1000) ;
  }
  BT.println("Liftoff, Hopefully...\n");delay(1000);
  
  IMUUpdate.begin(imuISR, TimeIntervalMilliSeconds * ms2us) ;
  TenHzTask.begin(TenHzISR, TimeInterval10HzTask * ms2us) ;
  
}

// BEGIN MAIN LOOP 
// ************************************************************************************************************** //
void loop() {
  struct xyz ip ;
  double yawT ;
  if (flightMode[flightModeIndex] == CHARGE ){
    IMUUpdate.end();
    TenHzTask.end();
    //charge(); *******************************************************
    IMUUpdate.begin(imuISR, TimeIntervalMilliSeconds * ms2us) ;
    TenHzTask.begin(TenHzISR, TimeInterval10HzTask * ms2us) ; 
    flightModeIndex = 1 ;
  }
  // 10 HZ GPS TASK
  if (newDataGPS == TRUE ) // if GPS Interrupt has fired
  {
    //curr_locf = getGPSData() ; 
    /*****************************/ // NEED THIS FUNCTION
    curr_loc.x = curr_locf.x ;
    curr_loc.y = curr_locf.y ;
    curr_loc.z = curr_locf.z ;
    
    imu.readYPR(ypr) ;
    /*  */
    yawT = ypr[0] + magneticDec ; // ******************************* // Does this need to be here? in ENU, does 'N' mean true north?
    if (yawT > 360.0) { yawT = yawT - 360.0; }
    else if (yawT < 0.0) { yawT = yawT + 360; }
    ip.x = initialPosition.x ;
    ip.y = initialPosition.y ;
    ip.z = initialPosition.z ;
    XYZ_SP = fcontrol.computeXYZSetpoints(flightCoors[flightModeIndex],curr_loc, flightMode[flightModeIndex], yawT, ip); // ******** NEED TO HAVE MAGNETIC DECLINATION? IS IT ADDING OR SUBTRACTING? WE HAVE "POSITIVE" DECLINATION IN CHENEY
    PID_Z.SetTunings(KpZ, KiZ, KdZ) ;  //  Takes in fdest, curr_loc vectors relative to antenna
    PID_N.SetTunings(KpN, KiN, KdN) ;
    PID_E.SetTunings(KpE, KiE, KdE) ;
    N_Setpoint = XYZ_SP.x ;
    E_Setpoint = XYZ_SP.y ;
    Z_Setpoint = XYZ_SP.z ;
    PID_N.Compute() ;
    PID_E.Compute() ;
    PID_Z.Compute() ;
    Roll_Setpoint = SpeedControl*OutputE ; // SpeedControl and ZControl allow for user changes to angles and thrust in real time 
    Pitch_Setpoint = SpeedControl*OutputN ;
    basePWM = basePWM+ZControl*OutputZ ;
    newDataGPS = FALSE ;
  }
  // 100 HZ IMU/MOTOR UPDATE TASK
  if (newDataIMU == TRUE) 
  {
    /* Read Pitch, Roll, and Yaw */
    imu.readYPR(ypr) ;
    /* Push inputs into PID Controllers, get outputs*/
    PID_P.SetTunings(KpP, KiP, KdP);
    PID_R.SetTunings(KpR, KiR, KdR);
    PID_Y.SetTunings(KpY, KiY, KdY);
    InputY = fcontrol.rotateAxes(Yaw_Setpoint, ypr[0]);
    InputP = ypr[1] ;
    InputR = ypr[2] ;
    PID_Y.Compute();
    PID_P.Compute();
    PID_R.Compute();
    /* End PID */
  
   /* Take PID output, Start Motor Update Sequence */
    yawOffset = u2deltaPWM_Y * OutputY ;
    rollOffset = u2deltaPWM_R * OutputR ;
    pitchOffset = u2deltaPWM_P * OutputP ;
  
    motorControl.setNS(basePWM, pitchOffset, yawOffset) ;
    motorControl.setEW(basePWM, rollOffset, yawOffset) ;
    /* End motor update sequence */
    
    newDataIMU = FALSE ;
    cnt++ ;
      /* Get New Commands */
  if (BT.available()) {
    getBT();
  }
  
  if (cnt == displayPeriod*100) {  
    /* Print for data/debugging*/
    printDebug();
  }
  }

}
//*****************************************************************************************************************//
// END MAIN LOOP //

/* IMU Read Interrupt Service Routine */
void imuISR ( void ) {
  newDataIMU = TRUE ;
}
/* End IMU ISR */

/* GPS Read Interrupt Service Routine */
void TenHzISR ( void ) {
  newDataGPS = TRUE ;
}
/* End GPS ISR */

/* IMU Calibration Function, runs for 120 seconds max, LED turns on when done */
void calibrateIMU() {
  BT.println("Cal: No=0, full=3");

  uint8_t stats[4];
  for (int i = 0; i < 240; i++) {
    imu.readCalibration(stats);
    if ((stats[0] == 3) && (stats[1] == 3) && (stats[3] == 3)) {
      break;
    }
    BT.print("  Sys "); BT.print(stats[0]);
    BT.print("  Gyr "); BT.print(stats[1]);
    BT.print("  Acc "); BT.print(stats[2]);
    BT.print("  Mag "); BT.println(stats[3]);
    delay(500);
  }
}
/* End IMU Calibration Function */

/* GET BLUETOOTH COMMANDS FUNCTION */
void getBT( void ) {
  char BTcommand ;
  BTcommand = (char)BT.read() ;
    BT.flush() ;
    // Begin Ridiculously Long Switch Statement
    switch (BTcommand) {
    // Kp Roll
      case 'm' :
        KpR = KpR - deltaKP ;
        break ;
      case 'M' :
        KpR = KpR + deltaKP ;
        break ;
    // Kp Pitch
      case 'n' :
        KpP = KpP - deltaKP ;
        break ;
      case 'N' :
        KpP = KpP + deltaKP ;
        break;
    // Kp Yaw
      case 'b' :
        KpY = KpY - deltaKP ;
        break ;
      case 'B' :
        KpY = KpY + deltaKP ;
        break ;
    
    // Ki Roll
      case 'v' :
        KiR = KiR - deltaKI ;
        break ;
      case 'V' :
        KiR = KiR + deltaKI ;
        break ;
    // Ki Pitch
      case 'c' : 
        KiP = KiP - deltaKI ;
        break ;
      case 'C' :
        KiP = KiP + deltaKI;
        break ;
    // Ki Yaw
      case 'x' :
        KiY = KiY - deltaKI ;
        break ;
      case 'X' :
        KiY = KiY + deltaKI ;
        break ;

    // Kd Roll
      case 'z' :
        KdR = KdR - deltaKD ;
        break ;
      case 'Z' :
        KdR = KdR + deltaKD ;
        break ;
    // Kd Pitch
      case 's' :
        KdP = KdP - deltaKD ;
        break ;
      case 'S' :
        KdP = KdP + deltaKD ;
        break ;
    // Kd Yaw
      case 'a' :
        KdY = KdY - deltaKD ;
        break ;
      case 'A' :
        KdY = KdY + deltaKD ;
        break ;
        
    // KILL THE TEST, watch it fall from the sky
      case 'k' :
        motorControl.stopAll();
        basePWM = 40.0 ;
        while ( (char)BT.read() != 'g' ) {
          BT.flush() ;  
        }
        PID_P.ResetOutput();
        PID_R.ResetOutput();
        Yaw_Setpoint = ypr[0];
        InputY = fcontrol.rotateAxes(Yaw_Setpoint, InputY);
        PID_Y.ResetOutput();
        break ;
    // MANUALLY INDUCE A HOVER STATE
      case 'h' :
        flightModeIndex = 2 ; // HOVER INDEX
        flightCoors[flightModeIndex].x = curr_loc.x ;
        flightCoors[flightModeIndex].y = curr_loc.y ;
        flightCoors[flightModeIndex].z = HOVERALTITUDE ;
        break ;
    // MANUALLY INDUCE A LANDING ATTEMPT (lowercase L)
      case 'l' :
        flightModeIndex = 3 ; // LANDING INDEX
        flightCoors[flightModeIndex].x = curr_loc.x ;
        flightCoors[flightModeIndex].y = curr_loc.y ;
        flightCoors[flightModeIndex].z = initialPosition.z ;
        break ;
     // Setpoints 
      case 'Y' :
        Yaw_Setpoint = Yaw_Setpoint + deltaSP ;
        if (Yaw_Setpoint < 0)
            Yaw_Setpoint = Yaw_Setpoint + 360.0 ;
        else if (Yaw_Setpoint > 360.0)
            Yaw_Setpoint = Yaw_Setpoint - 360.0 ;
        break ;
      case 'y' :
        Yaw_Setpoint = Yaw_Setpoint - deltaSP ;
        if (Yaw_Setpoint < 0)
            Yaw_Setpoint = Yaw_Setpoint + 360.0 ;
        else if (Yaw_Setpoint > 360.0)
            Yaw_Setpoint = Yaw_Setpoint - 360.0 ;
        break ;
      case '1' :
        Yaw_Setpoint = 25 ;
        break ;
      case '2' :
        Yaw_Setpoint = 45 ;
        break ;
      case '3' :
        Yaw_Setpoint = 90 ;
        break ;
      case '4' :
        Yaw_Setpoint = 135 ;
        break ;
      case '5' :
        Yaw_Setpoint = 270 ;
        break ;
      case '6' :
        Yaw_Setpoint = 315 ;
        break ;
// SPEED CONTROL XYZ
      case 't' :
        ZControl = ZControl - 0.25 ;
        break ;
      case 'T' :
        ZControl = ZControl + 0.25 ;
        break ;
      case '+' :
        SpeedControl = SpeedControl + 0.1 ; // XY
        break ;
      case '-' :
        SpeedControl = SpeedControl - 0.1 ; // XY
        if (SpeedControl <= 0.0)
            SpeedControl = 0.1 ;
    } 
}
/* END GET COMMANDS FUNCTION */

/* Charge() function
 * Waits for signal from charging station that charge has completed
*/
//void charge( void ) {
//    while ((char)CSBT.read() != 'd'){
//        BT.println("charging...") ;
//        CSBT.flush();
//        delay(1000);
//    }
//}



/* Print Values for Debugging */
void printDebug ( void ) {
  
      /* Print for data/debugging*/
    BT.println("------------------------------------------");
    switch (flightMode[flightModeIndex]) {
        case CHARGE :
            BT.print("MODE = CHARGE\n");
            break ;
        case TAKEOFF :
            BT.print("MODE = TAKEOFF\n");
            break ;
        case HOVER :
            BT.print("MODE = HOVER\n");
            break ;
        case TRANSLATE :
            BT.print("MODE = TRANSLATE\n");
            break ;
        case LAND :
            BT.print("MODE = LANDING\n");
            break ;
    }
    BT.print("DESTINATION > X = ");BT.print(flightCoors[flightModeIndex].x);
    BT.print(" Y = ");BT.print(flightCoors[flightModeIndex].y);BT.print(" Z = ");BT.print(flightCoors[flightModeIndex].z);
    BT.println();
    BT.print("CVec XY Leng = ");BT.print(SpeedControl);BT.print("\n");
    BT.print("CVec Z  Leng = ");BT.print(ZControl);BT.print("\n");
    BT.print("Base PWM     = ");BT.print(basePWM);BT.print("%");
    BT.print("\n\n");
    BT.print("Yaw Actual     = ");BT.print(ypr[0],4);BT.print("  ");BT.print(InputY);BT.println();
    BT.print("Pitch Actual   = ");BT.print(ypr[1],4);BT.println();
    BT.print("Roll Actual    = ");BT.print(ypr[2],4);BT.println();BT.println();
    BT.print("Yaw Setpoint   = ");BT.print(Yaw_Setpoint,4);BT.println();
    BT.print("Pitch Setpoint = ");BT.print(Pitch_Setpoint,4);BT.println();
    BT.print("Roll Setpoint  = ");BT.print(Roll_Setpoint,4);BT.println() ;
    BT.println();
    BT.print("Error Signal\n");BT.print("Pitch =  ");BT.print(pitchOffset,4);BT.println();
    BT.print("Roll =  ");BT.print(rollOffset,4);BT.println();
    BT.print("Yaw  =  ");BT.print(yawOffset,4);BT.println();BT.println();
    cnt = 0;
}
/* End print function */
