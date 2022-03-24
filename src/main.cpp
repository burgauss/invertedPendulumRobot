#include <Arduino.h>
#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#include <My_Motors.h>
#include <Encoder.h>  // Encoder Library - http://www.pjrc.com/teensy/td_libs_Encoder.html
#include "I2C.h"
#include <stdint.h>
#include "direct_pin_read.h"

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

// comment to test the git

/* IMU1 Data */
Kalman X_kalman; // Create the Kalman instances
double X_acc, Y_acc, Z_acc; // Raw acceleromoter Readings
double X_gyro, Y_gyro, Z_gyro; // Raw gyroscope Readings
double X_gyroangle, Y_gyroangle; // Angle calculate using the gyroscope only
double X_kalAngle, Y_kalAngle; // Calculated angle using a Kalman filter
uint32_t timer; // timer to track time
uint8_t i2cData[14]; // Buffer for I2C data

/* IMU2 Data */
Kalman X_kalman1; // Create the Kalman instances
double X_acc1, Y_acc1, Z_acc1; // Raw acceleromoter Readings
double X_gyro1, Y_gyro1, Z_gyro1; // Raw gyroscope Readings
double X_gyroangle1, Y_gyroangle1; // Angle calculate using the gyro only
double X_kalAngle1, Y_kalAngle1; // Calculated angle using a Kalman filter
uint32_t timer1; // timer to track time
uint8_t i2cData1[14]; // Buffer for I2C data

// For Speed Mesurement
float r_whl = 0.059; // radius of the wheel
float V_trans; // Translation velocity
float rpm_limit = 0.10; // RPM below this is considered 0
float avg_pt = 10.0;  // Number of points used for exponentially averaging the RPM signal
short PPR = 400; // Number of pulses per revolution of the encoder (for a gearbox 1:50, this value is 400)
float Final_Rpm_r, Final_Rpm_l; // Motor final averaged out RPM, units can be selected while calling get_RPM function
My_Motors Rmot(&Final_Rpm_r, rpm_limit, avg_pt, PPR); // Right motor object for calculating rotational velocities from encoder data
My_Motors Lmot(&Final_Rpm_l, rpm_limit, avg_pt, PPR); // Left motor object for calculating rotational velocities from encoder data

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc_r(2, 4);
Encoder myEnc_l(3, 5);

// Calibration routine
void setup() {
  Serial.begin(115200); // serial communication with specified baudrate
  Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

// for imu1
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x68,0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x68,0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x68,0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(i2cData[0]);
    Serial.print(F("Error reading first IMU"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x68,0x3B, i2cData, 6));
  X_acc = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  Y_acc = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  Z_acc = (int16_t)((i2cData[4] << 8) | i2cData[5]);

// Source: https://www.nxp.com/files-static/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(Y_acc, Z_acc) * RAD_TO_DEG;
  double pitch = atan(-X_acc / sqrt(Y_acc * Y_acc + Z_acc * Z_acc)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(Y_acc / sqrt(X_acc * X_acc + Z_acc * Z_acc)) * RAD_TO_DEG;
  double pitch = atan2(-X_acc, Z_acc) * RAD_TO_DEG;
#endif

  X_kalman.setAngle(roll); // Set starting angle
  // kalmanY.setAngle(pitch); // commented, not use of kalmanY
  X_gyroangle = roll;
  Y_gyroangle = pitch;
  timer = micros();


// for imu2
  i2cData1[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData1[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData1[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData1[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x69,0x19, i2cData1, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x69,0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x69,0x75, i2cData1, 1));
  if (i2cData1[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print("2nd one");
    Serial.print(i2cData1[0]);
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x69,0x3B, i2cData1, 6));
  X_acc1 = (int16_t)((i2cData1[0] << 8) | i2cData1[1]);
  Y_acc1 = (int16_t)((i2cData1[2] << 8) | i2cData1[3]);
  Z_acc1 = (int16_t)((i2cData1[4] << 8) | i2cData1[5]);

#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll1  = atan2(Y_acc1, Z_acc1) * RAD_TO_DEG;
  double pitch1 = atan(-X_acc1 / sqrt(Y_acc1 * Y_acc1 + Z_acc1 * Z_acc1)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll1  = atan(Y_acc1 / sqrt(X_acc1 * X_acc1 + Z_acc1 * Z_acc1)) * RAD_TO_DEG;
  double pitch1 = atan2(-X_acc1, Z_acc1) * RAD_TO_DEG;
#endif

  X_kalman1.setAngle(roll1); // Set starting angle
  // kalmanY1.setAngle(pitch1);
  X_gyroangle1 = roll1;
  Y_gyroangle1 = pitch1;
}

long positionLeft  = -999; // set encoder to ramdom value
long positionRight = -999; // set encoder to ramdom value

void loop() {


  //Reset Encoder when we have signal from Raspberry Pi
  if (Serial.available() > 0) {
    int Number = Serial.read() - '0';
    if (Number == 1) {
        myEnc_l.write(0);
        myEnc_r.write(0);}}

  /* Update all the values for imu1 */
  while (i2cRead(0x68,0x3B, i2cData, 14));
  X_acc = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  Y_acc = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  Z_acc = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  X_gyro = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  Y_gyro = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  Z_gyro = (int16_t)((i2cData[12] << 8) | i2cData[13]);;
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(Y_acc, Z_acc) * RAD_TO_DEG;
  double pitch = atan(-X_acc / sqrt(Y_acc * Y_acc + Z_acc * Z_acc)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(Y_acc / sqrt(X_acc * X_acc + Z_acc * Z_acc)) * RAD_TO_DEG;
  double pitch = atan2(-X_acc, Z_acc) * RAD_TO_DEG;
#endif

  double X_gyrorate = X_gyro / 131.0; // Convert to deg/s
  double Y_gyrorate = Y_gyro / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && X_kalAngle > 90) || (roll > 90 && X_kalAngle < -90)) {
    X_kalman.setAngle(roll);
    X_kalAngle = roll;
    X_gyroangle = roll;
  } else
    X_kalAngle = X_kalman.getAngle(roll, X_gyrorate, dt); // Calculate the angle using a Kalman filter

  if (abs(X_kalAngle) > 90)
    Y_gyrorate = -Y_gyrorate; // Invert rate, so it fits the restriced accelerometer reading
  // Y_kalAngle = kalmanY.getAngle(pitch, Y_gyrorate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && Y_kalAngle > 90) || (pitch > 90 && Y_kalAngle < -90)) {
    kalmanY.setAngle(pitch);
    Y_kalAngle = pitch;
    Y_gyroangle = pitch;
  } else
    Y_kalAngle = kalmanY.getAngle(pitch, Y_gyrorate, dt); // Calculate the angle using a Kalman filter

  if (abs(Y_kalAngle) > 90)
    X_gyrorate = -X_gyrorate; // Invert rate, so it fits the restriced accelerometer reading
  X_kalAngle = X_kalman.getAngle(roll, X_gyrorate, dt); // Calculate the angle using a Kalman filter
#endif

  X_gyroangle += X_gyrorate * dt; // Calculate gyro angle without any filter
  Y_gyroangle += Y_gyrorate * dt;

  // Reset the gyro angle when it has drifted too much
  if (X_gyroangle < -180 || X_gyroangle > 180)
    X_gyroangle = X_kalAngle;
  if (Y_gyroangle < -180 || Y_gyroangle > 180)
    Y_gyroangle = Y_kalAngle;


      /* Update all the values for imu2 */
  while (i2cRead(0x69,0x3B, i2cData1, 14));
  X_acc1 = (int16_t)((i2cData1[0] << 8) | i2cData1[1]);
  Y_acc1 = (int16_t)((i2cData1[2] << 8) | i2cData1[3]);
  Z_acc1 = (int16_t)((i2cData1[4] << 8) | i2cData1[5]);
  X_gyro1 = (int16_t)((i2cData1[8] << 8) | i2cData1[9]);
  Y_gyro1 = (int16_t)((i2cData1[10] << 8) | i2cData1[11]);
  Z_gyro1 = (int16_t)((i2cData1[12] << 8) | i2cData1[13]);;
  dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll1  = atan2(Y_acc1, Z_acc1) * RAD_TO_DEG;
  double pitch1 = atan(-X_acc1 / sqrt(Y_acc1 * Y_acc1 + Z_acc1 * Z_acc1)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll1  = atan(Y_acc1 / sqrt(X_acc1 * X_acc1 + Z_acc1 * Z_acc1)) * RAD_TO_DEG;
  double pitch1 = atan2(-X_acc1, Z_acc1) * RAD_TO_DEG;
#endif

  double X_gyrorate1 = X_gyro1 / 131.0; // Convert to deg/s
  double Y_gyrorate1 = Y_gyro1 / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll1 < -90 && X_kalAngle1 > 90) || (roll1 > 90 && X_kalAngle1 < -90)) {
    X_kalman1.setAngle(roll1);
    X_kalAngle1 = roll1;
    X_gyroangle1 = roll1;
  } else
    X_kalAngle1 = X_kalman1.getAngle(roll1, X_gyrorate1, dt); // Calculate the angle using a Kalman filter

  if (abs(X_kalAngle1) > 90)
    Y_gyrorate1 = -Y_gyrorate1; // Invert rate, so it fits the restriced accelerometer reading
  // Y_kalAngle1 = kalmanY1.getAngle(pitch1, Y_gyrorate1, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch1 < -90 && Y_kalAngle1 > 90) || (pitch1 > 90 && Y_kalAngle1 < -90)) {
    kalmanY1.setAngle(pitch1);
    Y_kalAngle1 = pitch1;
    Y_gyroangle1 = pitch1;
  } else
    Y_kalAngle1 = kalmanY1.getAngle(pitch1, Y_gyrorate1, dt); // Calculate the angle using a Kalman filter

  if (abs(Y_kalAngle1) > 90)
    X_gyrorate1 = -X_gyrorate1; // Invert rate, so it fits the restriced accelerometer reading
  X_kalAngle1 = X_kalman1.getAngle(roll1, X_gyrorate1, dt); // Calculate the angle using a Kalman filter
#endif

  X_gyroangle1 += X_gyrorate1 * dt; // Calculate gyro angle without any filter
  Y_gyroangle1 += Y_gyrorate1 * dt;

  // Reset the gyro angle when it has drifted too much
  if (X_gyroangle1 < -180 || X_gyroangle1 > 180)
    X_gyroangle1 = X_kalAngle1;
  if (Y_gyroangle1 < -180 || Y_gyroangle1 > 180)
    Y_gyroangle1 = Y_kalAngle1;

  // RPM measruement for motor
  Lmot.getRPM(myEnc_l.read(), "rad/s"); // Get current encoder counts & compute left motor rotational velocity in [rad/s] 
  Rmot.getRPM(myEnc_r.read(), "rad/s"); // Get current encoder counts & compute right motor rotational velocity in [rad/s]
  V_trans = 0.5 * (Final_Rpm_r + Final_Rpm_l);// Calculate the total Robot linear translation velocity [m/s]

  // Reading Encoder values
  long newLeft, newRight;
  newLeft = myEnc_l.read();
  newRight = myEnc_r.read();
  if (newLeft != positionLeft || newRight != positionRight) {
    positionLeft = newLeft;
    positionRight = newRight;
  }

  // Write data to serial line
  Serial.print("\r\n");
  delay(2);
  Serial.print('<');Serial.print(",");
  Serial.print(X_kalAngle1); Serial.print(",");
  Serial.print(X_kalAngle); Serial.print(",");
  Serial.print(newRight); Serial.print(",");
  Serial.print(newLeft); Serial.print(",");
  Serial.print(V_trans); Serial.print(",");
  Serial.print(Final_Rpm_r); Serial.print(",");
  Serial.print(Final_Rpm_l); Serial.print(",");
  double Y_acc_ = Y_acc/16384; // Acceration directly from accelerometer
  Serial.print(Y_acc_); Serial.print(",");
  Serial.print(X_gyrorate); Serial.print(",");
}
