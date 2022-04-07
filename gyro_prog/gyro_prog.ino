// MPU-9250 Digital Motion Processing (DMP) Library
#include <SparkFunMPU9250-DMP.h>
// config.h manages default logging parameters and can be used
// to adjust specific parameters of the IMU
#include "config.h"
#include <Adafruit_SleepyDog.h>

MPU9250_DMP imu; // Create an instance of the MPU9250_DMP class

unsigned short accelFSR = IMU_ACCEL_FSR;
unsigned short gyroFSR = IMU_GYRO_FSR;
unsigned short fifoRate = DMP_SAMPLE_RATE;
/////////////////////
// LED Blink Control //
///////////////////////
//bool ledState = false;
uint32_t lastBlink = 0;
void blinkLED()
{
  static bool ledState = false;
  digitalWrite(HW_LED_PIN, ledState);
  ledState = !ledState;
}

void setup()
{
  // Initialize LED, interrupt input, and serial port.
  // LED defaults to off:
  //delay(5000);
  Serial1.begin(115200);
  //SerialUSB.begin(9600);
  pinMode(10, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
   
  //SerialUSB.println("statre");
  initHardware();
  //SerialUSB.println("statre");
  // Initialize the MPU-9250. Should return true on successs
  if ( !initIMU() )
  {
    //LOG_PORT.println("Error connecting to MPU-9250");
    while (true)
    {
      //SerialUSB.println("1");
    }
    // Loop forever if we fail to connect
    // LED will remain off in this state.
  }
  //SerialUSB.println("start");
  Watchdog.enable(500);
}
//long time = 0;
void loop()
{
  // Then check IMU for new data, and log it
  if ( !imu.fifoAvailable() ) return; // return to the top of the loop
  //SerialUSB.println("loop2");
  // Read from the digital motion processor's FIFO
  if ( imu.dmpUpdateFifo() != INV_SUCCESS ) return; // If that fails (uh, oh), return to top
  //SerialUSB.println("loop3");
  // If enabled, read from the compass.
  if (imu.updateCompass() != INV_SUCCESS) return; // If compass read fails (uh, oh) return to to
  // If logging (to either UART and SD card) is enabled
  logIMUData(); // Log new data
  //SerialUSB.println(millis()-time);
  //time = millis();
  Watchdog.reset();
}
byte lastS = 0;
void logIMUData(void)
{
  imu.computeEulerAngles();
  Serial1.write(120 | (digitalRead(10) | (digitalRead(11)<<1)));
  long summ = 120;
  //SerialUSB.println("2");
  Serial1.write(int(imu.yaw) >> 8);
  summ+=int(imu.yaw) >> 8;
  //SerialUSB.println("3");
  Serial1.write(int(imu.yaw) & 255);
  summ+=int(int(imu.yaw) & 255);
  //SerialUSB.println("4");
  //SerialUSB.print(imu.pitch);
  //SerialUSB.print(" ");
  //SerialUSB.println(imu.roll - 180);
  int pt = (int)max(min(abs(imu.pitch), abs(360 - imu.pitch)), min(abs(imu.roll - 180), abs(360 - (imu.roll - 180))));
  Serial1.write(pt >> 8);
  
  //SerialUSB.println(pt);
  summ += pt >> 8;
  Serial1.write(pt & 255);
  summ += pt & 255;
  //SerialUSB.println("6");
  int ac = int((imu.calcAccel(imu.ay) + 10) * 10);
  Serial1.write(ac >> 8);
  summ += ac >> 8;
  //SerialUSB.println("7");
  Serial1.write(ac & 255);
  summ += ac & 255;
  //SerialUSB.println("8");
  Serial1.write(int(summ / 7));
  if ( millis() > lastBlink + UART_BLINK_RATE )
  {
    blinkLED();
    lastBlink = millis();
  }
  //SerialUSB.print(digitalRead(10));
  //SerialUSB.println(digitalRead(11));
}

void initHardware(void)
{
  // Set up LED pin (active-high, default to off)
  pinMode(HW_LED_PIN, OUTPUT);
  digitalWrite(HW_LED_PIN, LOW);

  // Set up MPU-9250 interrupt input (active-low)
  pinMode(MPU9250_INT_PIN, INPUT_PULLUP);

  // Set up serial log port
  //LOG_PORT.begin(SERIAL_BAUD_RATE);
  //SerialUSB.begin(9600);
}

bool initIMU(void)
{
  // imu.begin() should return 0 on success. Will initialize
  // I2C bus, and reset MPU-9250 to defaults.
  if (imu.begin() != INV_SUCCESS)
    return false;

  // Set up MPU-9250 interrupt:
  imu.enableInterrupt(); // Enable interrupt output
  imu.setIntLevel(1);    // Set interrupt to active-low
  imu.setIntLatched(1);  // Latch interrupt output

  // Configure sensors:
  // Set gyro full-scale range: options are 250, 500, 1000, or 2000:
  imu.setGyroFSR(gyroFSR);
  // Set accel full-scale range: options are 2, 4, 8, or 16 g
  imu.setAccelFSR(accelFSR);
  // Set gyro/accel LPF: options are5, 10, 20, 42, 98, 188 Hz
  imu.setLPF(IMU_AG_LPF);
  // Set gyro/accel sample rate: must be between 4-1000Hz
  // (note: this value will be overridden by the DMP sample rate)
  imu.setSampleRate(IMU_AG_SAMPLE_RATE);
  // Set compass sample rate: between 4-100Hz
  imu.setCompassSampleRate(IMU_COMPASS_SAMPLE_RATE);

  // Configure digital motion processor. Use the FIFO to get
  // data from the DMP.
  unsigned short dmpFeatureMask = 0;
  if (ENABLE_GYRO_CALIBRATION)
  {
    // Gyro calibration re-calibrates the gyro after a set amount
    // of no motion detected
    dmpFeatureMask |= DMP_FEATURE_SEND_CAL_GYRO;
    dmpFeatureMask |= DMP_FEATURE_GYRO_CAL;
  }
  else
  {
    // Otherwise add raw gyro readings to the DMP
    dmpFeatureMask |= DMP_FEATURE_SEND_RAW_GYRO;
  }
  // Add accel and quaternion's to the DMP
  dmpFeatureMask |= DMP_FEATURE_SEND_RAW_ACCEL;
  dmpFeatureMask |= DMP_FEATURE_6X_LP_QUAT;

  // Initialize the DMP, and set the FIFO's update rate:
  imu.dmpBegin(dmpFeatureMask, fifoRate);

  return true; // Return success
}
