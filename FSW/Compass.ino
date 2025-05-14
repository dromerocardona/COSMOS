#include <Wire.h>
#include <LIS3MDL.h>
#include <LSM6.h>
#include <cmath>  // For cos, sin, and atan2

LIS3MDL mag;
LSM6 imu;

// Calibration variables (for magnetometer)
float mx_min = 1e6, my_min = 1e6, mz_min = 1e6;
float mx_max = -1e6, my_max = -1e6, mz_max = -1e6;
float offset_x = 0, offset_y = 0, offset_z = 0;
float scale_x = 1, scale_y = 1, scale_z = 1;
bool calibrated = false;

// Running average variables
float avg_cos = 0;      // Average cosine component
float avg_sin = 0;      // Average sine component
float avgLen = 5;       // Smoothing factor (higher = smoother, slower response)
bool first_update = true; // Flag for initializing the average

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!mag.init() || !imu.init()) {
    Serial.println("Failed to initialize sensors!");
    while (1);
  }
  mag.enableDefault();
  imu.enableDefault();

  Serial.println("Rotate device for 10 seconds to calibrate...");
}

void collectCalibrationData() {
  mag.read();
  mx_min = min(mx_min, mag.m.x);
  my_min = min(my_min, mag.m.y);
  mz_min = min(mz_min, mag.m.z);
  mx_max = max(mx_max, mag.m.x);
  my_max = max(my_max, mag.m.y);
  mz_max = max(mz_max, mag.m.z);
}

void computeCalibration() {
  offset_x = (mx_max + mx_min) / 2.0;
  offset_y = (my_max + my_min) / 2.0;
  offset_z = (mz_max + mz_min) / 2.0;

  float rx = (mx_max - mx_min) / 2.0;
  float ry = (my_max - my_min) / 2.0;
  float rz = (mz_max - mz_min) / 2.0;
  float r_avg = (rx + ry + rz) / 3.0;

  scale_x = r_avg / rx;
  scale_y = r_avg / ry;
  scale_z = r_avg / rz;

  calibrated = true;
  Serial.println("Calibration complete.");
}

float computeTiltCompensatedHeading() {
  mag.read();
  imu.read();

  // Apply calibration
  LIS3MDL::vector<float> m = {
    (mag.m.x - offset_x) * scale_x,
    (mag.m.y - offset_y) * scale_y,
    (mag.m.z - offset_z) * scale_z
  };
  LIS3MDL::vector<int16_t> a = {-imu.a.y, imu.a.x, imu.a.z};

  // Tilt compensation
  LIS3MDL::vector<float> E, N;
  LIS3MDL::vector_cross(&m, &a, &E);
  LIS3MDL::vector_normalize(&E);
  LIS3MDL::vector_cross(&a, &E, &N);
  LIS3MDL::vector_normalize(&N);

  LIS3MDL::vector<int> z_axis = {0, 0, 1};
  float heading = atan2(LIS3MDL::vector_dot(&E, &z_axis),
                       LIS3MDL::vector_dot(&N, &z_axis)) * 180.0 / M_PI;
  if (heading < 0) heading += 360;

  return heading;
}

void updateRunningAverage(float heading, float& smoothed_heading) {
  // Convert heading to radians
  float heading_rad = heading * M_PI / 180.0;

  if (first_update) {
    // Initialize with the first heading
    avg_cos = cos(heading_rad);
    avg_sin = sin(heading_rad);
    smoothed_heading = heading;
    first_update = false;
  } else {
    // Update averages using EMA
    float alpha = (avgLen - 1.0) / avgLen; // Weight for previous average
    avg_cos = alpha * avg_cos + (1 - alpha) * cos(heading_rad);
    avg_sin = alpha * avg_sin + (1 - alpha) * sin(heading_rad);

    // Compute smoothed heading
    smoothed_heading = atan2(avg_sin, avg_cos) * 180.0 / M_PI;
    if (smoothed_heading < 0) smoothed_heading += 360;
  }
}

void loop() {
  static unsigned long startTime = millis();

  // Calibration phase
  if (!calibrated) {
    if (millis() - startTime < 10000) {
      collectCalibrationData();
      delay(50);
    } else {
      computeCalibration();
    }
    return;
  }

  // Compute heading and update running average
  float heading = computeTiltCompensatedHeading();
  float smoothed_heading;
  updateRunningAverage(heading, smoothed_heading);

  // Output for plotting (e.g., Serial Plotter)
  Serial.print("360,0,");         // Reference lines
  Serial.print(smoothed_heading); // Smoothed heading
  Serial.print(",");
  Serial.println(heading);        // Raw heading
}
