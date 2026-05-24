// =========================================================================
// ESP32 9-DOF Breadcrumb Trail Navigator - V5
// =========================================================================
// This code implements a high-accuracy pedometer and navigation system by
// fusing data from a 9-DOF sensor array.
//
// Key Features:
// - 9-DOF Sensor Fusion: Combines MPU6050 and QMC5883L for a stable heading.
// - Magnetometer Calibration: Mandatory startup routine for heading accuracy.
// - Autocorrelation Pedometer: For robust step detection.
// - Barometric Altitude: Uses a BMP180 sensor for precise altitude tracking.
//
// Hardware:
// - ESP32 Development Board
// - I2C Bus: MPU6050, BMP180, QMC5883L, SSD1306 (all on default I2C pins)
// =========================================================================

#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP085.h>
#include <QMC5883LCompass.h>

// --- I2C Bus Configuration ---
// For this version, we'll use the default I2C bus for all sensors
// Note: QMC5883LCompass library doesn't support setWire() method

// ----- WiFi Configuration -----
const char* ap_ssid = "myosa";        // ESP32 Access Point name
const char* ap_password = "";         // Leave empty for open access point
WebServer server(80);                 // Web server on port 80

// ----- Hardware Configuration -----
#define REVERSE_BUTTON_PIN 16    // GPIO15 for reverse navigation button
#define BUTTON_DEBOUNCE_MS 50    // Debounce delay in milliseconds

// ----- OLED Display Configuration -----
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

// The display is on the primary I2C bus.
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ----- Sensor Configuration -----
Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;
QMC5883LCompass compass;
bool bmp_found = false;

// ----- Enhanced Altitude System -----
struct AltitudeFilter {
  float barometric_altitude;
  float accelerometer_altitude;
  float filtered_altitude;
  float baseline_pressure;
  float altitude_offset;
  float vertical_velocity;
  unsigned long last_update;
  bool is_calibrated;
};

AltitudeFilter alt_filter = {0, 0, 0, 0, 0, 0, 0, false};

// ----- Path Tracking & Geolocation -----
#define MAX_PATH_POINTS 200
const float STEP_LENGTH_METERS = 0.75;
const float MAGNETIC_DECLINATION = -0.72; // For Thrissur, Kerala, India

// ----- Advanced Map Visualization -----
struct PathPoint {
  float x, y, z;
  float heading;
  uint8_t speed_level;  // 0-7 for visualization (0=stopped, 7=fast)
  unsigned long timestamp;
};

struct ViewPort {
  float centerX, centerY;
  float zoom_level;
  uint8_t zoom_mode;  // 0=auto, 1=recent_focus, 2=full_path
  bool auto_center;
};

// ----- Path Reversibility System -----
enum NavigationMode {
  FORWARD_MODE,      // Normal path recording
  REVERSE_MODE,      // Following path back to start
  RETURN_COMPLETE    // Successfully reached starting point
};

NavigationMode nav_mode = FORWARD_MODE;

// ----- Kalman Filter System -----
struct KalmanFilter {
  float x_est;           // Position estimate
  float x_vel;           // Velocity estimate
  float P[2][2];         // Error covariance matrix
  float Q;               // Process noise
  float R;               // Measurement noise
  unsigned long last_update;
};

// ----- Smart Waypoint System -----
enum WaypointType {
  WAYPOINT_GENERIC = 0,
  WAYPOINT_CAMP,
  WAYPOINT_WATER,
  WAYPOINT_FOOD,
  WAYPOINT_SHELTER,
  WAYPOINT_VIEW,
  WAYPOINT_DANGER,
  WAYPOINT_PHOTO,
  WAYPOINT_MEETING,
  WAYPOINT_EXIT
};

struct SmartWaypoint {
  float x, y, z;
  char name[16];
  WaypointType type;
  uint32_t timestamp;
  bool active;
  uint8_t icon_id;
};

#define MAX_WAYPOINTS 20
#define WAYPOINT_PROXIMITY_RADIUS 5.0  // Meters to consider "near" a waypoint

// ----- 3D Dashboard System -----
enum DashboardMode {
  DASHBOARD_OFF,     // Normal operation
  DASHBOARD_ON,      // Single 4-quadrant display  
  DASHBOARD_VERTICAL, // Vertical tracking display (follows 4-quadrant)
  DASHBOARD_TRANSITION // Animation transition state
};

struct DashboardState {
  DashboardMode current_mode;
  unsigned long mode_start_time;
  unsigned long last_transition_time;
  float transition_progress;
  int animation_phase;
  bool exiting;
};

struct SensorReadings {
  float temperature;
  float pressure;
  float altitude_abs;
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
  float air_density;
  unsigned long last_update;
};

// ----- Enhanced Reverse Navigation System -----
struct BreadcrumbTrail {
  int total_crumbs;           // Total breadcrumbs in trail
  int current_target_index;   // Current breadcrumb we're navigating to
  float target_x, target_y, target_z;   // Current target coordinates
  float distance_to_target;   // Distance to current target
  float bearing_to_target;    // Bearing to current target
  float required_heading;     // Required heading to reach target
  float heading_error;        // Difference between current and required heading
  bool reached_target;        // Flag when current target is reached
  float progress_percentage;  // Overall progress back to start (0-100%)
  unsigned long last_update;  // Last update timestamp
  bool algorithm_active;      // Is reverse navigation active
  int breadcrumbs_completed;  // Number of breadcrumbs successfully reached
  float total_remaining_distance; // Total distance remaining to start
  float distance_threshold;   // Distance threshold for reaching breadcrumb
  float direction_smoothing;  // Smoothing factor for direction changes
  float last_target_distance; // Previous distance to target
  float target_bearing;       // Smoothed bearing to target
};

BreadcrumbTrail breadcrumb_nav = {0, -1, 0, 0, 0, 0, 0, 0, 0, false, 0, 0, false, 0, 0, 1.5, 0.3, 999.0, 0.0};

#define ZOOM_LEVELS 5
const float zoom_scales[ZOOM_LEVELS] = {0.5, 1.0, 2.0, 4.0, 8.0};  // Zoom multipliers
const char* zoom_labels[ZOOM_LEVELS] = {"Wide", "Normal", "Close", "Detail", "Micro"};

// Enhanced visualization settings
#define TRAIL_LENGTH 8          // Number of points in current position trail
#define ARROW_SPACING 10        // Draw direction arrow every N points
#define PATH_FADE_STEPS 20      // Number of steps for path fading effect
#define COMPASS_SIZE 12         // Size of compass rose in pixels

// Path reversibility settings
#define WAYPOINT_PROXIMITY_THRESHOLD 2.0  // Meters to consider waypoint reached (better accuracy)
#define OFF_TRACK_THRESHOLD 4.0           // Meters before considering off-track (tighter tolerance)
#define MIN_WAYPOINT_DISTANCE 1.0         // Minimum distance between waypoints for reverse (more precise)
#define MAX_TURN_INSTRUCTION_ANGLE 15.0   // Degrees below which we say "go straight" (more forgiving)
#define INITIAL_GUIDANCE_TIME 3000        // Time to show initial turn guidance (3 seconds)
#define WAYPOINT_LOOKAHEAD 2              // Number of points to look ahead for smoother guidance
#define RETURN_TO_START_THRESHOLD 3.0     // Final approach threshold to starting point

// 3D Dashboard settings
#define DOUBLE_CLICK_TIMEOUT 400          // Maximum time between clicks for double-click (ms)
#define DASHBOARD_SCREEN_DURATION 8000    // Time each dashboard screen shows (8 seconds)
#define DASHBOARD_VERTICAL_DURATION 5000  // Time vertical tracking display shows (5 seconds) 
#define DASHBOARD_TRANSITION_TIME 400     // Time for simple transition animation (0.4 seconds)
#define DASHBOARD_REFRESH_RATE 50         // Sensor reading refresh rate (ms) - reduced lag
#define STARTUP_STABILIZATION_TIME 10000  // 10 seconds stabilization period to prevent false steps

// ----- Global Variables -----

// Orientation (9-DOF Sensor Fusion)
float pitch = 0, roll = 0, yaw = 0;
const float ALPHA = 0.98;
float gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0;

// Magnetometer calibration values
int mag_x_min = 32767, mag_x_max = -32768;
int mag_y_min = 32767, mag_y_max = -32768;
float mag_x_offset = 0, mag_y_offset = 0;
float mag_x_scale = 1.0, mag_y_scale = 1.0;

// Autocorrelation Pedometer
bool system_stabilized = false;           // Flag to prevent false steps during startup

// SOS Emergency System
bool sos_active = false;                   // SOS signal active flag
unsigned long sos_start_time = 0;         // When SOS was triggered
const unsigned long SOS_DURATION = 8000;  // 8 seconds SOS display duration
bool sos_triggered = false;               // Flag to track if SOS was sent

// Vertical Tracking System (BMP180 Altitude Monitoring)
struct VerticalTracker {
  float zero_altitude;                     // Reference altitude (entrance/ground level)
  float current_altitude;                  // Current altitude reading
  float filtered_altitude;                 // Noise-filtered altitude
  float depth_change;                      // Depth/height change from zero point
  float max_depth;                         // Maximum depth reached
  float max_height;                        // Maximum height reached
  bool zero_point_set;                     // Has zero point been established
  bool monitoring_active;                  // Is vertical tracking active
  unsigned long last_update;               // Last altitude update time
  
  // Noise filtering
  float altitude_buffer[10];               // Rolling average buffer
  int buffer_index;                        // Current buffer position
  float noise_threshold;                   // Minimum change to register
  
  // Cave/building floor detection
  int estimated_floor;                     // Estimated floor level (+ above ground, - below)
  float floor_height;                      // Standard floor height (3m default)
  bool is_underground;                     // Currently below zero point
  unsigned long time_underground;          // Time spent below ground
};

VerticalTracker vertical_tracker = {
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false, false, 0,
  {0}, 0, 0.1,  // 10cm noise threshold
  0, 3.0, false, 0
};

#define SAMPLE_RATE_HZ 50
#define SAMPLE_PERIOD_MS (1000 / SAMPLE_RATE_HZ)
#define ACCEL_BUFFER_SIZE 128
float accel_mag_buffer[ACCEL_BUFFER_SIZE] = {0};
int accel_buffer_index = 0;
unsigned long last_sample_time = 0;
float current_cadence_hz = 0;
float step_count = 0;
float prev_step_int = 0;
const int MIN_LAG = 20, MAX_LAG = 100;
const float AUTOCORR_THRESHOLD = 0.3; // Lowered for better step detection
unsigned long last_autocorr_time = 0;

// Path and Position - Enhanced
PathPoint pathPoints[MAX_PATH_POINTS];  // Enhanced path storage
int pathIndex = 0, path_points_count = 0;
float currentX = 0, currentY = 0, currentZ = 0;
float initial_altitude = 0;

// Advanced Map Visualization
ViewPort viewport = {0, 0, 1.0, 0, true};  // centerX, centerY, zoom, mode, auto_center
uint8_t current_zoom_level = 1;  // Index into zoom_scales array
float trail_points_x[TRAIL_LENGTH], trail_points_y[TRAIL_LENGTH];
uint8_t trail_index = 0;
unsigned long last_position_update = 0;
float total_distance = 0.0;  // Total path distance for statistics

// ----- Kalman Filter Variables -----
KalmanFilter kalman_x, kalman_y;  // Position filters for X and Y
float filtered_x = 0, filtered_y = 0;  // Kalman-filtered positions
bool kalman_initialized = false;

// ----- Smart Waypoint Variables -----
SmartWaypoint waypoints[MAX_WAYPOINTS];
int waypoint_count = 0;
int nearest_waypoint_index = -1;
float distance_to_nearest_waypoint = 999.0;
bool waypoint_notification_shown = false;
unsigned long last_waypoint_check = 0;

const char* waypoint_type_names[] = {
  "Generic", "Camp", "Water", "Food", "Shelter", 
  "View", "Danger", "Photo", "Meeting", "Exit"
};

const char* waypoint_icons[] = {
  "📍", "🏕️", "💧", "🍎", "🏠", 
  "👁️", "⚠️", "📷", "🤝", "🚪"
};

// Display & Timing
enum DisplayMode { INFO_SCREEN, MAP_SCREEN };
DisplayMode current_mode = INFO_SCREEN;
unsigned long startup_time = 0;
const unsigned long INFO_SCREEN_DURATION = 10000;
unsigned long previous_time_ms = 0;
unsigned long display_update_time = 0;

// ----- Enhanced Reverse Navigation Variables -----
bool button_pressed = false;
unsigned long last_button_time = 0;
float start_position_x = 0, start_position_y = 0, start_position_z = 0;

// ----- 3D Dashboard Variables -----
DashboardState dashboard = {DASHBOARD_OFF, 0, 0, 0.0, 0, false};
SensorReadings sensors = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
unsigned long first_click_time = 0;
bool waiting_for_second_click = false;
int click_count = 0;
unsigned long button_press_start = 0;
bool button_currently_pressed = false;

// Forward declarations
void calibrateGyro();
void calibrateCompass();
void handleButtonPress();
void initializeBreadcrumbNavigation();
void updateBreadcrumbNavigation();
void calculateNextBreadcrumb();
void drawEnhancedReverseDisplay();
void updateSensorReadings();
void update3DDashboard();
void draw3DDashboard();
void drawDirectionalArrow(int x, int y, float heading, uint8_t size);
void drawCompassRose(int x, int y);
void drawPathWithEffects();
void drawStartPosition();
void drawCurrentPositionWithEffects();
void drawInformationOverlay();
void drawAdvancedMap();
void displayInfoScreen();
float calculateLocalAreaSize();
void initializeKalmanFilters();
void updateKalmanFilter(KalmanFilter* kf, float measurement, float dt);
void updateFilteredPosition();
void calibrateAltitude();
void updateEnhancedAltitude(float dt);
float getAccelerometerAltitude();
int addWaypoint(float x, float y, float z, const char* name, WaypointType type);
void checkNearbyWaypoints();
void displayWaypointNotification();
void setupWiFi();
void setupWebServer();
void handleRoot();
void handleJavaScript();
void handleData();
void handlePath();
void handleSensors();
void handleStatus();
void handleWaypoints();
void handleAddWaypoint();
void handleDeleteWaypoint();
void handleToggleReverse();
void handleTriggerSOS();

// Vertical Tracking Functions
void initializeVerticalTracking();
void updateVerticalTracking();
void setAltitudeZeroPoint();
float filterAltitudeNoise(float raw_altitude);
void drawVerticalTrackingDisplay();
int estimateFloorLevel(float depth_change);

// #################################################################################################
// # 1. SETUP
// #################################################################################################

void setup() {
  Serial.begin(115200);
  
  // --- Initialize I2C bus ---
  Wire.begin(); // For all sensors on default I2C pins

  // --- Initialize GPIO for reverse button ---
  pinMode(REVERSE_BUTTON_PIN, INPUT_PULLUP);

  // Initialize OLED Display (uses default 'Wire' bus)
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (1);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Initializing...");
  display.display();

  // Initialize MPU6050 on the primary 'Wire' bus with retry mechanism
  int mpu_retry_count = 0;
  while (!mpu.begin(0x69, &Wire) && mpu_retry_count < 5) {
    mpu_retry_count++;
    display.clearDisplay(); 
    display.setCursor(0,0);
    display.print("MPU6050 retry: ");
    display.print(mpu_retry_count);
    display.display();
    delay(1000);
  }
  
  if (mpu_retry_count >= 5) {
    display.clearDisplay(); 
    display.setCursor(0,0);
    display.println("MPU6050 FAILED!");
    display.println("Continuing without");
    display.println("step detection...");
    display.display();
    delay(3000);
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  // Initialize QMC5883L compass
  compass.init();
  // Set magnetic declination: -0.72 degrees = -1 degrees, 17 minutes (0.28 * 60 = 16.8)
  compass.setMagneticDeclination(-1, 17);

  // --- Run Calibration Routines ---
  calibrateGyro();
  calibrateCompass(); // This is a blocking routine with user interaction

  // Initialize BMP180 on the primary 'Wire' bus
  if (bmp.begin(BMP085_ULTRAHIGHRES, &Wire)) {
    bmp_found = true;
    initial_altitude = bmp.readAltitude();
    calibrateAltitude();
  }

  // Initialize timing and path
  previous_time_ms = millis();
  startup_time = millis();
  
  // Initialize Kalman filters for position smoothing
  initializeKalmanFilters();
  
  // Initialize vertical tracking system
  initializeVerticalTracking();
  
  addBreadcrumb(); // Add starting point (0,0,0)
  
  // Store starting position for return navigation
  start_position_x = currentX;
  start_position_y = currentY;
  start_position_z = currentZ;
  
  // Initialize WiFi and Web Server
  setupWiFi();
  setupWebServer();
}

// #################################################################################################
// # 2. MAIN LOOP
// #################################################################################################

void loop() {
  static unsigned long watchdog_timer = 0;
  unsigned long current_time_ms = millis();
  
  // Simple watchdog - reset if loop takes too long
  if (current_time_ms - watchdog_timer > 5000) { // 5 second watchdog
    Serial.println("⚠️ Loop watchdog triggered - potential hang detected");
    watchdog_timer = current_time_ms;
  }
  
  float dt = (current_time_ms - previous_time_ms) / 1000.0;
  previous_time_ms = current_time_ms;

  // Check if system has stabilized (prevents false steps at startup)
  if (!system_stabilized && (current_time_ms - startup_time > STARTUP_STABILIZATION_TIME)) {
    system_stabilized = true;
    Serial.println("System stabilized - step detection enabled");
  }

  // Handle web server clients
  server.handleClient();

  // Handle button press for reverse navigation
  handleButtonPress();

  // Read data from all sensors on their respective buses
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  compass.read();

  // Process data with higher frequency for better tracking
  updateOrientation(a, g, dt);
  
  // Update pedometer with reduced lag (only after stabilization)
  if (system_stabilized && current_time_ms - last_sample_time >= SAMPLE_PERIOD_MS) {
    processPedometer(a, dt);
    last_sample_time = current_time_ms; // CRITICAL FIX: Update sample time
  }

  // Update reverse navigation if in reverse mode
  if (nav_mode == REVERSE_MODE) {
    updateBreadcrumbNavigation();
    // Removed blocking debug print that could cause issues
  }

  // Update Kalman filtered position
  updateFilteredPosition();
  
  // Update enhanced altitude measurement
  updateEnhancedAltitude(dt);
  
  // Update vertical tracking system
  updateVerticalTracking();
  
  // Check for nearby waypoints
  checkNearbyWaypoints();

  // Update sensor readings for dashboard
  updateSensorReadings();
  
  // Update 3D dashboard system
  update3DDashboard();

  // Update display mode
  if (current_mode == INFO_SCREEN && (current_time_ms - startup_time > INFO_SCREEN_DURATION)) {
    current_mode = MAP_SCREEN;
  }

  // Update display with error handling
  if (current_time_ms - display_update_time > 100) {
    // Check for SOS emergency display first
    if (isSOSActive()) {
      updateSOSDisplay();
    } else if (dashboard.current_mode != DASHBOARD_OFF) {
      draw3DDashboard();
    } else if (current_mode == INFO_SCREEN) {
      displayInfoScreen();
    } else {
      if (nav_mode == REVERSE_MODE) drawEnhancedReverseDisplay();
      else drawAdvancedMap();  // Use normal advanced map function
    }
    display_update_time = current_time_ms;
  }
  
  // Update watchdog timer at end of loop
  watchdog_timer = current_time_ms;
  
  // Small delay to prevent overwhelming the system
  yield(); // Allow other tasks to run
}

// #################################################################################################
// # 3. SENSOR CALIBRATION & FUSION
// #################################################################################################

void calibrateGyro() {
  display.clearDisplay(); display.setCursor(0, 0);
  display.println("Calibrating Gyro");
  display.println("Keep stationary..."); display.display();
  
  double gx_sum=0, gy_sum=0, gz_sum=0; // Use double to prevent overflow
  sensors_event_t a, g, temp;
  const int num_samples = 2000;
  for (int i = 0; i < num_samples; i++) {
    mpu.getEvent(&a, &g, &temp);
    gx_sum += g.gyro.x; gy_sum += g.gyro.y; gz_sum += g.gyro.z;
    delay(1);
  }
  gyro_x_offset = (float)gx_sum / num_samples;
  gyro_y_offset = (float)gy_sum / num_samples;
  gyro_z_offset = (float)gz_sum / num_samples;
}

void calibrateCompass() {
  display.clearDisplay(); display.setCursor(0, 0);
  display.println("Calibrate Compass");
  display.println("Follow directions:");
  display.println("1.Flat figure-8 (5s)");
  display.println("2.Vertical circles(5s)");
  display.println("3.Tilt all angles(5s)");
  display.display();

  unsigned long cal_start_time = millis();
  int phase = 1;
  
  while(millis() - cal_start_time < 15000) {
    compass.read(); // Reads from default Wire bus
    int x = compass.getX();
    int y = compass.getY();
    if (x < mag_x_min) mag_x_min = x;
    if (x > mag_x_max) mag_x_max = x;
    if (y < mag_y_min) mag_y_min = y;
    if (y > mag_y_max) mag_y_max = y;
    
    // Update phase instructions
    unsigned long elapsed = millis() - cal_start_time;
    int new_phase = (elapsed / 5000) + 1;
    if (new_phase != phase) {
      phase = new_phase;
      display.fillRect(0, 40, 128, 24, SSD1306_BLACK);
      display.setCursor(0, 40);
      if (phase == 1) {
        display.println("FLAT FIGURE-8");
        display.println("Move horizontally");
      } else if (phase == 2) {
        display.println("VERTICAL CIRCLES");
        display.println("Rotate vertically");
      } else {
        display.println("TILT ALL ANGLES");
        display.println("Tip in all ways");
      }
      display.display();
    }
    
    // Show countdown
    display.fillRect(90, 56, 38, 8, SSD1306_BLACK);
    display.setCursor(90, 56);
    display.print("T:");
    display.print(15 - elapsed/1000);
    display.print("s");
    display.display();
    delay(50);
  }

  mag_x_offset = (mag_x_max + mag_x_min) / 2.0;
  mag_y_offset = (mag_y_max + mag_y_min) / 2.0;
  
  float avg_delta_x = (mag_x_max - mag_x_min) / 2.0;
  float avg_delta_y = (mag_y_max - mag_y_min) / 2.0;
  float avg_delta = (avg_delta_x + avg_delta_y) / 2.0;

  // Prevent division by zero
  if (avg_delta_x > 0.001) {
    mag_x_scale = avg_delta / avg_delta_x;
  } else {
    mag_x_scale = 1.0;
  }
  
  if (avg_delta_y > 0.001) {
    mag_y_scale = avg_delta / avg_delta_y;
  } else {
    mag_y_scale = 1.0;
  }
  
  // Show completion
  display.clearDisplay();
  display.setCursor(0, 20);
  display.println("Calibration Complete!");
  display.println("Range X:"); display.print(mag_x_max - mag_x_min);
  display.println(" Y:"); display.print(mag_y_max - mag_y_min);
  display.display();
  delay(2000);
}

void updateOrientation(sensors_event_t accel_event, sensors_event_t gyro_event, float dt) {
  // --- Step 1: Calculate Roll and Pitch from MPU6050 ---
  float gyro_x = gyro_event.gyro.x - gyro_x_offset;
  float gyro_y = gyro_event.gyro.y - gyro_y_offset;
  
  // Prevent division by zero in angle calculations
  float denom_y = sqrt(pow(accel_event.acceleration.y, 2) + pow(accel_event.acceleration.z, 2));
  float denom_x = sqrt(pow(accel_event.acceleration.x, 2) + pow(accel_event.acceleration.z, 2));
  
  float accel_angle_y = 0, accel_angle_x = 0;
  if (denom_y > 0.001) {
    accel_angle_y = atan(-1 * accel_event.acceleration.x / denom_y) * (180.0 / PI);
  }
  if (denom_x > 0.001) {
    accel_angle_x = atan(accel_event.acceleration.y / denom_x) * (180.0 / PI);
  }

  roll = ALPHA * (roll + gyro_x * dt) + (1.0 - ALPHA) * accel_angle_x;
  pitch = ALPHA * (pitch + gyro_y * dt) + (1.0 - ALPHA) * accel_angle_y;

  // --- Step 2: Get Calibrated & Tilt-Compensated Heading from QMC5883L ---
  float mag_x = (compass.getX() - mag_x_offset) * mag_x_scale;
  float mag_y = (compass.getY() - mag_y_offset) * mag_y_scale;

  float roll_rad = roll * PI / 180.0;
  float pitch_rad = pitch * PI / 180.0;

  float comp_mag_x = mag_x * cos(pitch_rad) + compass.getZ() * sin(pitch_rad);
  float comp_mag_y = mag_x * sin(roll_rad) * sin(pitch_rad) + mag_y * cos(roll_rad) - compass.getZ() * sin(roll_rad) * cos(pitch_rad);

  float heading = atan2(comp_mag_y, comp_mag_x) * 180.0 / PI;
  heading += MAGNETIC_DECLINATION;
  if (heading < 0) heading += 360;
  
  yaw = heading;
}

// #################################################################################################
// # 4. KALMAN FILTER IMPLEMENTATION
// #################################################################################################

void initializeKalmanFilters() {
  // Initialize X-axis Kalman filter
  kalman_x.x_est = 0.0;
  kalman_x.x_vel = 0.0;
  kalman_x.P[0][0] = 1.0; kalman_x.P[0][1] = 0.0;
  kalman_x.P[1][0] = 0.0; kalman_x.P[1][1] = 1.0;
  kalman_x.Q = 0.1;  // Process noise (tunable)
  kalman_x.R = 0.5;  // Measurement noise (tunable)
  kalman_x.last_update = millis();
  
  // Initialize Y-axis Kalman filter  
  kalman_y.x_est = 0.0;
  kalman_y.x_vel = 0.0;
  kalman_y.P[0][0] = 1.0; kalman_y.P[0][1] = 0.0;
  kalman_y.P[1][0] = 0.0; kalman_y.P[1][1] = 1.0;
  kalman_y.Q = 0.1;  // Process noise
  kalman_y.R = 0.5;  // Measurement noise
  kalman_y.last_update = millis();
  
  kalman_initialized = true;
  Serial.println("Kalman filters initialized for position smoothing");
}

void updateKalmanFilter(KalmanFilter* kf, float measurement, float dt) {
  if (dt <= 0) return;  // Prevent invalid time steps
  
  // Prediction step
  float x_pred = kf->x_est + kf->x_vel * dt;
  float v_pred = kf->x_vel;
  
  // Predict error covariance
  float P_pred[2][2];
  P_pred[0][0] = kf->P[0][0] + kf->P[0][1] * dt + kf->P[1][0] * dt + kf->P[1][1] * dt * dt + kf->Q;
  P_pred[0][1] = kf->P[0][1] + kf->P[1][1] * dt;
  P_pred[1][0] = kf->P[1][0] + kf->P[1][1] * dt;
  P_pred[1][1] = kf->P[1][1] + kf->Q;
  
  // Update step
  float innovation = measurement - x_pred;
  float S = P_pred[0][0] + kf->R;  // Innovation covariance
  
  // Kalman gain
  float K[2];
  K[0] = P_pred[0][0] / S;
  K[1] = P_pred[1][0] / S;
  
  // Update estimate
  kf->x_est = x_pred + K[0] * innovation;
  kf->x_vel = v_pred + K[1] * innovation;
  
  // Update error covariance
  kf->P[0][0] = (1.0 - K[0]) * P_pred[0][0];
  kf->P[0][1] = (1.0 - K[0]) * P_pred[0][1];
  kf->P[1][0] = P_pred[1][0] - K[1] * P_pred[0][0];
  kf->P[1][1] = P_pred[1][1] - K[1] * P_pred[0][1];
}

void updateFilteredPosition() {
  if (!kalman_initialized) return;
  
  unsigned long current_time = millis();
  float dt_x = (current_time - kalman_x.last_update) / 1000.0;
  float dt_y = (current_time - kalman_y.last_update) / 1000.0;
  
  // Update filters with raw position measurements
  updateKalmanFilter(&kalman_x, currentX, dt_x);
  updateKalmanFilter(&kalman_y, currentY, dt_y);
  
  // Store filtered positions
  filtered_x = kalman_x.x_est;
  filtered_y = kalman_y.x_est;
  
  kalman_x.last_update = current_time;
  kalman_y.last_update = current_time;
}

void calibrateAltitude() {
  if (!bmp_found) return;
  
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Calibrating Altitude");
  display.println("Keep device still...");
  display.display();
  
  // Take multiple readings to establish baseline
  float pressure_sum = 0;
  float altitude_sum = 0;
  const int samples = 50;
  
  for (int i = 0; i < samples; i++) {
    pressure_sum += bmp.readPressure();
    altitude_sum += bmp.readAltitude();
    delay(20);
  }
  
  alt_filter.baseline_pressure = pressure_sum / samples;
  alt_filter.altitude_offset = altitude_sum / samples;
  alt_filter.filtered_altitude = 0; // Start from zero reference
  alt_filter.is_calibrated = true;
  alt_filter.last_update = millis();
  
  Serial.print("Altitude calibrated. Baseline pressure: ");
  Serial.print(alt_filter.baseline_pressure);
  Serial.println(" Pa");
}

void updateEnhancedAltitude(float dt) {
  if (!bmp_found || !alt_filter.is_calibrated) return;
  
  unsigned long current_time = millis();
  if (current_time - alt_filter.last_update < 100) return; // Update every 100ms
  
  // Get barometric altitude (relative to starting point)
  float current_pressure = bmp.readPressure();
  // Use standard atmosphere formula: altitude = 44330 * (1 - (P/P0)^0.1903)
  float pressure_ratio = current_pressure / alt_filter.baseline_pressure;
  alt_filter.barometric_altitude = 44330.0 * (1.0 - pow(pressure_ratio, 0.1903));
  
  // Get accelerometer-based altitude estimate
  alt_filter.accelerometer_altitude = getAccelerometerAltitude();
  
  // Fusion: Use barometric for slow changes, accelerometer for fast changes
  float weight_baro = 0.7; // Trust barometric more for long-term accuracy
  float weight_accel = 0.3; // Use accelerometer for short-term responsiveness
  
  // Apply complementary filter
  float new_altitude = weight_baro * alt_filter.barometric_altitude + 
                      weight_accel * alt_filter.accelerometer_altitude;
  
  // Smooth the result
  alt_filter.filtered_altitude = 0.8 * alt_filter.filtered_altitude + 0.2 * new_altitude;
  
  // Update current Z position
  currentZ = alt_filter.filtered_altitude;
  
  alt_filter.last_update = current_time;
}

float getAccelerometerAltitude() {
  // This is a simplified accelerometer-based altitude estimation
  // In practice, this would require double integration of vertical acceleration
  // For now, we'll use the pitch-based estimation as a backup
  
  static float accel_altitude = 0;
  static unsigned long last_accel_time = 0;
  
  unsigned long current_time = millis();
  float dt = (current_time - last_accel_time) / 1000.0;
  
  if (dt > 0) {
    // Use pitch to estimate vertical movement during steps
    float vertical_component = STEP_LENGTH_METERS * sin(pitch * (PI / 180.0));
    
    // Accumulate vertical movement (simplified)
    if (current_cadence_hz > 0) {
      accel_altitude += vertical_component * current_cadence_hz * dt;
    }
    
    // Apply decay to prevent drift
    accel_altitude *= 0.99;
  }
  
  last_accel_time = current_time;
  return accel_altitude;
}

// #################################################################################################
// # 5. SMART WAYPOINT SYSTEM
// #################################################################################################

int addWaypoint(float x, float y, float z, const char* name, WaypointType type) {
  if (waypoint_count >= MAX_WAYPOINTS) return -1;
  
  SmartWaypoint* wp = &waypoints[waypoint_count];
  wp->x = x;
  wp->y = y;
  wp->z = z;
  strncpy(wp->name, name, 15);
  wp->name[15] = '\0';
  wp->type = type;
  wp->timestamp = millis();
  wp->active = true;
  wp->icon_id = (uint8_t)type;
  
  waypoint_count++;
  
  Serial.print("Waypoint added: ");
  Serial.print(wp->name);
  Serial.print(" at (");
  Serial.print(wp->x); Serial.print(", ");
  Serial.print(wp->y); Serial.println(")");
  
  return waypoint_count - 1;
}

void checkNearbyWaypoints() {
  unsigned long current_time = millis();
  if (current_time - last_waypoint_check < 2000) return;  // Check every 2 seconds
  
  nearest_waypoint_index = -1;
  distance_to_nearest_waypoint = 999.0;
  
  for (int i = 0; i < waypoint_count; i++) {
    if (!waypoints[i].active) continue;
    
    float dx = waypoints[i].x - currentX;
    float dy = waypoints[i].y - currentY;
    float distance = sqrt(dx * dx + dy * dy);
    
    if (distance < distance_to_nearest_waypoint) {
      distance_to_nearest_waypoint = distance;
      nearest_waypoint_index = i;
    }
  }
  
  last_waypoint_check = current_time;
}

void displayWaypointNotification() {
  if (nearest_waypoint_index >= 0 && distance_to_nearest_waypoint < WAYPOINT_PROXIMITY_RADIUS) {
    SmartWaypoint* wp = &waypoints[nearest_waypoint_index];
    
    // Show notification on OLED (bottom line)
    display.fillRect(0, 56, SCREEN_WIDTH, 8, SSD1306_BLACK);
    display.setCursor(2, 56);
    display.setTextSize(1);
    display.print("Near: ");
    display.print(wp->name);
    display.print(" ");
    display.print((int)distance_to_nearest_waypoint);
    display.print("m");
  }
}

// #################################################################################################
// # 6. PEDOMETER & POSITIONING (Enhanced with Kalman Filter)
// #################################################################################################

void processPedometer(sensors_event_t accel_event, float dt) {
  unsigned long current_time_ms = millis();

  // Add accelerometer sample to buffer only after system is stabilized
  if (!system_stabilized) {
    // Clear buffer during stabilization to prevent false readings
    for (int i = 0; i < ACCEL_BUFFER_SIZE; i++) {
      accel_mag_buffer[i] = 0;
    }
    accel_buffer_index = 0;
    current_cadence_hz = 0;
    return;
  }

  float mag = sqrt(pow(accel_event.acceleration.x, 2) + pow(accel_event.acceleration.y, 2) + pow(accel_event.acceleration.z, 2));
  accel_mag_buffer[accel_buffer_index] = mag;
  accel_buffer_index = (accel_buffer_index + 1) % ACCEL_BUFFER_SIZE;

  // Run autocorrelation more frequently for faster detection
  if (current_time_ms - last_autocorr_time > 250) { // Reduced from 500ms
    last_autocorr_time = current_time_ms;
    runAutocorrelation();
  }

  if (current_cadence_hz > 0.1) { // Restored to original sensitive threshold
    step_count += current_cadence_hz * dt;
    if (floor(step_count) > prev_step_int) {
      prev_step_int = floor(step_count);
      updatePosition();
      
      // Only add breadcrumbs in FORWARD mode, not during reverse navigation
      if (nav_mode == FORWARD_MODE) {
        addBreadcrumb();
      }
    }
  } else {
    // Reset cadence if below threshold to prevent drift
    current_cadence_hz = 0;
    
    // Decay mechanism: slowly reduce step count when no movement detected
    // This prevents accumulated false steps from persisting
    if (current_time_ms - last_autocorr_time > 3000) { // Increased to 3 seconds for less aggressive decay
      if (step_count > prev_step_int + 0.1) { // If there are fractional steps accumulated
        step_count = prev_step_int; // Reset to last integer step count
      }
    }
  }
}

void runAutocorrelation() {
  // Don't run autocorrelation during system stabilization
  if (!system_stabilized) {
    current_cadence_hz = 0;
    return;
  }

  float temp_buffer[ACCEL_BUFFER_SIZE];
  float mean = 0;
  for (int i = 0; i < ACCEL_BUFFER_SIZE; i++) {
    int source_index = (accel_buffer_index + i) % ACCEL_BUFFER_SIZE;
    temp_buffer[i] = accel_mag_buffer[source_index];
    mean += temp_buffer[i];
  }
  mean /= ACCEL_BUFFER_SIZE;
  for (int i = 0; i < ACCEL_BUFFER_SIZE; i++) temp_buffer[i] -= mean;

  float max_corr = 0;
  int best_lag = 0;
  for (int lag = MIN_LAG; lag < MAX_LAG; lag++) {
    float corr = 0;
    for (int i = 0; i < ACCEL_BUFFER_SIZE - lag; i++) {
      corr += temp_buffer[i] * temp_buffer[i + lag];
    }
    if (corr > max_corr) {
      max_corr = corr;
      best_lag = lag;
    }
  }
  
  float self_corr_zero_lag = 0;
  for (int i = 0; i < ACCEL_BUFFER_SIZE; i++) self_corr_zero_lag += temp_buffer[i] * temp_buffer[i];
  float normalized_corr = (self_corr_zero_lag > 0) ? (max_corr / self_corr_zero_lag) : 0;

  if (normalized_corr > AUTOCORR_THRESHOLD && best_lag > 0) {
    float detected_cadence = (float)SAMPLE_RATE_HZ / best_lag;
    
    // Additional validation to prevent false detection:
    // 1. Reasonable cadence range (0.5 to 4 Hz = 30-240 steps/min)
    // 2. Consistent detection over multiple cycles
    if (detected_cadence >= 0.5 && detected_cadence <= 4.0) {
      current_cadence_hz = detected_cadence;
      // Debug output for step detection (non-blocking)
      static unsigned long last_debug_time = 0;
      if (millis() - last_debug_time > 1000) { // Only print once per second
        Serial.print("🚶 Step detected! Cadence: ");
        Serial.print(current_cadence_hz * 60, 1);
        Serial.print(" steps/min, Correlation: ");
        Serial.println(normalized_corr, 3);
        last_debug_time = millis();
      }
    } else {
      current_cadence_hz = 0; // Reject unrealistic cadence
    }
  } else {
    current_cadence_hz = 0;
  }
}

void updatePosition() {
    float yaw_rad = yaw * (PI / 180.0);
    float prev_x = currentX, prev_y = currentY;
    
    currentX += STEP_LENGTH_METERS * sin(yaw_rad);
    currentY += STEP_LENGTH_METERS * cos(yaw_rad);
    
    if (bmp_found) {
        currentZ = bmp.readAltitude() - initial_altitude;
    } else {
        currentZ += STEP_LENGTH_METERS * sin(pitch * (PI / 180.0));
    }
    
    // Calculate distance for this step
    float step_distance = sqrt(pow(currentX - prev_x, 2) + pow(currentY - prev_y, 2));
    total_distance += step_distance;
    
    // Update trail ONLY in forward mode - freeze trail during reverse navigation
    if (nav_mode == FORWARD_MODE) {
        trail_points_x[trail_index] = currentX;
        trail_points_y[trail_index] = currentY;
        trail_index = (trail_index + 1) % TRAIL_LENGTH;
    }
    
    last_position_update = millis();
}

void addBreadcrumb() {
  // Don't add breadcrumbs during reverse navigation - path should be frozen
  if (nav_mode == REVERSE_MODE) {
    Serial.println("🚫 Breadcrumb skipped - in REVERSE mode (path frozen)");
    return;
  }
  
  // Calculate speed level based on current cadence
  uint8_t speed = 0;
  if (current_cadence_hz > 0) {
    speed = constrain((int)(current_cadence_hz * 4), 1, 7);  // Map 0-2Hz to 1-7
  }
  
  // Store enhanced path point
  pathPoints[pathIndex].x = currentX;
  pathPoints[pathIndex].y = currentY;
  pathPoints[pathIndex].z = currentZ;
  pathPoints[pathIndex].heading = yaw;
  pathPoints[pathIndex].speed_level = speed;
  pathPoints[pathIndex].timestamp = millis();
  
  pathIndex = (pathIndex + 1) % MAX_PATH_POINTS;
  if (path_points_count < MAX_PATH_POINTS) {
    path_points_count++;
  }
  
  // Update viewport center if auto-centering is enabled
  if (viewport.auto_center) {
    viewport.centerX = currentX;
    viewport.centerY = currentY;
  }
}

// #################################################################################################
// # 5. DISPLAY & VISUALIZATION - Advanced Implementation
// #################################################################################################

void calculateOptimalViewport() {
  if (path_points_count < 2) return;
  
  int start_index = (path_points_count == MAX_PATH_POINTS) ? pathIndex : 0;
  float minX = pathPoints[start_index].x, maxX = pathPoints[start_index].x;
  float minY = pathPoints[start_index].y, maxY = pathPoints[start_index].y;
  
  // Find bounds of path
  for (int i = 1; i < path_points_count; i++) {
    int idx = (start_index + i) % MAX_PATH_POINTS;
    if (pathPoints[idx].x < minX) minX = pathPoints[idx].x;
    if (pathPoints[idx].x > maxX) maxX = pathPoints[idx].x;
    if (pathPoints[idx].y < minY) minY = pathPoints[idx].y;
    if (pathPoints[idx].y > maxY) maxY = pathPoints[idx].y;
  }
  
  float pathWidth = maxX - minX;
  float pathHeight = maxY - minY;
  
  // Navigation-focused zoom logic based on mode and context
  if (nav_mode == REVERSE_MODE) {
    // REVERSE MODE: Always use maximum detail for precise navigation
    current_zoom_level = 4; // Maximum zoom (8.0x) for breadcrumb following
    viewport.zoom_mode = 1; // Recent focus mode
    viewport.centerX = currentX;
    viewport.centerY = currentY;
    Serial.println("Reverse mode: Maximum zoom for precision navigation");
    
  } else if (viewport.zoom_mode == 0) {  // Auto mode for forward navigation
    // Dynamic zoom based on movement speed and local area density
    float recent_area_size = calculateLocalAreaSize(); // Look at last few waypoints
    
    if (recent_area_size < 3) {
      current_zoom_level = 3; // Detail view (4.0x) for tight spaces
    } else if (recent_area_size < 8) {
      current_zoom_level = 2; // Close view (2.0x) for normal walking
    } else if (recent_area_size < 25) {
      current_zoom_level = 1; // Normal view (1.0x) for open areas
    } else {
      current_zoom_level = 0; // Wide view (0.5x) only for very large open areas
    }
    
    // Auto-center on current position for navigation
    viewport.centerX = currentX;
    viewport.centerY = currentY;
    
  } else if (viewport.zoom_mode == 1) {  // Recent focus - follow current position
    // Keep current zoom level but follow position
    viewport.centerX = currentX;
    viewport.centerY = currentY;
    
  } else if (viewport.zoom_mode == 2) {  // Full path overview
    // Only use this for planning - auto-fit entire path
    viewport.centerX = (minX + maxX) / 2;
    viewport.centerY = (minY + maxY) / 2;
    
    // Intelligent zoom to fit path with minimum usable detail
    float max_dimension = max(pathWidth, pathHeight);
    if (max_dimension < 10) current_zoom_level = 2;      // Close view minimum
    else if (max_dimension < 30) current_zoom_level = 1;  // Normal view
    else current_zoom_level = 0;                         // Wide view only when necessary
  }
  
  viewport.zoom_level = zoom_scales[current_zoom_level];
  viewport.auto_center = (viewport.zoom_mode <= 1); // Auto-center for navigation modes
}

// Calculate the size of the local area around current position
float calculateLocalAreaSize() {
  if (path_points_count < 5) return 5.0; // Default for small paths
  
  // Look at last 8 waypoints to determine local movement area
  int recent_count = min(8, path_points_count);
  float local_minX = currentX, local_maxX = currentX;
  float local_minY = currentY, local_maxY = currentY;
  
  for (int i = 0; i < recent_count; i++) {
    int idx = (path_points_count == MAX_PATH_POINTS) ? 
              (pathIndex - i + MAX_PATH_POINTS) % MAX_PATH_POINTS : 
              max(0, path_points_count - 1 - i);
    
    if (pathPoints[idx].x < local_minX) local_minX = pathPoints[idx].x;
    if (pathPoints[idx].x > local_maxX) local_maxX = pathPoints[idx].x;
    if (pathPoints[idx].y < local_minY) local_minY = pathPoints[idx].y;
    if (pathPoints[idx].y > local_maxY) local_maxY = pathPoints[idx].y;
  }
  
  float localWidth = local_maxX - local_minX;
  float localHeight = local_maxY - local_minY;
  return max(localWidth, localHeight);
}

void drawDirectionalArrow(int x, int y, float heading, uint8_t size) {
  float head_rad = heading * PI / 180.0;
  int dx = (int)(size * sin(head_rad));
  int dy = (int)(size * cos(head_rad));
  
  // Draw main arrow line
  display.drawLine(x - dx/2, y + dy/2, x + dx/2, y - dy/2, SSD1306_WHITE);
  
  // Draw arrowhead
  int arrow_size = size / 3;
  float arrow_angle1 = head_rad + 2.5;
  float arrow_angle2 = head_rad - 2.5;
  
  int ax1 = x + dx/2 - (int)(arrow_size * sin(arrow_angle1));
  int ay1 = y - dy/2 + (int)(arrow_size * cos(arrow_angle1));
  int ax2 = x + dx/2 - (int)(arrow_size * sin(arrow_angle2));
  int ay2 = y - dy/2 + (int)(arrow_size * cos(arrow_angle2));
  
  display.drawLine(x + dx/2, y - dy/2, ax1, ay1, SSD1306_WHITE);
  display.drawLine(x + dx/2, y - dy/2, ax2, ay2, SSD1306_WHITE);
}

void drawCompassRose(int x, int y) {
  // Draw compass circle
  display.drawCircle(x, y, COMPASS_SIZE/2, SSD1306_WHITE);
  
  // Draw north indicator
  float north_rad = (yaw - 90) * PI / 180.0;  // Relative to current heading
  int nx = x + (int)((COMPASS_SIZE/2 - 2) * cos(north_rad));
  int ny = y + (int)((COMPASS_SIZE/2 - 2) * sin(north_rad));
  display.drawLine(x, y, nx, ny, SSD1306_WHITE);
  display.fillCircle(nx, ny, 1, SSD1306_WHITE);
}

void drawPathWithEffects() {
  if (path_points_count < 2) return;
  
  calculateOptimalViewport();
  
  int start_index = (path_points_count == MAX_PATH_POINTS) ? pathIndex : 0;
  
  // Calculate scaling for current viewport
  float view_width = (SCREEN_WIDTH - 20) / viewport.zoom_level;
  float view_height = (SCREEN_HEIGHT - 20) / viewport.zoom_level;
  
  float scale = min((SCREEN_WIDTH - 20) / view_width, (SCREEN_HEIGHT - 20) / view_height);
  
  // Draw path segments with varying thickness and effects
  for (int i = 0; i < path_points_count - 1; i++) {
    int idx1 = (start_index + i) % MAX_PATH_POINTS;
    int idx2 = (start_index + i + 1) % MAX_PATH_POINTS;
    
    // Calculate screen coordinates
    int sx1 = (int)((pathPoints[idx1].x - viewport.centerX) * scale * viewport.zoom_level) + SCREEN_WIDTH/2;
    int sy1 = (int)((pathPoints[idx1].y - viewport.centerY) * scale * viewport.zoom_level) + SCREEN_HEIGHT/2;
    int sx2 = (int)((pathPoints[idx2].x - viewport.centerX) * scale * viewport.zoom_level) + SCREEN_WIDTH/2;
    int sy2 = (int)((pathPoints[idx2].y - viewport.centerY) * scale * viewport.zoom_level) + SCREEN_HEIGHT/2;
    
    // Skip if outside screen bounds (culling)
    if ((sx1 < -5 || sx1 > SCREEN_WIDTH + 5) && (sx2 < -5 || sx2 > SCREEN_WIDTH + 5)) continue;
    if ((sy1 < -5 || sy1 > SCREEN_HEIGHT + 5) && (sy2 < -5 || sy2 > SCREEN_HEIGHT + 5)) continue;
    
    // Draw path segment
    display.drawLine(sx1, sy1, sx2, sy2, SSD1306_WHITE);
    
    // Draw thicker line for recent path (last 20 points)
    if (i >= path_points_count - 20) {
      // Draw additional lines for thickness effect
      if (sx1 != sx2) {
        display.drawLine(sx1, sy1 + 1, sx2, sy2 + 1, SSD1306_WHITE);
      }
      if (sy1 != sy2) {
        display.drawLine(sx1 + 1, sy1, sx2 + 1, sy2, SSD1306_WHITE);
      }
    }
    
    // Draw directional arrows at intervals
    if (i % ARROW_SPACING == 0 && viewport.zoom_level >= 2.0) {
      int arrow_x = (sx1 + sx2) / 2;
      int arrow_y = (sy1 + sy2) / 2;
      drawDirectionalArrow(arrow_x, arrow_y, pathPoints[idx1].heading, 6);
    }
  }
}

void drawStartPosition() {
  int start_index = (path_points_count == MAX_PATH_POINTS) ? pathIndex : 0;
  if (path_points_count == 0) return;
  
  float scale = min((SCREEN_WIDTH - 20) / ((SCREEN_WIDTH - 20) / viewport.zoom_level), 
                    (SCREEN_HEIGHT - 20) / ((SCREEN_HEIGHT - 20) / viewport.zoom_level));
  
  int start_sx = (int)((pathPoints[start_index].x - viewport.centerX) * scale * viewport.zoom_level) + SCREEN_WIDTH/2;
  int start_sy = (int)((pathPoints[start_index].y - viewport.centerY) * scale * viewport.zoom_level) + SCREEN_HEIGHT/2;
  
  // Draw start marker (hollow square)
  display.drawRect(start_sx - 3, start_sy - 3, 6, 6, SSD1306_WHITE);
  display.drawRect(start_sx - 2, start_sy - 2, 4, 4, SSD1306_WHITE);
}

void drawCurrentPositionWithEffects() {
  // Calculate current position on screen
  float scale = min((SCREEN_WIDTH - 20) / ((SCREEN_WIDTH - 20) / viewport.zoom_level), 
                    (SCREEN_HEIGHT - 20) / ((SCREEN_HEIGHT - 20) / viewport.zoom_level));
  
  int current_sx = (int)((currentX - viewport.centerX) * scale * viewport.zoom_level) + SCREEN_WIDTH/2;
  int current_sy = (int)((currentY - viewport.centerY) * scale * viewport.zoom_level) + SCREEN_HEIGHT/2;
  
  // Draw trail effect
  for (int i = 0; i < TRAIL_LENGTH - 1; i++) {
    int trail_idx1 = (trail_index + i) % TRAIL_LENGTH;
    int trail_idx2 = (trail_index + i + 1) % TRAIL_LENGTH;
    
    int tx1 = (int)((trail_points_x[trail_idx1] - viewport.centerX) * scale * viewport.zoom_level) + SCREEN_WIDTH/2;
    int ty1 = (int)((trail_points_y[trail_idx1] - viewport.centerY) * scale * viewport.zoom_level) + SCREEN_HEIGHT/2;
    int tx2 = (int)((trail_points_x[trail_idx2] - viewport.centerX) * scale * viewport.zoom_level) + SCREEN_WIDTH/2;
    int ty2 = (int)((trail_points_y[trail_idx2] - viewport.centerY) * scale * viewport.zoom_level) + SCREEN_HEIGHT/2;
    
    // Draw fading trail (simulate by drawing fewer pixels for older trail)
    if (i < TRAIL_LENGTH / 2) {
      display.drawLine(tx1, ty1, tx2, ty2, SSD1306_WHITE);
    } else if (i % 2 == 0) {  // Dotted line for older trail
      display.drawPixel(tx1, ty1, SSD1306_WHITE);
    }
  }
  
  // Draw current position marker with heading indicator
  display.fillCircle(current_sx, current_sy, 4, SSD1306_WHITE);
  display.drawCircle(current_sx, current_sy, 6, SSD1306_WHITE);
  
  // Draw heading arrow
  drawDirectionalArrow(current_sx, current_sy, yaw, 8);
  
  // Draw speed indicator (pulsing effect based on cadence)
  if (current_cadence_hz > 0) {
    int pulse_size = 8 + (int)(3 * sin(millis() * 0.01));
    display.drawCircle(current_sx, current_sy, pulse_size, SSD1306_WHITE);
  }
}

void drawInformationOverlay() {
  // Top-left: Compass rose
  drawCompassRose(15, 15);
  
  // Top-right: Zoom level indicator
  display.setTextSize(1);
  display.setCursor(SCREEN_WIDTH - 25, 2);
  display.print(zoom_labels[current_zoom_level]);
  
  // Direction indicator next to zoom
  display.setCursor(SCREEN_WIDTH - 30, 12);
  if (yaw >= 337.5 || yaw < 22.5) display.print("N");
  else if (yaw >= 22.5 && yaw < 67.5) display.print("NE");
  else if (yaw >= 67.5 && yaw < 112.5) display.print("E");
  else if (yaw >= 112.5 && yaw < 157.5) display.print("SE");
  else if (yaw >= 157.5 && yaw < 202.5) display.print("S");
  else if (yaw >= 202.5 && yaw < 247.5) display.print("SW");
  else if (yaw >= 247.5 && yaw < 292.5) display.print("W");
  else display.print("NW");
  
  // Navigation mode indicator (top center)
  display.setCursor(50, 2);
  if (nav_mode == FORWARD_MODE) display.print("FWD");
  else if (nav_mode == REVERSE_MODE) display.print("REV");
  else display.print("END");
  
  // Bottom: Enhanced statistics
  char stats[40];
  if (total_distance >= 1000) {
    snprintf(stats, sizeof(stats), "%.1fkm %ds Z:%.1fm", 
             total_distance/1000, (int)step_count, currentZ);
  } else {
    snprintf(stats, sizeof(stats), "%.0fm %ds Z:%.1fm", 
             total_distance, (int)step_count, currentZ);
  }
  
  display.fillRect(0, SCREEN_HEIGHT - 10, SCREEN_WIDTH, 10, SSD1306_BLACK);
  display.setCursor(2, SCREEN_HEIGHT - 8);
  display.print(stats);
  
  // Mode indicator
  const char* mode_text = "";
  switch(viewport.zoom_mode) {
    case 0: mode_text = "AUTO"; break;
    case 1: mode_text = "TRACK"; break;
    case 2: mode_text = "FULL"; break;
  }
  display.setCursor(SCREEN_WIDTH - 20, SCREEN_HEIGHT - 8);
  display.print(mode_text);
}

void drawAdvancedMap() {
  display.clearDisplay();
  
  if (path_points_count < 2) {
    display.setCursor(0, 0);
    display.println("Walk to begin path...");
    
    // Show basic compass even when no path
    drawCompassRose(SCREEN_WIDTH/2, SCREEN_HEIGHT/2);
    
    // Show current heading
    display.setTextSize(1);
    display.setCursor(SCREEN_WIDTH/2 - 20, SCREEN_HEIGHT/2 + 20);
    display.print("Heading: ");
    display.print((int)yaw);
    display.print("°");
    
    display.display();
    return;
  }
  
  // Draw path with advanced effects
  drawPathWithEffects();
  
  // Draw start position marker
  drawStartPosition();
  
  // Draw current position with trail and effects
  drawCurrentPositionWithEffects();
  
  // Draw information overlay
  drawInformationOverlay();
  
  // Show waypoint notification if nearby
  displayWaypointNotification();
  
  display.display();
}

void displayInfoScreen() {
  display.clearDisplay();
  display.setCursor(0, 0);

  display.setTextSize(2);
  display.print("Steps:");
  display.setCursor(70, 0);
  display.println((int)step_count);

  display.drawLine(0, 18, SCREEN_WIDTH, 18, SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 22);
  display.print("R:"); display.print(roll, 0);
  display.setCursor(64, 22);
  display.print("P:"); display.print(pitch, 0);

  display.setCursor(0, 32);
  display.print("Head: "); display.print(yaw, 0); display.print("d");
  
  display.setCursor(0, 42);
  display.print("Cadence: "); display.print(current_cadence_hz * 60, 0); display.print(" spm");

  long time_left = (INFO_SCREEN_DURATION - (millis() - startup_time)) / 1000;
  display.setCursor(0, 56);
  display.print("Map in: "); display.print(time_left > 0 ? time_left : 0); display.print("s");

  display.display();
}

// #################################################################################################
// # 6. PATH REVERSIBILITY SYSTEM
// #################################################################################################

void handleButtonPress() {
  static bool last_button_state = HIGH;
  bool current_button_state = digitalRead(REVERSE_BUTTON_PIN);
  unsigned long current_time = millis();
  
  // Detect button press (transition from HIGH to LOW)
  if (last_button_state == HIGH && current_button_state == LOW) {
    button_press_start = current_time;
    button_currently_pressed = true;
    Serial.println("Button pressed");
  }
  
  // Detect button release (transition from LOW to HIGH)
  if (last_button_state == LOW && current_button_state == HIGH) {
    unsigned long press_duration = current_time - button_press_start;
    button_currently_pressed = false;
    
    // Handle short press (click counting for double-click)
    if (press_duration >= BUTTON_DEBOUNCE_MS) {
      click_count++;
      
      if (click_count == 1) {
        first_click_time = current_time;
        waiting_for_second_click = true;
        Serial.println("First click detected");
      } else if (click_count == 2) {
        // Double click detected - check if within timeout
        if (current_time - first_click_time <= DOUBLE_CLICK_TIMEOUT) {
          // Double click confirmed - toggle dashboard
          if (dashboard.current_mode == DASHBOARD_OFF) {
            dashboard.current_mode = DASHBOARD_TRANSITION;
            dashboard.last_transition_time = current_time;
            dashboard.transition_progress = 0.0;
            dashboard.exiting = false;
            Serial.println("✓ Double click - Dashboard activated");
          } else {
            dashboard.current_mode = DASHBOARD_TRANSITION;
            dashboard.last_transition_time = current_time;
            dashboard.transition_progress = 0.0;
            dashboard.exiting = true;
            Serial.println("✓ Double click - Dashboard exiting");
          }
        }
        click_count = 0;
        waiting_for_second_click = false;
        last_button_state = current_button_state;
        return;
      }
    }
  }
  
  // Handle single click timeout
  if (waiting_for_second_click && click_count == 1) {
    unsigned long time_since_first_click = current_time - first_click_time;
    
    if (time_since_first_click > DOUBLE_CLICK_TIMEOUT) {
      // Single click confirmed - handle navigation mode switching
      Serial.println("Single click confirmed - switching navigation mode");
      
      switch (nav_mode) {
        case FORWARD_MODE:
          if (path_points_count >= 3) { // Fixed: Need at least 3 points for breadcrumb navigation
            nav_mode = REVERSE_MODE;
            initializeBreadcrumbNavigation();
            Serial.println("✓ Single click - REVERSE MODE ACTIVATED");
          } else {
            Serial.print("✗ Single click - Not enough path points for reverse navigation (have ");
            Serial.print(path_points_count);
            Serial.println(", need at least 3)");
          }
          break;
          
        case REVERSE_MODE:
          nav_mode = FORWARD_MODE;
          breadcrumb_nav.algorithm_active = false;
          Serial.println("✓ Single click - FORWARD MODE RESUMED");
          break;
          
        case RETURN_COMPLETE:
          nav_mode = FORWARD_MODE;
          // Reset path for new journey
          path_points_count = 0;
          pathIndex = 0;
          currentX = start_position_x;
          currentY = start_position_y;
          currentZ = start_position_z;
          total_distance = 0;
          addBreadcrumb();
          Serial.println("✓ Single click - NEW JOURNEY STARTED");
          break;
      }
      
      click_count = 0;
      waiting_for_second_click = false;
    }
  }

  last_button_state = current_button_state;
}

// Initialize breadcrumb navigation system
void initializeBreadcrumbNavigation() {
  Serial.print("🧭 Initializing visual breadcrumb navigation - Path points: ");
  Serial.println(path_points_count);
  
  if (path_points_count < 3) { // Reduced from 5 to 3 for easier testing
    Serial.println("❌ Not enough path points for breadcrumb navigation (need at least 3)");
    nav_mode = FORWARD_MODE;
    return;
  }
  
  breadcrumb_nav.algorithm_active = true;
  breadcrumb_nav.current_target_index = path_points_count - 2; // Start from second-to-last point
  breadcrumb_nav.distance_threshold = 1.5; // More precise navigation
  breadcrumb_nav.direction_smoothing = 0.3; // Smooth direction changes
  breadcrumb_nav.last_target_distance = 999.0;
  breadcrumb_nav.target_bearing = 0.0;
  breadcrumb_nav.breadcrumbs_completed = 0;
  
  calculateCurrentBreadcrumbTarget();
  
  Serial.print("✅ Visual breadcrumb navigation ACTIVE!");
  Serial.print(" Starting from point ");
  Serial.print(breadcrumb_nav.current_target_index + 1);
  Serial.print(" at coordinates (");
  Serial.print(breadcrumb_nav.target_x, 1);
  Serial.print(", ");
  Serial.print(breadcrumb_nav.target_y, 1);
  Serial.print(") - Distance: ");
  Serial.print(sqrt(pow(currentX - breadcrumb_nav.target_x, 2) + pow(currentY - breadcrumb_nav.target_y, 2)), 1);
  Serial.println("m");
}

// Update breadcrumb navigation during reverse mode
void updateBreadcrumbNavigation() {
  if (!breadcrumb_nav.algorithm_active || nav_mode != REVERSE_MODE) return;
  
  float distance_to_target = sqrt(
    pow(currentX - breadcrumb_nav.target_x, 2) + 
    pow(currentY - breadcrumb_nav.target_y, 2)
  );
  
  // Check if we've reached the current breadcrumb
  if (distance_to_target <= breadcrumb_nav.distance_threshold) {
    Serial.print("Reached breadcrumb ");
    Serial.print(breadcrumb_nav.current_target_index);
    Serial.print(", moving to next");
    
    breadcrumb_nav.current_target_index--;
    
    if (breadcrumb_nav.current_target_index < 0) {
      // Reached the start - navigation complete
      nav_mode = RETURN_COMPLETE;
      breadcrumb_nav.algorithm_active = false;
      Serial.println(" - Return journey complete!");
      return;
    }
    
    calculateCurrentBreadcrumbTarget();
    Serial.print(" (");
    Serial.print(breadcrumb_nav.target_x, 1);
    Serial.print(", ");
    Serial.print(breadcrumb_nav.target_y, 1);
    Serial.println(")");
  }
  
  // Update direction guidance
  calculateBreadcrumbDirection();
  breadcrumb_nav.last_target_distance = distance_to_target;
}

// Calculate target position for current breadcrumb
void calculateCurrentBreadcrumbTarget() {
  if (breadcrumb_nav.current_target_index >= 0 && 
      breadcrumb_nav.current_target_index < path_points_count) {
    
    breadcrumb_nav.target_x = pathPoints[breadcrumb_nav.current_target_index].x;
    breadcrumb_nav.target_y = pathPoints[breadcrumb_nav.current_target_index].y;
    breadcrumb_nav.target_z = pathPoints[breadcrumb_nav.current_target_index].z;
    
    // Update distance immediately
    breadcrumb_nav.last_target_distance = sqrt(
      pow(currentX - breadcrumb_nav.target_x, 2) + 
      pow(currentY - breadcrumb_nav.target_y, 2)
    );
    
    Serial.print("🎯 Target set to breadcrumb ");
    Serial.print(breadcrumb_nav.current_target_index);
    Serial.print(" at (");
    Serial.print(breadcrumb_nav.target_x, 1);
    Serial.print(", ");
    Serial.print(breadcrumb_nav.target_y, 1);
    Serial.print(") - Distance: ");
    Serial.print(breadcrumb_nav.last_target_distance, 1);
    Serial.println("m");
  } else {
    Serial.print("❌ Invalid breadcrumb index: ");
    Serial.print(breadcrumb_nav.current_target_index);
    Serial.print(" (total points: ");
    Serial.print(path_points_count);
    Serial.println(")");
    breadcrumb_nav.algorithm_active = false;
  }
}

// Calculate direction to next breadcrumb
void calculateBreadcrumbDirection() {
  float dx = breadcrumb_nav.target_x - currentX;
  float dy = breadcrumb_nav.target_y - currentY;
  
  float raw_bearing = atan2(dy, dx) * 180.0 / PI;
  if (raw_bearing < 0) raw_bearing += 360.0;
  
  // Smooth the direction change
  float bearing_diff = raw_bearing - breadcrumb_nav.target_bearing;
  if (bearing_diff > 180) bearing_diff -= 360;
  if (bearing_diff < -180) bearing_diff += 360;
  
  breadcrumb_nav.target_bearing += bearing_diff * breadcrumb_nav.direction_smoothing;
  if (breadcrumb_nav.target_bearing < 0) breadcrumb_nav.target_bearing += 360;
  if (breadcrumb_nav.target_bearing >= 360) breadcrumb_nav.target_bearing -= 360;
}

  // Enhanced reverse display for OLED - Shows visual map with breadcrumb trail (NO BORDER for more space)
void drawEnhancedReverseDisplay() {
  display.clearDisplay();
  
  if (!breadcrumb_nav.algorithm_active || path_points_count < 2) {
    display.setTextSize(2);
    display.setCursor(10, 15);
    display.print("REVERSE");
    display.setTextSize(1);
    display.setCursor(15, 35);
    display.print("No breadcrumb");
    display.setCursor(25, 45);
    display.print("trail data");
    display.display();
    return;
  }
  
  // Add REVERSE MODE label at the top
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("REVERSE MODE");
  
  // Draw the FROZEN path with current position - no new trail during reverse
  // Adjust map area to start from line 10 to leave space for title
  drawMiniMapWithTrail();
  
  // Bottom status area - Distance Display "Dist: X.Xm" as specified
  float distance_to_target = sqrt(
    pow(currentX - breadcrumb_nav.target_x, 2) + 
    pow(currentY - breadcrumb_nav.target_y, 2)
  );
  
  // Distance info (Lines 55-64)
  display.setTextSize(1);
  display.setCursor(0, 55);
  display.print("Dist: ");
  display.print(distance_to_target, 1);
  display.print("m");
  
  // Progress indicator
  display.setCursor(60, 55);
  display.print("P");
  display.print(breadcrumb_nav.current_target_index + 1);
  display.print("/");
  display.print(path_points_count);
  
  // Large Direction Arrow in bottom right - Points toward START position like a compass
  float bearing_to_start = atan2(
    pathPoints[0].y - currentY,   // Direction to START point, not current target
    pathPoints[0].x - currentX
  ) * 180.0 / PI;
  
  if (bearing_to_start < 0) bearing_to_start += 360.0;
  
  // Ensure compass arrow is visible in bottom right corner
  display.setTextSize(2);  // Large size for direction arrow as specified
  display.setCursor(105, 48);  // Moved slightly left and up for better visibility
  if (bearing_to_start >= 315 || bearing_to_start < 45) display.print("^");      // North - up arrow
  else if (bearing_to_start >= 45 && bearing_to_start < 135) display.print(">");  // East - right arrow  
  else if (bearing_to_start >= 135 && bearing_to_start < 225) display.print("v"); // South - down arrow
  else display.print("<");  // West - left arrow
  
  // Draw a small 'S' label next to arrow to indicate it points to Start
  display.setTextSize(1);
  display.setCursor(115, 58);
  display.print("S");
  
  display.display();
}

// Draw a mini map showing the breadcrumb trail and current position (ADJUSTED for title space)
void drawMiniMapWithTrail() {
  if (path_points_count < 2) return;
  
  // Calculate bounds of the path for auto-scaling
  float minX = pathPoints[0].x, maxX = pathPoints[0].x;
  float minY = pathPoints[0].y, maxY = pathPoints[0].y;
  
  for (int i = 0; i < path_points_count; i++) {
    if (pathPoints[i].x < minX) minX = pathPoints[i].x;
    if (pathPoints[i].x > maxX) maxX = pathPoints[i].x;
    if (pathPoints[i].y < minY) minY = pathPoints[i].y;
    if (pathPoints[i].y > maxY) maxY = pathPoints[i].y;
  }
  
  // Add current position to bounds
  if (currentX < minX) minX = currentX;
  if (currentX > maxX) maxX = currentX;
  if (currentY < minY) minY = currentY;
  if (currentY > maxY) maxY = currentY;
  
  // Calculate scale to fit in the available area (y: 10-50, x: 0-128) - Leave space for title
  float pathWidth = maxX - minX;
  float pathHeight = maxY - minY;
  
  if (pathWidth < 1.0) pathWidth = 1.0;  // Prevent division by zero
  if (pathHeight < 1.0) pathHeight = 1.0;
  
  float scaleX = 124.0 / pathWidth;   // Use almost full width (leave 2 pixels margin)
  float scaleY = 40.0 / pathHeight;   // Use 40 pixels height (10-50, save space for title and status)
  float scale = min(scaleX, scaleY);  // Use the smaller scale to fit both dimensions
  
  // Center the map in the available area (shifted down for title)
  float centerX = (minX + maxX) / 2;
  float centerY = (minY + maxY) / 2;
  
  // Draw the breadcrumb trail
  for (int i = 0; i < path_points_count - 1; i++) {
    int x1 = (int)(64 + (pathPoints[i].x - centerX) * scale);
    int y1 = (int)(30 - (pathPoints[i].y - centerY) * scale);      // Adjusted for title space
    int x2 = (int)(64 + (pathPoints[i + 1].x - centerX) * scale);
    int y2 = (int)(30 - (pathPoints[i + 1].y - centerY) * scale);  // Adjusted for title space
    
    // Ensure points are within display bounds (adjusted for title)
    if (x1 >= 0 && x1 < 128 && y1 >= 10 && y1 < 50 &&
        x2 >= 0 && x2 < 128 && y2 >= 10 && y2 < 50) {
      
      // Draw thicker line for completed path (already traversed back)
      if (i > breadcrumb_nav.current_target_index) {
        // Thick solid line for completed reverse path
        display.drawLine(x1, y1, x2, y2, SSD1306_WHITE);
        // Add thickness
        if (x1 != x2) display.drawLine(x1, y1 + 1, x2, y2 + 1, SSD1306_WHITE);
        if (y1 != y2) display.drawLine(x1 + 1, y1, x2 + 1, y2, SSD1306_WHITE);
      } else {
        // Dotted line for remaining reverse path
        int steps = max(abs(x2 - x1), abs(y2 - y1));
        for (int step = 0; step < steps; step += 3) {
          int x = x1 + (x2 - x1) * step / steps;
          int y = y1 + (y2 - y1) * step / steps;
          display.drawPixel(x, y, SSD1306_WHITE);
        }
      }
    }
  }
  
  // Draw start position (hollow square - 4x4 pixels as specified) - Where we want to return
  int startX = (int)(64 + (pathPoints[0].x - centerX) * scale);
  int startY = (int)(30 - (pathPoints[0].y - centerY) * scale);     // Adjusted for title space
  if (startX >= 2 && startX < 126 && startY >= 12 && startY < 48) {
    display.drawRect(startX - 2, startY - 2, 4, 4, SSD1306_WHITE);
    // Make it more visible - double border
    display.drawRect(startX - 1, startY - 1, 2, 2, SSD1306_WHITE);
  }
  
  // Draw current target (filled square - 3x3 pixels as specified) - Next waypoint
  if (breadcrumb_nav.current_target_index >= 0) {
    int targetX = (int)(64 + (breadcrumb_nav.target_x - centerX) * scale);
    int targetY = (int)(30 - (breadcrumb_nav.target_y - centerY) * scale); // Adjusted for title space
    if (targetX >= 1 && targetX < 127 && targetY >= 11 && targetY < 49) {
      display.fillRect(targetX - 1, targetY - 1, 3, 3, SSD1306_WHITE);
    }
  }
  
  // Draw current position (filled circle - 2-pixel radius as specified) - Where you are now
  int currentPosX = (int)(64 + (currentX - centerX) * scale);
  int currentPosY = (int)(30 - (currentY - centerY) * scale);       // Adjusted for title space
  if (currentPosX >= 2 && currentPosX < 126 && currentPosY >= 12 && currentPosY < 48) {
    display.fillCircle(currentPosX, currentPosY, 2, SSD1306_WHITE);
    
    // Add heading indicator - line extending from current position showing direction you're facing
    float headingRad = yaw * PI / 180.0;
    int headingX = currentPosX + (int)(5 * sin(headingRad)); // Longer line for better visibility
    int headingY = currentPosY - (int)(5 * cos(headingRad));
    // Ensure heading line stays within bounds
    if (headingX >= 0 && headingX < 128 && headingY >= 10 && headingY < 50) {
      display.drawLine(currentPosX, currentPosY, headingX, headingY, SSD1306_WHITE);
    }
  }
}

// =============================================================================
// SOS EMERGENCY SYSTEM
// =============================================================================

// Trigger SOS emergency signal
void triggerSOS() {
  sos_active = true;
  sos_start_time = millis();
  sos_triggered = true;
  
  Serial.println("🆘 SOS SIGNAL TRIGGERED! Emergency alert sent via BLE");
  Serial.println("🚁 Rescue services have been notified - help is on the way!");
  
  // Send SOS via BLE (simulate for now)
  // In real implementation, this would send GPS coordinates via BLE/cellular
}

// Check if SOS is currently active
bool isSOSActive() {
  if (!sos_active) return false;
  
  if (millis() - sos_start_time > SOS_DURATION) {
    sos_active = false;
    Serial.println("✅ SOS display timeout - returning to normal navigation");
    return false;
  }
  
  return true;
}

// Draw SOS emergency display
void updateSOSDisplay() {
  display.clearDisplay();
  
  // Flashing SOS display
  bool flash = (millis() / 500) % 2; // Flash every 500ms
  
  if (flash) {
    // Large SOS text
    display.setTextSize(3);
    display.setCursor(25, 15);
    display.print("SOS");
    
    // Emergency message
    display.setTextSize(1);
    display.setCursor(15, 45);
    display.print("EMERGENCY SIGNAL");
    display.setCursor(35, 55);
    display.print("SENT VIA BLE");
  } else {
    // Alternate display
    display.setTextSize(2);
    display.setCursor(35, 20);
    display.print("HELP");
    display.setTextSize(1);
    display.setCursor(20, 45);
    display.print("Rescue on the way");
  }
  
  display.display();
}

// =============================================================================
// VERTICAL TRACKING SYSTEM (BMP180 Altitude Monitoring)
// =============================================================================

// Initialize vertical tracking system
void initializeVerticalTracking() {
  if (!bmp_found) {
    Serial.println("❌ BMP180 not found - Vertical tracking unavailable");
    return;
  }
  
  vertical_tracker.monitoring_active = true;
  vertical_tracker.zero_point_set = true; // Automatically set to true
  vertical_tracker.buffer_index = 0;
  vertical_tracker.noise_threshold = 0.1; // 10cm noise threshold
  vertical_tracker.floor_height = 3.0;    // 3m standard floor height
  
  // Initialize altitude buffer and automatically set zero point
  float initial_altitude = bmp.readAltitude();
  for (int i = 0; i < 10; i++) {
    vertical_tracker.altitude_buffer[i] = initial_altitude;
  }
  
  // Automatically set current altitude as zero reference
  vertical_tracker.zero_altitude = initial_altitude;
  vertical_tracker.filtered_altitude = initial_altitude;
  vertical_tracker.current_altitude = initial_altitude;
  vertical_tracker.depth_change = 0.0;
  vertical_tracker.max_depth = 0.0;
  vertical_tracker.max_height = 0.0;
  vertical_tracker.estimated_floor = 0;
  vertical_tracker.is_underground = false;
  vertical_tracker.time_underground = 0;
  
  Serial.println("✅ Vertical tracking system initialized");
  Serial.print("  Automatic altitude zero point set at: ");
  Serial.print(vertical_tracker.zero_altitude, 2);
  Serial.println("m");
}

// Set current altitude as zero reference point (entrance/ground level)
void setAltitudeZeroPoint() {
  if (!bmp_found) return;
  
  vertical_tracker.zero_altitude = bmp.readAltitude();
  vertical_tracker.filtered_altitude = vertical_tracker.zero_altitude;
  vertical_tracker.current_altitude = vertical_tracker.zero_altitude;
  vertical_tracker.zero_point_set = true;
  vertical_tracker.depth_change = 0.0;
  vertical_tracker.max_depth = 0.0;
  vertical_tracker.max_height = 0.0;
  vertical_tracker.estimated_floor = 0;
  vertical_tracker.is_underground = false;
  vertical_tracker.time_underground = 0;
  
  Serial.print("📍 Altitude zero point set at: ");
  Serial.print(vertical_tracker.zero_altitude, 2);
  Serial.println("m");
}

// Noise filtering using rolling average with outlier rejection
float filterAltitudeNoise(float raw_altitude) {
  // Add new reading to circular buffer
  vertical_tracker.altitude_buffer[vertical_tracker.buffer_index] = raw_altitude;
  vertical_tracker.buffer_index = (vertical_tracker.buffer_index + 1) % 10;
  
  // Calculate rolling average
  float sum = 0;
  float min_val = vertical_tracker.altitude_buffer[0];
  float max_val = vertical_tracker.altitude_buffer[0];
  
  for (int i = 0; i < 10; i++) {
    sum += vertical_tracker.altitude_buffer[i];
    if (vertical_tracker.altitude_buffer[i] < min_val) min_val = vertical_tracker.altitude_buffer[i];
    if (vertical_tracker.altitude_buffer[i] > max_val) max_val = vertical_tracker.altitude_buffer[i];
  }
  
  // Remove outliers if range is too large (> 1m suggests noise)
  if ((max_val - min_val) > 1.0) {
    sum = sum - min_val - max_val; // Remove extreme values
    return sum / 8.0; // Average of remaining 8 values
  }
  
  return sum / 10.0; // Normal average
}

// Estimate floor level based on depth change
int estimateFloorLevel(float depth_change) {
  return (int)round(depth_change / vertical_tracker.floor_height);
}

// Update vertical tracking measurements
void updateVerticalTracking() {
  if (!bmp_found || !vertical_tracker.monitoring_active) return;
  
  unsigned long current_time = millis();
  if (current_time - vertical_tracker.last_update < 500) return; // Update every 500ms
  
  // Read raw altitude
  float raw_altitude = bmp.readAltitude();
  vertical_tracker.current_altitude = raw_altitude;
  
  // Apply noise filtering
  vertical_tracker.filtered_altitude = filterAltitudeNoise(raw_altitude);
  
  if (!vertical_tracker.zero_point_set) {
    vertical_tracker.last_update = current_time;
    return;
  }
  
  // Calculate depth/height change from zero point
  float prev_depth = vertical_tracker.depth_change;
  vertical_tracker.depth_change = vertical_tracker.zero_altitude - vertical_tracker.filtered_altitude;
  
  // Only update if change is above noise threshold
  if (abs(vertical_tracker.depth_change - prev_depth) > vertical_tracker.noise_threshold) {
    
    // Update max depth/height
    if (vertical_tracker.depth_change > vertical_tracker.max_depth) {
      vertical_tracker.max_depth = vertical_tracker.depth_change;
    }
    if (vertical_tracker.depth_change < 0 && abs(vertical_tracker.depth_change) > vertical_tracker.max_height) {
      vertical_tracker.max_height = abs(vertical_tracker.depth_change);
    }
    
    // Estimate floor level
    vertical_tracker.estimated_floor = estimateFloorLevel(vertical_tracker.depth_change);
    
    // Track underground time
    bool was_underground = vertical_tracker.is_underground;
    vertical_tracker.is_underground = (vertical_tracker.depth_change > 0.5); // 50cm threshold
    
    if (vertical_tracker.is_underground && !was_underground) {
      vertical_tracker.time_underground = current_time;
    }
    
    // Removed problematic debug output that could cause blocking
  }
  
  vertical_tracker.last_update = current_time;
}

// Draw vertical tracking display (after 4-quadrant display)
void drawVerticalTrackingDisplay() {
  display.clearDisplay();
  
  if (!bmp_found) {
    display.setTextSize(2);
    display.setCursor(15, 15);
    display.print("NO BMP180");
    display.setTextSize(1);
    display.setCursor(20, 40);
    display.print("Vertical tracking");
    display.setCursor(30, 50);
    display.print("unavailable");
    display.display();
    return;
  }
  
  // Main vertical tracking display (zero point is automatically set)
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("ALTITUDE MONITOR");
  
  // Current depth/height - larger text
  display.setTextSize(2);
  display.setCursor(0, 12);
  if (vertical_tracker.depth_change >= 0) {
    display.print("D:");
    display.print(vertical_tracker.depth_change, 1);
    display.print("m");
  } else {
    display.print("H:");
    display.print(abs(vertical_tracker.depth_change), 1);
    display.print("m");
  }
  
  // Floor level estimate
  display.setTextSize(1);
  display.setCursor(0, 30);
  display.print("Floor: ");
  if (vertical_tracker.estimated_floor > 0) {
    display.print("B");
    display.print(vertical_tracker.estimated_floor);
  } else if (vertical_tracker.estimated_floor < 0) {
    display.print("F");
    display.print(abs(vertical_tracker.estimated_floor));
  } else {
    display.print("Ground");
  }
  
  // Status indicators - move to separate line
  display.setCursor(0, 40);
  if (vertical_tracker.is_underground) {
    display.print("UNDERGROUND");
  } else {
    display.print("Above Ground");
  }
  
  // Max depth/height reached - compact format
  display.setCursor(0, 50);
  display.print("Max D:");
  display.print(vertical_tracker.max_depth, 1);
  display.print(" H:");
  display.print(vertical_tracker.max_height, 1);
  
  // Visual depth indicator (bar graph) - adjusted position
  int bar_height = constrain(abs(vertical_tracker.depth_change) * 2, 0, 25);
  int bar_x = 105;
  int bar_y = 64 - bar_height;
  
  if (vertical_tracker.depth_change >= 0) {
    // Below ground - fill downward
    display.fillRect(bar_x, 32, 15, bar_height, SSD1306_WHITE);
    display.drawRect(bar_x - 1, 31, 17, 32, SSD1306_WHITE);
  } else {
    // Above ground - fill upward  
    display.fillRect(bar_x, bar_y, 15, bar_height, SSD1306_WHITE);
    display.drawRect(bar_x - 1, 0, 17, 32, SSD1306_WHITE);
  }
  
  display.display();
}

// =============================================================================

// =============================================================================
// DASHBOARD DISPLAY FUNCTIONS  
// =============================================================================
// Note: Old reverse navigation display functions removed - now using enhanced breadcrumb system

// =============================================================================
// SENSOR AND DASHBOARD SUPPORT FUNCTIONS  
// =============================================================================

void calculateAirDensity() {
  // Calculate air density using ideal gas law approximation
  // ρ = P / (R * T) where P=pressure, R=specific gas constant, T=temperature
  // Using approximation: ρ ≈ P / (287 * (T + 273.15))
  float temp_kelvin = sensors.temperature + 273.15;
  if (temp_kelvin > 0) {
    sensors.air_density = (sensors.pressure * 100) / (287.0 * temp_kelvin); // kg/m³
  } else {
    sensors.air_density = 1.225; // Standard air density at sea level
  }
}

void drawLargeDirectionalArrow(int center_x, int center_y, float relative_bearing, int size) {
  float arrow_rad = relative_bearing * PI / 180.0;
  
  // Arrow shaft
  int shaft_x = center_x + (int)(size * sin(arrow_rad));
  int shaft_y = center_y - (int)(size * cos(arrow_rad));
  display.drawLine(center_x, center_y, shaft_x, shaft_y, SSD1306_WHITE);
  
  // Arrow head
  float head_angle1 = arrow_rad + 2.5;
  float head_angle2 = arrow_rad - 2.5;
  int head_size = size / 2;
  
  int head_x1 = shaft_x - (int)(head_size * sin(head_angle1));
  int head_y1 = shaft_y + (int)(head_size * cos(head_angle1));
  int head_x2 = shaft_x - (int)(head_size * sin(head_angle2));
  int head_y2 = shaft_y + (int)(head_size * cos(head_angle2));
  
  display.drawLine(shaft_x, shaft_y, head_x1, head_y1, SSD1306_WHITE);
  display.drawLine(shaft_x, shaft_y, head_x2, head_y2, SSD1306_WHITE);
}

void drawReturnCompleteScreen() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(15, 10);
  display.print("RETURN");
  display.setCursor(5, 30);
  display.print("COMPLETE!");
  
  display.setTextSize(1);
  display.setCursor(10, 50);
  display.print("Press button for new");
  display.setCursor(30, 58);
  display.print("journey");
  
  // Draw celebration animation
  int star_time = (millis() / 200) % 4;
  for (int i = 0; i < 4; i++) {
    if (i == star_time) {
      display.fillCircle(20 + i * 22, 5, 2, SSD1306_WHITE);
    } else {
      display.drawCircle(20 + i * 22, 5, 1, SSD1306_WHITE);
    }
  }
}

// #################################################################################################
// # 7. 3D DASHBOARD SYSTEM - Advanced Sensor Visualization
// #################################################################################################

void updateSensorReadings() {
  unsigned long current_time = millis();
  
  // Update sensor readings at specified refresh rate
  if (current_time - sensors.last_update >= DASHBOARD_REFRESH_RATE) {
    // Get fresh sensor data
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    // Temperature from MPU6050
    sensors.temperature = temp.temperature;
    
    // BMP180 readings
    if (bmp_found) {
      sensors.pressure = bmp.readPressure() / 100.0; // Convert to hPa
      sensors.altitude_abs = bmp.readAltitude();
      
      // Calculate air density
      calculateAirDensity();
    }
    
    // Acceleration (m/s²)
    sensors.accel_x = a.acceleration.x;
    sensors.accel_y = a.acceleration.y;
    sensors.accel_z = a.acceleration.z;
    
    // Gyroscope (rad/s converted to deg/s)
    sensors.gyro_x = (g.gyro.x - gyro_x_offset) * 57.2958;
    sensors.gyro_y = (g.gyro.y - gyro_y_offset) * 57.2958;
    sensors.gyro_z = (g.gyro.z - gyro_z_offset) * 57.2958;
    
    sensors.last_update = current_time;
  }
}

void update3DDashboard() {
  if (dashboard.current_mode == DASHBOARD_OFF) return;
  
  unsigned long current_time = millis();
  unsigned long elapsed = current_time - dashboard.last_transition_time;
  
  switch (dashboard.current_mode) {
    case DASHBOARD_TRANSITION:
      // Calculate transition progress (0.0 to 1.0)
      dashboard.transition_progress = (float)elapsed / DASHBOARD_TRANSITION_TIME;
      
      if (!dashboard.exiting) {
        // Entering dashboard
        if (dashboard.transition_progress >= 1.0) {
          dashboard.current_mode = DASHBOARD_ON;
          dashboard.mode_start_time = current_time;
          dashboard.transition_progress = 1.0;
        }
      } else {
        // Exiting dashboard
        if (dashboard.transition_progress >= 1.0) {
          dashboard.current_mode = DASHBOARD_OFF;
          dashboard.exiting = false;
        }
      }
      break;
      
    case DASHBOARD_ON:
      // Auto-transition to vertical tracking after 5 seconds
      if (current_time - dashboard.mode_start_time >= 5000) {
        dashboard.current_mode = DASHBOARD_VERTICAL;
        dashboard.mode_start_time = current_time;
        Serial.println("🏔️ Dashboard transitioning to vertical tracking display");
      }
      break;
      
    case DASHBOARD_VERTICAL:
      // Auto-exit after 5 seconds of vertical tracking
      if (current_time - dashboard.mode_start_time >= DASHBOARD_VERTICAL_DURATION) {
        dashboard.current_mode = DASHBOARD_TRANSITION;
        dashboard.last_transition_time = current_time;
        dashboard.transition_progress = 0.0;
        dashboard.exiting = true;
        Serial.println("✅ Vertical tracking display timeout - returning to normal navigation");
      }
      break;
  }
}

void draw3DDashboard() {
  if (dashboard.current_mode == DASHBOARD_OFF) return;
  
  if (dashboard.current_mode == DASHBOARD_TRANSITION) {
    display.clearDisplay();
    draw3DTransition();
    display.display();
  } else if (dashboard.current_mode == DASHBOARD_ON) {
    display.clearDisplay();
    drawDashboardScreen1();
    display.display();
  } else if (dashboard.current_mode == DASHBOARD_VERTICAL) {
    drawVerticalTrackingDisplay(); // This function already handles clearDisplay and display()
  }
}

void draw3DTransition() {
  // Clean 4-quadrant division with simple lines
  int center_x = SCREEN_WIDTH / 2;
  int center_y = SCREEN_HEIGHT / 2;
  
  // Calculate line extension based on progress
  float progress = dashboard.transition_progress;
  if (dashboard.exiting) progress = 1.0 - progress;
  
  // Smooth easing function
  float smooth_progress = progress * progress * (3.0 - 2.0 * progress);
  
  // Draw simple dividing lines
  int line_length_h = (int)(center_x * smooth_progress);
  int line_length_v = (int)(center_y * smooth_progress);
  
  // Horizontal line (extends from center outward)
  display.drawLine(center_x - line_length_h, center_y, 
                   center_x + line_length_h, center_y, SSD1306_WHITE);
  
  // Vertical line (extends from center outward)  
  display.drawLine(center_x, center_y - line_length_v, 
                   center_x, center_y + line_length_v, SSD1306_WHITE);
  
  // Draw quadrant labels that fade in
  if (smooth_progress > 0.5) {
    display.setTextSize(1);
    
    // Top-left quadrant
    display.setCursor(8, 8);
    display.print("TEMP");
    
    // Top-right quadrant  
    display.setCursor(center_x + 8, 8);
    display.print("PRESS");
    
    // Bottom-left quadrant
    display.setCursor(8, center_y + 8);
    display.print("ALT");
    
    // Bottom-right quadrant
    display.setCursor(center_x + 8, center_y + 8);
    display.print("STEPS");
  }
}

void drawDashboardScreen1() {
  // Clean 4-quadrant layout with dividing lines
  int center_x = SCREEN_WIDTH / 2;
  int center_y = SCREEN_HEIGHT / 2;
  
  // Draw the dividing lines
  display.drawLine(0, center_y, SCREEN_WIDTH, center_y, SSD1306_WHITE);  // Horizontal line
  display.drawLine(center_x, 0, center_x, SCREEN_HEIGHT, SSD1306_WHITE); // Vertical line
  
  // Quadrant 1: Temperature (top-left)
  display.setTextSize(1);
  display.setCursor(2, 2);
  display.print("TEMP");
  display.setTextSize(2);
  display.setCursor(8, 15);
  display.print(sensors.temperature, 1);
  display.setTextSize(1);
  display.setCursor(8, 30);
  display.print("C");
  
  // Quadrant 2: Pressure (top-right)
  display.setTextSize(1);
  display.setCursor(center_x + 2, 2);
  display.print("PRESS");
  display.setTextSize(2);
  display.setCursor(center_x + 8, 15);
  display.print((int)sensors.pressure);
  display.setTextSize(1);
  display.setCursor(center_x + 8, 30);
  display.print("hPa");
  
  // Quadrant 3: Altitude (bottom-left)
  display.setTextSize(1);
  display.setCursor(2, center_y + 2);
  display.print("ALT");
  display.setTextSize(2);
  display.setCursor(8, center_y + 15);
  display.print(sensors.altitude_abs, 1);
  display.setTextSize(1);
  display.setCursor(8, center_y + 30);
  display.print("m");
  
  // Quadrant 4: Steps (bottom-right)
  display.setTextSize(1);
  display.setCursor(center_x + 2, center_y + 2);
  display.print("STEPS");
  display.setTextSize(2);
  display.setCursor(center_x + 8, center_y + 15);
  display.print((int)step_count);
}

// #################################################################################################
// # 8. WIFI & WEB SERVER SYSTEM
// #################################################################################################

void setupWiFi() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Starting WiFi AP");
  display.println("SSID: myosa");
  display.display();
  
  // Create Access Point
  WiFi.mode(WIFI_AP);
  bool success = WiFi.softAP(ap_ssid, ap_password);
  
  if (success) {
    IPAddress IP = WiFi.softAPIP();
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("WiFi AP Started!");
    display.println("SSID: myosa");
    display.println("IP: ");
    display.println(IP);
    display.println("Connect to 'myosa'");
    display.println("Open: " + IP.toString());
    display.display();
    Serial.println("WiFi Access Point started");
    Serial.print("AP IP address: ");
    Serial.println(IP);
    Serial.println("Connect to 'myosa' and go to " + IP.toString());
    delay(3000);
  } else {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("WiFi AP Failed!");
    display.println("Restarting ESP32...");
    display.display();
    Serial.println("WiFi Access Point failed to start");
    delay(2000);
    ESP.restart();
  }
}

void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/navigator.js", handleJavaScript);
  server.on("/data", handleData);
  server.on("/path", handlePath);
  server.on("/sensors", handleSensors);
  server.on("/status", handleStatus);
  server.on("/waypoints", HTTP_GET, handleWaypoints);
  server.on("/waypoints", HTTP_POST, handleAddWaypoint);
  server.on("/waypoints", HTTP_DELETE, handleDeleteWaypoint);
  server.on("/toggle-reverse", HTTP_POST, handleToggleReverse);
  server.on("/trigger-sos", HTTP_POST, handleTriggerSOS);
  
  server.begin();
  Serial.println("Web server started with waypoint support and reverse toggle");
}

void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>ESP32 9-DOF Navigator</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <style>
        :root {
            /* Light mode variables */
            --bg-color: #f0f0f0;
            --text-color: #333;
            --panel-bg: white;
            --header-bg: #2c3e50;
            --header-text: white;
            --border-color: #bdc3c7;
            --shadow: 0 2px 10px rgba(0,0,0,0.1);
            --btn-bg: #3498db;
            --btn-hover: #2980b9;
            --btn-text: white;
        }
        
        /* Dark mode variables */
        [data-theme="dark"] {
            --bg-color: #1a1a1a;
            --text-color: #e0e0e0;
            --panel-bg: #2d2d2d;
            --header-bg: #1e1e1e;
            --header-text: #e0e0e0;
            --border-color: #404040;
            --shadow: 0 2px 10px rgba(0,0,0,0.3);
            --btn-bg: #4a90e2;
            --btn-hover: #357abd;
            --btn-text: white;
        }
        
        /* Apply dark mode to additional elements */
        [data-theme="dark"] .sensor-item {
            background: #3a3a3a;
            border: 1px solid var(--border-color);
        }
        
        [data-theme="dark"] canvas {
            filter: brightness(0.9);
        }
        
        [data-theme="dark"] input, [data-theme="dark"] select {
            background: #3a3a3a;
            color: var(--text-color);
            border: 1px solid var(--border-color);
        }
        
        body { 
            font-family: Arial, sans-serif; 
            margin: 0; 
            padding: 10px; 
            background: var(--bg-color); 
            color: var(--text-color);
            transition: background-color 0.3s ease, color 0.3s ease;
        }
        .container { max-width: 1200px; margin: 0 auto; }
        .header { 
            text-align: center; 
            background: var(--header-bg); 
            color: var(--header-text); 
            padding: 15px; 
            border-radius: 10px; 
            margin-bottom: 15px; 
            position: relative;
            transition: background-color 0.3s ease;
        }
        .header h1 { margin: 0; font-size: 1.5rem; }
        .header p { margin: 5px 0 0 0; font-size: 0.9rem; }
        
        /* Mobile-first responsive design */
        .dashboard { display: grid; grid-template-columns: 1fr; gap: 15px; margin-bottom: 15px; }
        .panel { 
            background: var(--panel-bg); 
            padding: 15px; 
            border-radius: 10px; 
            box-shadow: var(--shadow); 
            transition: background-color 0.3s ease, box-shadow 0.3s ease;
        }
        .map-panel { position: relative; }
        .map-fullscreen { position: fixed; top: 0; left: 0; width: 100vw; height: 100vh; z-index: 1000; background: white; }
        
        .sensor-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
        .sensor-item { background: #ecf0f1; padding: 10px; border-radius: 8px; text-align: center; }
        .sensor-value { font-size: 1.2rem; font-weight: bold; color: #2c3e50; }
        .sensor-label { font-size: 0.8rem; color: #7f8c8d; margin-top: 3px; }
        
        #mapCanvas { border: 2px solid #34495e; border-radius: 10px; background: #2c3e50; width: 100%; height: auto; max-width: 100%; }
        
        .controls { background: white; padding: 15px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); margin-top: 10px; }
        .btn-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; margin-bottom: 10px; }
        .btn-grid-wide { display: grid; grid-template-columns: 1fr; gap: 8px; }
        .btn { 
            background: var(--btn-bg); 
            color: var(--btn-text); 
            padding: 12px 8px; 
            border: none; 
            border-radius: 5px; 
            cursor: pointer; 
            font-size: 0.9rem; 
            text-align: center; 
            transition: all 0.3s ease;
        }
        .btn:hover { background: var(--btn-hover); }
        .btn.active { background: #e74c3c; }
        .btn:disabled { background: #bdc3c7; cursor: not-allowed; }
        
        .status-bar { background: #34495e; color: white; padding: 8px; border-radius: 5px; margin-top: 10px; font-size: 0.85rem; }
        .nav-info { background: #f39c12; color: white; padding: 12px; border-radius: 10px; margin-bottom: 15px; }
        .waypoint-panel { background: #ecf0f1; padding: 12px; border-radius: 10px; margin-top: 10px; }
        .waypoint-form { display: grid; grid-template-columns: 1fr; gap: 8px; }
        .waypoint-form input, .waypoint-form select { padding: 8px; border: 1px solid #bdc3c7; border-radius: 4px; font-size: 0.9rem; }
        .waypoint-list { max-height: 150px; overflow-y: auto; }
        .waypoint-item { display: flex; justify-content: space-between; align-items: center; padding: 6px; background: white; margin: 3px 0; border-radius: 5px; font-size: 0.85rem; }
        .waypoint-icon { font-size: 14px; margin-right: 8px; }
        .fullscreen-btn { position: absolute; top: 5px; right: 5px; background: rgba(52, 73, 94, 0.8); color: white; border: none; padding: 5px 8px; border-radius: 5px; cursor: pointer; font-size: 0.8rem; }
        .close-fullscreen { position: fixed; top: 15px; right: 15px; background: #e74c3c; color: white; border: none; padding: 8px 12px; border-radius: 5px; cursor: pointer; z-index: 1001; }
        
        /* Tablet and larger screens */
        @media (min-width: 768px) {
            body { padding: 20px; }
            .dashboard { grid-template-columns: 1fr 1fr; gap: 20px; }
            .header h1 { font-size: 2rem; }
            .header p { font-size: 1rem; }
            .sensor-grid { grid-template-columns: 1fr 1fr 1fr 1fr; }
            .btn-grid { grid-template-columns: 1fr 1fr 1fr; }
            .waypoint-form { grid-template-columns: 1fr auto auto; align-items: center; }
            .waypoint-list { max-height: 200px; }
            #mapCanvas { width: 400px; height: 300px; }
        }
        
        /* Desktop screens */
        @media (min-width: 1024px) {
            .sensor-grid { grid-template-columns: repeat(4, 1fr); }
            .btn-grid { grid-template-columns: repeat(5, 1fr); }
        }
        
        /* Touch-friendly improvements */
        @media (pointer: coarse) {
            .btn { padding: 14px 10px; }
            .waypoint-item button { padding: 8px 12px; }
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🧭 ESP32 9-DOF Navigator</h1>
            <p>Real-time GPS-free Navigation System</p>
            <button id="darkModeToggle" class="btn" onclick="toggleDarkMode()" style="position: absolute; top: 15px; right: 15px; padding: 8px 12px; background: var(--btn-bg);">🌙</button>
        </div>
        
        <div id="navInfo" class="nav-info" style="display: none;">
            <h3>Navigation Status</h3>
            <div id="navStatus">Ready</div>
        </div>
        
        <div class="dashboard">
            <div class="panel">
                <h3>📊 Live Sensor Data</h3>
                <div class="sensor-grid">
                    <div class="sensor-item">
                        <div class="sensor-value" id="temperature">--</div>
                        <div class="sensor-label">Temperature (°C)</div>
                    </div>
                    <div class="sensor-item">
                        <div class="sensor-value" id="pressure">--</div>
                        <div class="sensor-label">Pressure (hPa)</div>
                    </div>
                    <div class="sensor-item">
                        <div class="sensor-value" id="altitude">--</div>
                        <div class="sensor-label">Altitude (m)</div>
                    </div>
                    <div class="sensor-item">
                        <div class="sensor-value" id="steps">--</div>
                        <div class="sensor-label">Steps</div>
                    </div>
                    <div class="sensor-item">
                        <div class="sensor-value" id="heading">--</div>
                        <div class="sensor-label">Heading (°)</div>
                    </div>
                    <div class="sensor-item">
                        <div class="sensor-value" id="distance">--</div>
                        <div class="sensor-label">Distance (m)</div>
                    </div>
                </div>
            </div>
            
            <!-- Vertical Tracking Panel with Graph -->
            <div class="panel">
                <h3>🏔️ Vertical Tracking</h3>
                <div id="verticalStatus" class="sensor-item" style="margin-bottom: 15px;">
                    <div class="sensor-value" id="currentDepth">--</div>
                    <div class="sensor-label">Current Depth/Height (m)</div>
                </div>
                <canvas id="verticalChart" width="350" height="150" style="border: 1px solid #bdc3c7; border-radius: 5px; width: 100%; max-width: 350px; background: #f8f9fa;"></canvas>
                <div class="sensor-grid" style="margin-top: 10px;">
                    <div class="sensor-item">
                        <div class="sensor-value" id="currentFloor">--</div>
                        <div class="sensor-label">Floor Level</div>
                    </div>
                    <div class="sensor-item">
                        <div class="sensor-value" id="maxDepth">--</div>
                        <div class="sensor-label">Max Depth (m)</div>
                    </div>
                    <div class="sensor-item">
                        <div class="sensor-value" id="maxHeight">--</div>
                        <div class="sensor-label">Max Height (m)</div>
                    </div>
                    <div class="sensor-item">
                        <div class="sensor-value" id="undergroundStatus">--</div>
                        <div class="sensor-label">Status</div>
                    </div>
                </div>
            </div>
            
            <div class="panel map-panel">
                <h3>🗺️ Interactive Map</h3>
                <button class="fullscreen-btn" onclick="toggleFullscreen()">⛶ Fullscreen</button>
                <canvas id="mapCanvas" width="300" height="250"></canvas>
                <div class="controls">
                    <div class="btn-grid">
                        <button class="btn" onclick="zoomIn()">🔍+ Zoom</button>
                        <button class="btn" onclick="zoomOut()">🔍- Zoom</button>
                        <button class="btn" onclick="centerMap()">📍 Center</button>
                        <button class="btn" onclick="showFullPath()">🗺️ Full</button>
                    </div>
                    <div class="btn-grid-wide">
                        <button class="btn" id="reverseBtn" onclick="toggleReverse()">↩️ Reverse Mode</button>
                        <button class="btn" id="sosBtn" onclick="triggerSOS()" style="background: #e74c3c; font-weight: bold;">🆘 EMERGENCY SOS</button>
                    </div>
                </div>
                <div class="status-bar">
                    <span>Zoom: <span id="zoomLevel">1.0x</span></span>
                    <span style="margin-left: 20px;">Mode: <span id="currentMode">Forward</span></span>
                    <span style="margin-left: 20px;">Waypoints: <span id="waypointCount">0</span></span>
                </div>
                
                <div class="waypoint-panel">
                    <h4>📍 Waypoint Management</h4>
                    <div class="waypoint-form">
                        <input type="text" id="waypointName" placeholder="Waypoint name" maxlength="15">
                        <select id="waypointType">
                            <option value="0">📍 Generic</option>
                            <option value="1">🏕️ Camp</option>
                            <option value="2">💧 Water</option>
                            <option value="3">🍎 Food</option>
                            <option value="4">🏠 Shelter</option>
                            <option value="5">👁️ View</option>
                            <option value="6">⚠️ Danger</option>
                            <option value="7">📷 Photo</option>
                            <option value="8">🤝 Meeting</option>
                            <option value="9">🚪 Exit</option>
                        </select>
                        <button class="btn" onclick="addWaypoint()">➕ Add</button>
                    </div>
                    <div id="waypointList" class="waypoint-list">
                        <!-- Waypoints will be populated here -->
                    </div>
                </div>
            </div>
        </div>
        
        <div class="panel">
            <h3>📈 Motion Analytics</h3>
            <div class="sensor-grid">
                <div class="sensor-item">
                    <div class="sensor-value" id="accelMag">--</div>
                    <div class="sensor-label">Acceleration (m/s²)</div>
                </div>
                <div class="sensor-item">
                    <div class="sensor-value" id="gyroMag">--</div>
                    <div class="sensor-label">Gyroscope (°/s)</div>
                </div>
                <div class="sensor-item">
                    <div class="sensor-value" id="airDensity">--</div>
                    <div class="sensor-label">Air Density (kg/m³)</div>
                </div>
                <div class="sensor-item">
                    <div class="sensor-value" id="cadence">--</div>
                    <div class="sensor-label">Cadence (steps/min)</div>
                </div>
            </div>
        </div>
    </div>
    
    <script src="/navigator.js"></script>
</body>
</html>
)rawliteral";
  
  server.send(200, "text/html", html);
}

void handleData() {
  DynamicJsonDocument doc(1024);
  
  // Current position and orientation
  doc["position"]["x"] = currentX;
  doc["position"]["y"] = currentY;
  doc["position"]["z"] = currentZ;
  doc["orientation"]["pitch"] = pitch;
  doc["orientation"]["roll"] = roll;
  doc["orientation"]["yaw"] = yaw;
  
  // Navigation data
  doc["navigation"]["mode"] = (nav_mode == FORWARD_MODE) ? "forward" : 
                             (nav_mode == REVERSE_MODE) ? "reverse" : "complete";
  doc["navigation"]["steps"] = (int)step_count;
  doc["navigation"]["distance"] = total_distance;
  doc["navigation"]["cadence"] = current_cadence_hz * 60; // steps per minute
  
  // Reverse navigation info
  if (nav_mode == REVERSE_MODE) {
    doc["reverse"]["distance_to_target"] = breadcrumb_nav.distance_to_target;
    doc["reverse"]["target_x"] = breadcrumb_nav.target_x;
    doc["reverse"]["target_y"] = breadcrumb_nav.target_y;
    doc["reverse"]["current_index"] = breadcrumb_nav.current_target_index;
    doc["reverse"]["total_breadcrumbs"] = path_points_count;
    doc["reverse"]["algorithm_active"] = breadcrumb_nav.algorithm_active;
  }
  
  // SOS Emergency Status
  doc["emergency"]["sos_active"] = sos_active;
  doc["emergency"]["sos_triggered"] = sos_triggered;
  if (sos_active) {
    doc["emergency"]["sos_remaining"] = (SOS_DURATION - (millis() - sos_start_time)) / 1000;
  }
  
  // Vertical Tracking Status
  if (bmp_found) {
    doc["vertical"]["available"] = true;
    doc["vertical"]["zero_set"] = vertical_tracker.zero_point_set;
    if (vertical_tracker.zero_point_set) {
      doc["vertical"]["depth_change"] = vertical_tracker.depth_change; // Fixed: use depth_change instead of depth
      doc["vertical"]["estimated_floor"] = vertical_tracker.estimated_floor; // Fixed: use estimated_floor instead of floor
      doc["vertical"]["is_underground"] = vertical_tracker.is_underground; // Fixed: use is_underground instead of underground
      doc["vertical"]["max_depth"] = vertical_tracker.max_depth;
      doc["vertical"]["max_height"] = vertical_tracker.max_height;
    }
  } else {
    doc["vertical"]["available"] = false;
  }
  
  // Timestamp
  doc["timestamp"] = millis();
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handlePath() {
  DynamicJsonDocument doc(4096);
  JsonArray pathArray = doc.createNestedArray("path");
  
  // Send path points
  int start_idx = (path_points_count == MAX_PATH_POINTS) ? pathIndex : 0;
  for (int i = 0; i < path_points_count; i++) {
    int idx = (start_idx + i) % MAX_PATH_POINTS;
    JsonObject point = pathArray.createNestedObject();
    point["x"] = pathPoints[idx].x;
    point["y"] = pathPoints[idx].y;
    point["z"] = pathPoints[idx].z;
    point["heading"] = pathPoints[idx].heading;
    point["speed"] = pathPoints[idx].speed_level;
    point["timestamp"] = pathPoints[idx].timestamp;
  }
  
  // Add start position
  doc["start"]["x"] = start_position_x;
  doc["start"]["y"] = start_position_y;
  doc["start"]["z"] = start_position_z;
  
  // Add current viewport
  doc["viewport"]["centerX"] = viewport.centerX;
  doc["viewport"]["centerY"] = viewport.centerY;
  doc["viewport"]["zoom"] = viewport.zoom_level;
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleSensors() {
  DynamicJsonDocument doc(1024);
  
  // Environmental sensors
  doc["environmental"]["temperature"] = sensors.temperature;
  doc["environmental"]["pressure"] = sensors.pressure;
  doc["environmental"]["altitude"] = sensors.altitude_abs;
  doc["environmental"]["air_density"] = sensors.air_density;
  
  // Vertical tracking system
  doc["vertical"]["monitoring_active"] = vertical_tracker.monitoring_active;
  doc["vertical"]["zero_point_set"] = vertical_tracker.zero_point_set;
  if (vertical_tracker.zero_point_set) {
    doc["vertical"]["zero_altitude"] = vertical_tracker.zero_altitude;
    doc["vertical"]["current_altitude"] = vertical_tracker.current_altitude;
    doc["vertical"]["filtered_altitude"] = vertical_tracker.filtered_altitude;
    doc["vertical"]["depth_change"] = vertical_tracker.depth_change;
    doc["vertical"]["max_depth"] = vertical_tracker.max_depth;
    doc["vertical"]["max_height"] = vertical_tracker.max_height;
    doc["vertical"]["estimated_floor"] = vertical_tracker.estimated_floor;
    doc["vertical"]["is_underground"] = vertical_tracker.is_underground;
    if (vertical_tracker.is_underground && vertical_tracker.time_underground > 0) {
      doc["vertical"]["time_underground_ms"] = millis() - vertical_tracker.time_underground;
    }
  }
  
  // Motion sensors
  doc["motion"]["accel_x"] = sensors.accel_x;
  doc["motion"]["accel_y"] = sensors.accel_y;
  doc["motion"]["accel_z"] = sensors.accel_z;
  doc["motion"]["gyro_x"] = sensors.gyro_x;
  doc["motion"]["gyro_y"] = sensors.gyro_y;
  doc["motion"]["gyro_z"] = sensors.gyro_z;
  
  // Calculated values
  float accel_mag = sqrt(sensors.accel_x*sensors.accel_x + 
                        sensors.accel_y*sensors.accel_y + 
                        sensors.accel_z*sensors.accel_z);
  float gyro_mag = sqrt(sensors.gyro_x*sensors.gyro_x + 
                       sensors.gyro_y*sensors.gyro_y + 
                       sensors.gyro_z*sensors.gyro_z);
  
  doc["calculated"]["accel_magnitude"] = accel_mag;
  doc["calculated"]["gyro_magnitude"] = gyro_mag;
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleStatus() {
  DynamicJsonDocument doc(512);
  
  doc["system"]["uptime"] = millis();
  doc["system"]["free_heap"] = ESP.getFreeHeap();
  doc["system"]["wifi_rssi"] = WiFi.RSSI();
  doc["system"]["wifi_status"] = WiFi.status();
  
  doc["navigation"]["path_points"] = path_points_count;
  doc["navigation"]["max_points"] = MAX_PATH_POINTS;
  doc["navigation"]["current_mode"] = (nav_mode == FORWARD_MODE) ? "forward" : 
                                     (nav_mode == REVERSE_MODE) ? "reverse" : "complete";
  
  doc["dashboard"]["active"] = (dashboard.current_mode != DASHBOARD_OFF);
  doc["dashboard"]["mode"] = dashboard.current_mode;
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleWaypoints() {
  DynamicJsonDocument doc(2048);
  JsonArray waypointArray = doc.createNestedArray("waypoints");
  
  for (int i = 0; i < waypoint_count; i++) {
    if (!waypoints[i].active) continue;
    
    JsonObject wp = waypointArray.createNestedObject();
    wp["id"] = i;
    wp["name"] = waypoints[i].name;
    wp["x"] = waypoints[i].x;
    wp["y"] = waypoints[i].y;
    wp["z"] = waypoints[i].z;
    wp["type"] = waypoints[i].type;
    wp["icon"] = waypoint_icons[waypoints[i].type];
    wp["timestamp"] = waypoints[i].timestamp;
    
    // Calculate distance from current position
    float dx = waypoints[i].x - currentX;
    float dy = waypoints[i].y - currentY;
    wp["distance"] = sqrt(dx * dx + dy * dy);
  }
  
  doc["count"] = waypoint_count;
  doc["max_waypoints"] = MAX_WAYPOINTS;
  doc["nearest_waypoint"] = nearest_waypoint_index;
  doc["distance_to_nearest"] = distance_to_nearest_waypoint;
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleAddWaypoint() {
  if (waypoint_count >= MAX_WAYPOINTS) {
    server.send(400, "application/json", "{\"error\":\"Maximum waypoints reached\"}");
    return;
  }
  
  String body = server.arg("plain");
  DynamicJsonDocument doc(512);
  DeserializationError error = deserializeJson(doc, body);
  
  if (error) {
    server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
    return;
  }
  
  String name = doc["name"] | "Waypoint";
  int type = doc["type"] | 0;
  bool use_current_pos = doc["use_current_position"] | true;
  
  float x = use_current_pos ? currentX : (doc["x"] | 0.0);
  float y = use_current_pos ? currentY : (doc["y"] | 0.0);
  float z = use_current_pos ? currentZ : (doc["z"] | 0.0);
  
  int waypoint_id = addWaypoint(x, y, z, name.c_str(), (WaypointType)type);
  
  if (waypoint_id >= 0) {
    DynamicJsonDocument response_doc(256);
    response_doc["success"] = true;
    response_doc["waypoint_id"] = waypoint_id;
    response_doc["message"] = "Waypoint added successfully";
    
    String response;
    serializeJson(response_doc, response);
    server.send(200, "application/json", response);
  } else {
    server.send(500, "application/json", "{\"error\":\"Failed to add waypoint\"}");
  }
}

void handleDeleteWaypoint() {
  String waypoint_id = server.arg("id");
  int id = waypoint_id.toInt();
  
  if (id >= 0 && id < waypoint_count && waypoints[id].active) {
    waypoints[id].active = false;
    
    DynamicJsonDocument response_doc(256);
    response_doc["success"] = true;
    response_doc["message"] = "Waypoint deleted successfully";
    
    String response;
    serializeJson(response_doc, response);
    server.send(200, "application/json", response);
  } else {
    server.send(404, "application/json", "{\"error\":\"Waypoint not found\"}");
  }
}

void handleToggleReverse() {
  Serial.println("Web interface: Toggle reverse requested");
  
  DynamicJsonDocument response_doc(256);
  
  // Toggle the navigation mode
  switch (nav_mode) {
    case FORWARD_MODE:
      if (path_points_count > 2) {
        nav_mode = REVERSE_MODE;
        initializeBreadcrumbNavigation();
        response_doc["success"] = true;
        response_doc["new_mode"] = "reverse";
        response_doc["message"] = "Reverse mode activated";
        Serial.println("Web interface: Reverse mode activated");
      } else {
        response_doc["success"] = false;
        response_doc["new_mode"] = "forward";
        response_doc["message"] = "Not enough path points for reverse navigation";
        Serial.println("Web interface: Not enough path points");
      }
      break;
      
    case REVERSE_MODE:
      nav_mode = FORWARD_MODE;
      breadcrumb_nav.algorithm_active = false;
      response_doc["success"] = true;
      response_doc["new_mode"] = "forward";
      response_doc["message"] = "Forward mode activated";
      Serial.println("Web interface: Forward mode activated");
      break;
      
    case RETURN_COMPLETE:
      nav_mode = FORWARD_MODE;
      response_doc["success"] = true;
      response_doc["new_mode"] = "forward";
      response_doc["message"] = "New journey ready";
      Serial.println("Web interface: New journey started");
      break;
  }
  
  String response;
  serializeJson(response_doc, response);
  server.send(200, "application/json", response);
}

void handleTriggerSOS() {
  Serial.println("🆘 SOS TRIGGERED via Web Interface!");
  
  // Trigger the SOS system
  triggerSOS();
  
  // Send response to web interface
  StaticJsonDocument<200> response_doc;
  response_doc["success"] = true;
  response_doc["message"] = "SOS signal sent successfully";
  response_doc["timestamp"] = millis();
  response_doc["status"] = "Emergency alert transmitted via BLE";
  
  String response;
  serializeJson(response_doc, response);
  server.send(200, "application/json", response);
}

void handleJavaScript() {
  String js = R"rawliteral(
class NavigatorApp {
    constructor() {
        this.canvas = document.getElementById('mapCanvas');
        this.ctx = this.canvas.getContext('2d');
        this.pathData = [];
        this.waypointData = [];
        this.currentPosition = { x: 0, y: 0, z: 0, heading: 0 };
        this.startPosition = { x: 0, y: 0, z: 0 };
        this.viewport = { centerX: 0, centerY: 0, zoom: 1.0 };
        this.navigationMode = 'forward';
        this.reverseNavData = {};
        this.isFullscreen = false;
        
        // Vertical tracking data
        this.verticalData = [];
        this.maxVerticalDataPoints = 50;
        this.verticalChart = document.getElementById('verticalChart');
        this.verticalCtx = this.verticalChart ? this.verticalChart.getContext('2d') : null;
        
        this.mapSettings = {
            scale: 10,
            backgroundColor: '#2c3e50',
            pathColor: '#3498db',
            currentPosColor: '#e74c3c',
            startPosColor: '#2ecc71',
            gridColor: '#34495e',
            targetColor: '#f39c12',
            waypointColor: '#9b59b6'
        };
        
        this.waypointIcons = ['📍', '🏕️', '💧', '🍎', '🏠', '👁️', '⚠️', '📷', '🤝', '🚪'];
        
        this.setupCanvas();
        this.startDataUpdates();
        this.setupEventListeners();
    }
    
    setupCanvas() {
        // Make canvas responsive
        const container = this.canvas.parentElement;
        const containerWidth = container.clientWidth - 40; // Account for padding
        const isMobile = window.innerWidth < 768;
        
        if (isMobile) {
            this.canvas.width = Math.min(containerWidth, 320);
            this.canvas.height = Math.min(240, this.canvas.width * 0.75);
        } else {
            this.canvas.width = Math.min(containerWidth, 400);
            this.canvas.height = Math.min(300, this.canvas.width * 0.75);
        }
        
        // Update canvas CSS for responsiveness
        this.canvas.style.width = '100%';
        this.canvas.style.height = 'auto';
        this.canvas.style.maxWidth = '100%';
    }
    
    setupEventListeners() {
        // Canvas mouse/touch events
        this.canvas.addEventListener('wheel', (e) => {
            e.preventDefault();
            const zoomFactor = e.deltaY > 0 ? 0.9 : 1.1;
            this.viewport.zoom = Math.max(0.1, Math.min(10, this.viewport.zoom * zoomFactor));
            this.updateZoomDisplay();
            this.redrawMap();
        });
        
        this.canvas.addEventListener('dblclick', (e) => {
            const rect = this.canvas.getBoundingClientRect();
            const x = e.clientX - rect.left;
            const y = e.clientY - rect.top;
            const worldPos = this.screenToWorld(x, y);
            this.showWaypointDialog(worldPos.x, worldPos.y);
        });
        
        // Window resize handler for responsive design
        window.addEventListener('resize', () => {
            setTimeout(() => {
                this.setupCanvas();
                this.redrawMap();
            }, 100);
        });
        
        // Touch support for mobile
        let touchStartPos = null;
        this.canvas.addEventListener('touchstart', (e) => {
            e.preventDefault();
            if (e.touches.length === 1) {
                const touch = e.touches[0];
                const rect = this.canvas.getBoundingClientRect();
                touchStartPos = {
                    x: touch.clientX - rect.left,
                    y: touch.clientY - rect.top
                };
            }
        });
        
        this.canvas.addEventListener('touchend', (e) => {
            e.preventDefault();
            if (e.changedTouches.length === 1 && touchStartPos) {
                const touch = e.changedTouches[0];
                const rect = this.canvas.getBoundingClientRect();
                const touchEndPos = {
                    x: touch.clientX - rect.left,
                    y: touch.clientY - rect.top
                };
                
                // Check if it was a tap (not a drag)
                const distance = Math.sqrt(
                    Math.pow(touchEndPos.x - touchStartPos.x, 2) + 
                    Math.pow(touchEndPos.y - touchStartPos.y, 2)
                );
                
                if (distance < 10) {
                    const worldPos = this.screenToWorld(touchEndPos.x, touchEndPos.y);
                    this.showWaypointDialog(worldPos.x, worldPos.y);
                }
                
                touchStartPos = null;
            }
        });
    }
    
    screenToWorld(screenX, screenY) {
        const worldX = this.viewport.centerX + (screenX - this.canvas.width / 2) / (this.mapSettings.scale * this.viewport.zoom);
        const worldY = this.viewport.centerY - (screenY - this.canvas.height / 2) / (this.mapSettings.scale * this.viewport.zoom);
        return { x: worldX, y: worldY };
    }
    
    showWaypointDialog(x, y) {
        const name = prompt('Enter waypoint name:', 'Waypoint_' + (this.waypointData.length + 1));
        if (name) {
            this.addWaypointAt(x, y, name, 0); // Generic type
        }
    }
    
    startDataUpdates() {
        this.sensorUpdateTimer = setInterval(() => this.updateSensorData(), 500);
        this.pathUpdateTimer = setInterval(() => this.updatePathData(), 1000);
        this.navUpdateTimer = setInterval(() => this.updateNavigationData(), 250);
        this.waypointUpdateTimer = setInterval(() => this.updateWaypointData(), 2000);
    }
    
    async updateSensorData() {
        try {
            const response = await fetch('/sensors');
            const data = await response.json();
            
            document.getElementById('temperature').textContent = data.environmental.temperature.toFixed(1);
            document.getElementById('pressure').textContent = data.environmental.pressure.toFixed(0);
            document.getElementById('altitude').textContent = data.environmental.altitude.toFixed(1);
            document.getElementById('accelMag').textContent = data.calculated.accel_magnitude.toFixed(2);
            document.getElementById('gyroMag').textContent = data.calculated.gyro_magnitude.toFixed(1);
            document.getElementById('airDensity').textContent = data.environmental.air_density.toFixed(3);
            
            // Update vertical tracking data if available
            if (data.vertical) {
                this.updateVerticalTracking(data.vertical);
            }
        } catch (error) {
            console.error('Sensor update error:', error);
        }
    }
    
    updateVerticalTracking(verticalData) {
        // Update vertical tracking display elements
        const currentDepthEl = document.getElementById('currentDepth');
        const currentFloorEl = document.getElementById('currentFloor');
        const maxDepthEl = document.getElementById('maxDepth');
        const maxHeightEl = document.getElementById('maxHeight');
        const undergroundStatusEl = document.getElementById('undergroundStatus');
        
        if (verticalData.zero_set) {
            // Store data point for graph
            const timestamp = Date.now();
            this.verticalData.push({
                time: timestamp,
                depth: verticalData.depth_change || 0,
                floor: verticalData.estimated_floor || 0
            });
            
            // Limit data points for performance
            if (this.verticalData.length > this.maxVerticalDataPoints) {
                this.verticalData.shift();
            }
            
            // Update display values
            const depth = verticalData.depth_change || 0;
            if (depth >= 0) {
                currentDepthEl.textContent = `D: ${depth.toFixed(1)}m`;
                currentDepthEl.style.color = '#e74c3c'; // Red for depth
            } else {
                currentDepthEl.textContent = `H: ${Math.abs(depth).toFixed(1)}m`;
                currentDepthEl.style.color = '#3498db'; // Blue for height
            }
            
            // Floor display
            const floor = verticalData.estimated_floor || 0;
            if (floor > 0) {
                currentFloorEl.textContent = `B${floor}`;
            } else if (floor < 0) {
                currentFloorEl.textContent = `F${Math.abs(floor)}`;
            } else {
                currentFloorEl.textContent = 'Ground';
            }
            
            maxDepthEl.textContent = (verticalData.max_depth || 0).toFixed(1);
            maxHeightEl.textContent = (verticalData.max_height || 0).toFixed(1);
            undergroundStatusEl.textContent = verticalData.is_underground ? 'Underground' : 'Surface';
            undergroundStatusEl.style.color = verticalData.is_underground ? '#e74c3c' : '#27ae60';
            
            // Update graph
            this.drawVerticalChart();
        } else {
            currentDepthEl.textContent = 'Not Set';
            currentFloorEl.textContent = '--';
            maxDepthEl.textContent = '--';
            maxHeightEl.textContent = '--';
            undergroundStatusEl.textContent = 'Inactive';
        }
    }
    
    drawVerticalChart() {
        if (!this.verticalCtx || this.verticalData.length < 2) return;
        
        const canvas = this.verticalChart;
        const ctx = this.verticalCtx;
        
        // Clear canvas
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        
        // Chart dimensions
        const padding = 30;
        const chartWidth = canvas.width - 2 * padding;
        const chartHeight = canvas.height - 2 * padding;
        
        // Find data range
        let minDepth = Math.min(...this.verticalData.map(d => d.depth));
        let maxDepth = Math.max(...this.verticalData.map(d => d.depth));
        
        // Add some padding to the range
        const range = Math.max(Math.abs(maxDepth - minDepth), 1);
        minDepth -= range * 0.1;
        maxDepth += range * 0.1;
        
        // Draw background
        ctx.fillStyle = '#f8f9fa';
        ctx.fillRect(0, 0, canvas.width, canvas.height);
        
        // Draw grid lines
        ctx.strokeStyle = '#e9ecef';
        ctx.lineWidth = 1;
        
        // Horizontal grid lines (depth levels)
        for (let i = 0; i <= 5; i++) {
            const y = padding + (i / 5) * chartHeight;
            ctx.beginPath();
            ctx.moveTo(padding, y);
            ctx.lineTo(padding + chartWidth, y);
            ctx.stroke();
        }
        
        // Vertical grid lines (time)
        for (let i = 0; i <= 5; i++) {
            const x = padding + (i / 5) * chartWidth;
            ctx.beginPath();
            ctx.moveTo(x, padding);
            ctx.lineTo(x, padding + chartHeight);
            ctx.stroke();
        }
        
        // Draw zero line (ground level)
        const zeroY = padding + chartHeight - ((-minDepth) / (maxDepth - minDepth)) * chartHeight;
        ctx.strokeStyle = '#2c3e50';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(padding, zeroY);
        ctx.lineTo(padding + chartWidth, zeroY);
        ctx.stroke();
        
        // Draw depth curve
        if (this.verticalData.length > 1) {
            ctx.strokeStyle = '#e74c3c';
            ctx.lineWidth = 3;
            ctx.beginPath();
            
            for (let i = 0; i < this.verticalData.length; i++) {
                const x = padding + (i / (this.verticalData.length - 1)) * chartWidth;
                const y = padding + chartHeight - ((this.verticalData[i].depth - minDepth) / (maxDepth - minDepth)) * chartHeight;
                
                if (i === 0) {
                    ctx.moveTo(x, y);
                } else {
                    ctx.lineTo(x, y);
                }
            }
            ctx.stroke();
            
            // Draw current position dot
            const lastPoint = this.verticalData[this.verticalData.length - 1];
            const currentX = padding + chartWidth;
            const currentY = padding + chartHeight - ((lastPoint.depth - minDepth) / (maxDepth - minDepth)) * chartHeight;
            
            ctx.fillStyle = '#e74c3c';
            ctx.beginPath();
            ctx.arc(currentX, currentY, 4, 0, 2 * Math.PI);
            ctx.fill();
        }
        
        // Draw labels
        ctx.fillStyle = '#2c3e50';
        ctx.font = '12px Arial';
        ctx.textAlign = 'center';
        
        // Y-axis labels (depth)
        ctx.textAlign = 'right';
        for (let i = 0; i <= 5; i++) {
            const depth = maxDepth - (i / 5) * (maxDepth - minDepth);
            const y = padding + (i / 5) * chartHeight + 4;
            ctx.fillText(depth.toFixed(1) + 'm', padding - 5, y);
        }
        
        // Title
        ctx.textAlign = 'center';
        ctx.font = 'bold 14px Arial';
        ctx.fillText('Altitude Over Time', canvas.width / 2, 20);
    }
    
    async updateWaypointData() {
        try {
            const response = await fetch('/waypoints');
            const data = await response.json();
            this.waypointData = data.waypoints;
            
            document.getElementById('waypointCount').textContent = data.count;
            this.updateWaypointList();
            this.redrawMap();
        } catch (error) {
            console.error('Waypoint update error:', error);
        }
    }
    
    updateWaypointList() {
        const listElement = document.getElementById('waypointList');
        listElement.innerHTML = '';
        
        this.waypointData.forEach((wp, index) => {
            const item = document.createElement('div');
            item.className = 'waypoint-item';
            item.innerHTML = `
                <div>
                    <span class="waypoint-icon">${wp.icon}</span>
                    <strong>${wp.name}</strong> - ${wp.distance.toFixed(1)}m
                </div>
                <button class="btn" onclick="app.deleteWaypoint(${wp.id})" style="background: #e74c3c; padding: 5px 10px;">🗑️</button>
            `;
            listElement.appendChild(item);
        });
    }
    
    async addWaypointAt(x, y, name, type) {
        try {
            const response = await fetch('/waypoints', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    name: name,
                    type: type,
                    use_current_position: false,
                    x: x,
                    y: y,
                    z: 0
                })
            });
            
            if (response.ok) {
                this.updateWaypointData();
            }
        } catch (error) {
            console.error('Add waypoint error:', error);
        }
    }
    
    async deleteWaypoint(id) {
        try {
            const response = await fetch(`/waypoints?id=${id}`, {
                method: 'DELETE'
            });
            
            if (response.ok) {
                this.updateWaypointData();
            }
        } catch (error) {
            console.error('Delete waypoint error:', error);
        }
    }
    
    async updatePathData() {
        try {
            const response = await fetch('/path');
            const data = await response.json();
            
            this.pathData = data.path;
            this.startPosition = data.start;
            
            if (data.viewport) {
                this.viewport.centerX = data.viewport.centerX;
                this.viewport.centerY = data.viewport.centerY;
            }
            
            this.redrawMap();
        } catch (error) {
            console.error('Path update error:', error);
        }
    }
    
    async updateNavigationData() {
        try {
            const response = await fetch('/data');
            const data = await response.json();
            
            this.currentPosition = data.position;
            this.currentPosition.heading = data.orientation.yaw;
            
            document.getElementById('steps').textContent = data.navigation.steps;
            document.getElementById('distance').textContent = data.navigation.distance.toFixed(0);
            document.getElementById('heading').textContent = data.orientation.yaw.toFixed(0);
            document.getElementById('cadence').textContent = data.navigation.cadence.toFixed(0);
            
            this.navigationMode = data.navigation.mode;
            document.getElementById('currentMode').textContent = 
                this.navigationMode.charAt(0).toUpperCase() + this.navigationMode.slice(1);
            
            if (data.reverse) {
                this.reverseNavData = data.reverse;
                this.updateNavigationStatus();
            }
            
            // Update vertical tracking if available
            if (data.vertical) {
                this.updateVerticalTracking(data.vertical);
            }
            
            const reverseBtn = document.getElementById('reverseBtn');
            if (this.navigationMode === 'reverse') {
                reverseBtn.classList.add('active');
                reverseBtn.textContent = '⏹️ Stop';
            } else {
                reverseBtn.classList.remove('active');
                reverseBtn.textContent = '↩️ Reverse Mode';
            }
            
            this.redrawMap();
        } catch (error) {
            console.error('Navigation update error:', error);
        }
    }
    
    updateNavigationStatus() {
        const navInfo = document.getElementById('navInfo');
        const navStatus = document.getElementById('navStatus');
        
        if (this.navigationMode === 'reverse') {
            navInfo.style.display = 'block';
            
            let statusText = '';
            if (this.reverseNavData.approaching_start) {
                statusText = `🏠 Returning Home - ${this.reverseNavData.distance_to_start.toFixed(1)}m to start`;
            } else {
                statusText = `🧭 Following Path - ${this.reverseNavData.distance_to_target.toFixed(1)}m to next waypoint`;
                
                if (this.reverseNavData.off_track) {
                    statusText += ' ⚠️ OFF TRACK';
                }
                
                if (Math.abs(this.reverseNavData.turn_angle) < 15) {
                    statusText += ' - Go Straight';
                } else if (this.reverseNavData.turn_angle > 0) {
                    statusText += ` - Turn Right ${Math.abs(this.reverseNavData.turn_angle).toFixed(0)}°`;
                } else {
                    statusText += ` - Turn Left ${Math.abs(this.reverseNavData.turn_angle).toFixed(0)}°`;
                }
            }
            
            navStatus.innerHTML = statusText;
        } else {
            navInfo.style.display = 'none';
        }
    }
    
    redrawMap() {
        this.ctx.fillStyle = this.mapSettings.backgroundColor;
        this.ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);
        
        this.drawGrid();
        this.drawPath();
        this.drawWaypoints();
        this.drawStartPosition();
        this.drawCurrentPosition();
        
        if (this.navigationMode === 'reverse') {
            this.drawReverseNavigation();
        }
        
        this.drawCompass();
        this.drawScale();
    }
    
    drawWaypoints() {
        this.waypointData.forEach(waypoint => {
            const screen = this.worldToScreen(waypoint.x, waypoint.y);
            
            // Draw waypoint circle
            this.ctx.fillStyle = this.mapSettings.waypointColor;
            this.ctx.beginPath();
            this.ctx.arc(screen.x, screen.y, 8, 0, 2 * Math.PI);
            this.ctx.fill();
            
            // Draw waypoint border
            this.ctx.strokeStyle = '#ffffff';
            this.ctx.lineWidth = 2;
            this.ctx.beginPath();
            this.ctx.arc(screen.x, screen.y, 8, 0, 2 * Math.PI);
            this.ctx.stroke();
            
            // Draw waypoint icon
            this.ctx.fillStyle = '#ffffff';
            this.ctx.font = '12px Arial';
            this.ctx.textAlign = 'center';
            this.ctx.fillText(waypoint.icon, screen.x, screen.y + 4);
            
            // Draw waypoint name
            this.ctx.fillStyle = '#ffffff';
            this.ctx.font = '10px Arial';
            this.ctx.fillText(waypoint.name, screen.x, screen.y + 22);
        });
    }
    
    drawGrid() {
        const gridSpacing = 20 * this.viewport.zoom;
        const offsetX = (this.viewport.centerX * this.mapSettings.scale * this.viewport.zoom) % gridSpacing;
        const offsetY = (this.viewport.centerY * this.mapSettings.scale * this.viewport.zoom) % gridSpacing;
        
        this.ctx.strokeStyle = this.mapSettings.gridColor;
        this.ctx.lineWidth = 1;
        
        for (let x = -offsetX; x < this.canvas.width + gridSpacing; x += gridSpacing) {
            this.ctx.beginPath();
            this.ctx.moveTo(x, 0);
            this.ctx.lineTo(x, this.canvas.height);
            this.ctx.stroke();
        }
        
        for (let y = -offsetY; y < this.canvas.height + gridSpacing; y += gridSpacing) {
            this.ctx.beginPath();
            this.ctx.moveTo(0, y);
            this.ctx.lineTo(this.canvas.width, y);
            this.ctx.stroke();
        }
    }
    
    worldToScreen(worldX, worldY) {
        const screenX = this.canvas.width / 2 + 
                       (worldX - this.viewport.centerX) * this.mapSettings.scale * this.viewport.zoom;
        const screenY = this.canvas.height / 2 - 
                       (worldY - this.viewport.centerY) * this.mapSettings.scale * this.viewport.zoom;
        return { x: screenX, y: screenY };
    }
    
    drawPath() {
        if (this.pathData.length < 2) return;
        
        this.ctx.strokeStyle = this.mapSettings.pathColor;
        this.ctx.lineWidth = 2;
        this.ctx.beginPath();
        
        let firstPoint = true;
        for (const point of this.pathData) {
            const screen = this.worldToScreen(point.x, point.y);
            
            if (firstPoint) {
                this.ctx.moveTo(screen.x, screen.y);
                firstPoint = false;
            } else {
                this.ctx.lineTo(screen.x, screen.y);
            }
        }
        
        this.ctx.stroke();
    }
    
    drawStartPosition() {
        const screen = this.worldToScreen(this.startPosition.x, this.startPosition.y);
        
        this.ctx.fillStyle = this.mapSettings.startPosColor;
        this.ctx.fillRect(screen.x - 6, screen.y - 6, 12, 12);
        
        this.ctx.strokeStyle = '#27ae60';
        this.ctx.lineWidth = 2;
        this.ctx.strokeRect(screen.x - 6, screen.y - 6, 12, 12);
        
        this.ctx.fillStyle = '#ffffff';
        this.ctx.font = '12px Arial';
        this.ctx.textAlign = 'center';
        this.ctx.fillText('START', screen.x, screen.y - 10);
    }
    
    drawCurrentPosition() {
        const screen = this.worldToScreen(this.currentPosition.x, this.currentPosition.y);
        
        this.ctx.fillStyle = this.mapSettings.currentPosColor;
        this.ctx.beginPath();
        this.ctx.arc(screen.x, screen.y, 8, 0, 2 * Math.PI);
        this.ctx.fill();
        
        this.ctx.strokeStyle = '#ffffff';
        this.ctx.lineWidth = 3;
        this.ctx.beginPath();
        this.ctx.moveTo(screen.x, screen.y);
        const headingRad = (this.currentPosition.heading || 0) * Math.PI / 180;
        const endX = screen.x + 15 * Math.sin(headingRad);
        const endY = screen.y - 15 * Math.cos(headingRad);
        this.ctx.lineTo(endX, endY);
        this.ctx.stroke();
        
        const time = Date.now() / 1000;
        const pulseRadius = 12 + 4 * Math.sin(time * 3);
        this.ctx.strokeStyle = this.mapSettings.currentPosColor;
        this.ctx.lineWidth = 2;
        this.ctx.beginPath();
        this.ctx.arc(screen.x, screen.y, pulseRadius, 0, 2 * Math.PI);
        this.ctx.stroke();
    }
    
    drawReverseNavigation() {
        if (!this.reverseNavData) return;
        
        if (this.reverseNavData.approaching_start) {
            const startScreen = this.worldToScreen(this.startPosition.x, this.startPosition.y);
            const currentScreen = this.worldToScreen(this.currentPosition.x, this.currentPosition.y);
            
            this.ctx.strokeStyle = this.mapSettings.targetColor;
            this.ctx.lineWidth = 3;
            this.ctx.setLineDash([10, 5]);
            this.ctx.beginPath();
            this.ctx.moveTo(currentScreen.x, currentScreen.y);
            this.ctx.lineTo(startScreen.x, startScreen.y);
            this.ctx.stroke();
            this.ctx.setLineDash([]);
        }
        
        if (this.reverseNavData.off_track) {
            const screen = this.worldToScreen(this.currentPosition.x, this.currentPosition.y);
            
            this.ctx.strokeStyle = '#e74c3c';
            this.ctx.lineWidth = 4;
            this.ctx.beginPath();
            this.ctx.arc(screen.x, screen.y, 20, 0, 2 * Math.PI);
            this.ctx.stroke();
            
            this.ctx.fillStyle = '#e74c3c';
            this.ctx.font = 'bold 14px Arial';
            this.ctx.textAlign = 'center';
            this.ctx.fillText('OFF TRACK', screen.x, screen.y + 35);
        }
    }
    
    drawCompass() {
        const compassX = this.canvas.width - 40;
        const compassY = 40;
        const compassRadius = 25;
        
        this.ctx.strokeStyle = '#ffffff';
        this.ctx.lineWidth = 2;
        this.ctx.beginPath();
        this.ctx.arc(compassX, compassY, compassRadius, 0, 2 * Math.PI);
        this.ctx.stroke();
        
        this.ctx.fillStyle = '#e74c3c';
        this.ctx.beginPath();
        this.ctx.arc(compassX, compassY - compassRadius + 5, 3, 0, 2 * Math.PI);
        this.ctx.fill();
        
        const heading = this.currentPosition.heading || 0;
        const needleRad = heading * Math.PI / 180;
        
        this.ctx.strokeStyle = '#f39c12';
        this.ctx.lineWidth = 3;
        this.ctx.beginPath();
        this.ctx.moveTo(compassX, compassY);
        const needleX = compassX + (compassRadius - 8) * Math.sin(needleRad);
        const needleY = compassY - (compassRadius - 8) * Math.cos(needleRad);
        this.ctx.lineTo(needleX, needleY);
        this.ctx.stroke();
        
        this.ctx.fillStyle = '#ffffff';
        this.ctx.font = '12px Arial';
        this.ctx.textAlign = 'center';
        this.ctx.fillText(`${heading.toFixed(0)}°`, compassX, compassY + compassRadius + 15);
    }
    
    drawScale() {
        const scaleLength = 50;
        const scaleMeters = scaleLength / (this.mapSettings.scale * this.viewport.zoom);
        
        const x = 20;
        const y = this.canvas.height - 30;
        
        this.ctx.strokeStyle = '#ffffff';
        this.ctx.lineWidth = 2;
        this.ctx.beginPath();
        this.ctx.moveTo(x, y);
        this.ctx.lineTo(x + scaleLength, y);
        this.ctx.stroke();
        
        this.ctx.beginPath();
        this.ctx.moveTo(x, y - 5);
        this.ctx.lineTo(x, y + 5);
        this.ctx.moveTo(x + scaleLength, y - 5);
        this.ctx.lineTo(x + scaleLength, y + 5);
        this.ctx.stroke();
        
        this.ctx.fillStyle = '#ffffff';
        this.ctx.font = '12px Arial';
        this.ctx.textAlign = 'center';
        const scaleText = scaleMeters >= 1 ? `${scaleMeters.toFixed(0)}m` : `${(scaleMeters * 100).toFixed(0)}cm`;
        this.ctx.fillText(scaleText, x + scaleLength / 2, y + 20);
    }
    
    updateZoomDisplay() {
        document.getElementById('zoomLevel').textContent = `${this.viewport.zoom.toFixed(1)}x`;
    }
    
    resizeCanvas(width, height) {
        this.canvas.width = width;
        this.canvas.height = height;
        this.redrawMap();
    }
}

// Dark Mode Toggle Function
function toggleDarkMode() {
    const body = document.body;
    const darkModeToggle = document.getElementById('darkModeToggle');
    
    if (body.getAttribute('data-theme') === 'dark') {
        // Switch to light mode
        body.removeAttribute('data-theme');
        darkModeToggle.textContent = '🌙';
        localStorage.setItem('theme', 'light');
    } else {
        // Switch to dark mode
        body.setAttribute('data-theme', 'dark');
        darkModeToggle.textContent = '☀️';
        localStorage.setItem('theme', 'dark');
    }
}

// Initialize theme on page load
function initializeTheme() {
    const savedTheme = localStorage.getItem('theme');
    const darkModeToggle = document.getElementById('darkModeToggle');
    
    if (savedTheme === 'dark') {
        document.body.setAttribute('data-theme', 'dark');
        darkModeToggle.textContent = '☀️';
    } else {
        darkModeToggle.textContent = '🌙';
    }
}

function toggleFullscreen() {
    const mapPanel = document.querySelector('.map-panel');
    const canvas = document.getElementById('mapCanvas');
    
    if (!app.isFullscreen) {
        // Enter fullscreen
        mapPanel.classList.add('map-fullscreen');
        const closeBtn = document.createElement('button');
        closeBtn.className = 'close-fullscreen';
        closeBtn.textContent = '✕ Close';
        closeBtn.onclick = toggleFullscreen;
        document.body.appendChild(closeBtn);
        
        app.resizeCanvas(window.innerWidth - 40, window.innerHeight - 100);
        app.isFullscreen = true;
    } else {
        // Exit fullscreen
        mapPanel.classList.remove('map-fullscreen');
        const closeBtn = document.querySelector('.close-fullscreen');
        if (closeBtn) closeBtn.remove();
        
        app.resizeCanvas(400, 300);
        app.isFullscreen = false;
    }
}

async function addWaypoint() {
    const name = document.getElementById('waypointName').value || 'Waypoint';
    const type = parseInt(document.getElementById('waypointType').value);
    
    try {
        const response = await fetch('/waypoints', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                name: name,
                type: type,
                use_current_position: true
            })
        });
        
        if (response.ok) {
            document.getElementById('waypointName').value = '';
            app.updateWaypointData();
        }
    } catch (error) {
        console.error('Add waypoint error:', error);
    }
}

function zoomIn() {
    app.viewport.zoom = Math.min(10, app.viewport.zoom * 1.5);
    app.updateZoomDisplay();
    app.redrawMap();
}

function zoomOut() {
    app.viewport.zoom = Math.max(0.1, app.viewport.zoom / 1.5);
    app.updateZoomDisplay();
    app.redrawMap();
}

function centerMap() {
    app.viewport.centerX = app.currentPosition.x;
    app.viewport.centerY = app.currentPosition.y;
    app.redrawMap();
}

function showFullPath() {
    if (app.pathData.length === 0) return;
    
    let minX = app.pathData[0].x, maxX = app.pathData[0].x;
    let minY = app.pathData[0].y, maxY = app.pathData[0].y;
    
    for (const point of app.pathData) {
        minX = Math.min(minX, point.x);
        maxX = Math.max(maxX, point.x);
        minY = Math.min(minY, point.y);
        maxY = Math.max(maxY, point.y);
    }
    
    app.viewport.centerX = (minX + maxX) / 2;
    app.viewport.centerY = (minY + maxY) / 2;
    
    const pathWidth = maxX - minX;
    const pathHeight = maxY - minY;
    const zoomX = (app.canvas.width * 0.8) / (pathWidth * app.mapSettings.scale);
    const zoomY = (app.canvas.height * 0.8) / (pathHeight * app.mapSettings.scale);
    app.viewport.zoom = Math.min(zoomX, zoomY, 10);
    
    app.updateZoomDisplay();
    app.redrawMap();
}

async function toggleReverse() {
    try {
        const response = await fetch('/toggle-reverse', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' }
        });
        
        if (response.ok) {
            const result = await response.json();
            console.log('Reverse mode toggle result:', result);
            
            // Update button state immediately
            const reverseBtn = document.getElementById('reverseBtn');
            if (result.new_mode === 'reverse') {
                reverseBtn.classList.add('active');
                reverseBtn.textContent = '⏹️ Stop';
                console.log('Reverse mode activated via web interface');
            } else {
                reverseBtn.classList.remove('active');
                reverseBtn.textContent = '↩️ Reverse Mode';
                console.log('Forward mode activated via web interface');
            }
        } else {
            console.error('Failed to toggle reverse mode:', response.status);
        }
    } catch (error) {
        console.error('Error toggling reverse mode:', error);
    }
}

// SOS Emergency function
async function triggerSOS() {
    if (confirm('🆘 EMERGENCY: Are you sure you want to send an SOS signal? This will alert rescue services!')) {
        try {
            const response = await fetch('/trigger-sos', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' }
            });
            
            if (response.ok) {
                const result = await response.json();
                console.log('SOS triggered:', result);
                
                // Show SOS confirmation in web interface
                const sosBtn = document.getElementById('sosBtn');
                sosBtn.style.background = '#27ae60';
                sosBtn.innerHTML = '✅ SOS SENT - Help Coming!';
                sosBtn.disabled = true;
                
                // Show alert popup
                alert('🆘 SOS SIGNAL SENT!\\n\\n📡 Emergency alert transmitted via BLE\\n🚁 Rescue services notified\\n📍 GPS coordinates shared\\n\\n⏱️ Help is on the way!');
                
                // Reset button after 10 seconds
                setTimeout(() => {
                    sosBtn.style.background = '#e74c3c';
                    sosBtn.innerHTML = '🆘 EMERGENCY SOS';
                    sosBtn.disabled = false;
                }, 10000);
                
            } else {
                alert('❌ Failed to send SOS signal. Please try again or use alternative communication.');
            }
        } catch (error) {
            console.error('SOS request failed:', error);
            alert('❌ Network error. SOS signal not sent. Please try again.');
        }
    }
}

let app;
document.addEventListener('DOMContentLoaded', () => {
    initializeTheme(); // Initialize dark mode theme
    app = new NavigatorApp();
    console.log('ESP32 Navigator Web Interface Initialized with Kalman Filter and Smart Waypoints');
});
)rawliteral";
  
  server.send(200, "application/javascript", js);
}


