#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Arduino.h>

// OLED Display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// MPU6050 object
Adafruit_MPU6050 mpu;
float zDeg = 0.0;
unsigned long lastTime = 0;

// Encoder pins (Set to ESP32 GPIO pins as per your ESP-IDF setup)
#define ENCODER_LEFT_A_PIN 6    // Set encoder A pin for left motor
#define ENCODER_LEFT_B_PIN 7    // Set encoder B pin for left motor
#define ENCODER_RIGHT_A_PIN 10  // Set encoder A pin for right motor
#define ENCODER_RIGHT_B_PIN 11  // Set encoder B pin for right motor

// Encoder counts and direction
volatile int encoder_left_count = 0;
volatile int encoder_right_count = 0;
volatile bool encoder_left_direction = true;   // True = Forward, False = Backward
volatile bool encoder_right_direction = true;  // True = Forward, False = Backward

// I2C settings from your ESP-IDF code
#define SDA_PIN 4
#define SCL_PIN 9

// Setup I2C
void setup() {
  // Start Serial communication for logging
  Serial.begin(115200);

  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Halt execution if OLED init fails
  }
  display.display();
  delay(2000);  // Wait for the OLED to initialize

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to initialize MPU6050");
    for (;;)
      ;  // Halt execution if MPU6050 init fails
  }
  Serial.println("MPU6050 Initialized");

  // Display splash messages with delay
  splash("Welcome");
  splash("Vacuum");
  splash("Robot");
  splash("Starting On");
  splash("Ready to take commands");

  // Encoder setup
  pinMode(ENCODER_LEFT_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B_PIN, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B_PIN, INPUT_PULLUP);

  // Attach interrupts to encoder A pins for both motors
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A_PIN), encoder_left_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A_PIN), encoder_right_ISR, CHANGE);
}

// Display splash messages
void splash(const char* message) {
  display.clearDisplay();
  display.setTextSize(2);  // Text size 2
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 20);
  display.print(message);
  display.display();
  delay(1000);  // 1 second delay
}

// Draw axis on the screen
void draw_axis() {
  display.drawLine(0, 32, 127, 32, SSD1306_WHITE);  // X-axis
  display.drawLine(64, 0, 64, 63, SSD1306_WHITE);   // Y-axis
}

// Log movement on the OLED screen
void log_movement(const char* movement) {
  display.setTextSize(1);
  display.setCursor(0, 55);
  display.print(movement);
  display.display();
}

// Encoder ISR for Left Motor
void encoder_left_ISR() {
  // Read the state of the encoder B pin for direction detection
  bool left_a_state = digitalRead(ENCODER_LEFT_A_PIN);
  bool left_b_state = digitalRead(ENCODER_LEFT_B_PIN);

  // Check direction based on the quadrature signal phase
  if (left_a_state == left_b_state) {
    encoder_left_count++;  // Forward movement
    encoder_left_direction = true;
  } else {
    encoder_left_count--;  // Backward movement
    encoder_left_direction = false;
  }
}

// // Encoder ISR for Right Motor
void encoder_right_ISR() {
  // Read the state of the encoder B pin for direction detection
  bool right_a_state = digitalRead(ENCODER_RIGHT_A_PIN);
  bool right_b_state = digitalRead(ENCODER_RIGHT_B_PIN);

  // Check direction based on the quadrature signal phase
  if (right_a_state == right_b_state) {
    encoder_right_count++;  // Forward movement
    encoder_right_direction = true;
  } else {
    encoder_right_count--;  // Backward movement
    encoder_right_direction = false;
  }
}

void drawZRotation(float zDeg) {
  // Circle
  display.drawCircle(64, 32, 25, SSD1306_WHITE);

  // Convert degrees to radians
  float rad = zDeg * 0.0174533;

  // End of rotating line
  int x = 64 + cos(rad) * 20;
  int y = 32 + sin(rad) * 20;

  display.drawLine(64, 32, x, y, SSD1306_WHITE);
}

// Main loop to display IMU data and movement logs
void loop() {
  // Clear previous data
  display.clearDisplay();

  // Read the Z-axis gyro data from MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  float z = g.gyro.z * 57.2958;  // Convert rad/s to deg/s
  zDeg += z*dt;

  if (zDeg < 0) zDeg += 360;
  if (zDeg >= 360) zDeg -= 360;

  // float z = g.gyro.z;  // Correct way to get the Z-axis gyroscope data

  // Display IMU data
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("IMU Z: ");
  display.print(z, 2);
  display.print(" deg/s");

  // Draw the axis
  draw_axis();

  // Determine movement based on encoder counts
  if (encoder_left_count > encoder_right_count) {
    if (encoder_left_count - encoder_right_count > 10 && !encoder_right_direction)  {
      log_movement("Moving Right");
    } else {
      log_movement("Moving Forward");
    }
  } else if (encoder_left_count < encoder_right_count) {
    if (encoder_right_count - encoder_left_count > 10 && !encoder_left_direction){
      log_movement("Moving Left");
    } else {
      log_movement("Moving Forward");
    }
  } else if (encoder_left_count == encoder_right_count && encoder_left_count > 0) {
    log_movement("Moving Forward");
  } else if (encoder_left_count == encoder_right_count && encoder_left_count < 0) {
    log_movement("Moving Backward");
  }else if (encoder_left_count == 0 && encoder_right_count == 0) {
    log_movement("Motors Idle");
  }
  drawZRotation(zDeg);

  // Update the display
  display.display();
  delay(100);  // Refresh every 100ms
}


// #include <stdio.h>
// #include <string.h>
// #include <math.h>

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

// #include "driver/i2c.h"
// #include "driver/gpio.h"
// #include "esp_timer.h"

// /* =====================================================
//    I2C CONFIG
// ===================================================== */
// #define SDA_PIN 4
// #define SCL_PIN 9
// #define I2C_PORT I2C_NUM_0

// /* =====================================================
//    OLED CONFIG
// ===================================================== */
// #define SSD1306_WIDTH 128
// #define SSD1306_HEIGHT 64
// #define SSD1306_ADDR 0x3C

// static uint8_t oled_buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

// /* =====================================================
//    MPU6050 CONFIG
// ===================================================== */
// #define MPU6050_ADDR 0x68
// #define REG_PWR_MGMT_1 0x6B
// #define REG_GYRO_ZOUT_H 0x47

// /* =====================================================
//    ENCODERS (same pins as Arduino sketch)
// ===================================================== */
// #define ENCODER_LEFT_A_PIN   6
// #define ENCODER_LEFT_B_PIN   7
// #define ENCODER_RIGHT_A_PIN 10
// #define ENCODER_RIGHT_B_PIN 11

// volatile int encoder_left_count  = 0;
// volatile int encoder_right_count = 0;
// volatile bool encoder_left_direction  = true;
// volatile bool encoder_right_direction = true;

// /* =====================================================
//    GYRO STATE
// ===================================================== */
// static float zDeg = 0.0f;
// static int64_t last_time_us = 0;

// /* =====================================================
//    FONT — EXACT FULL TABLE FROM YOUR ESP-IDF CODE
//    ASCII 32–127
// ===================================================== */
// static const uint8_t font5x7[][5] = { {0,0,0,0,0}, {0,0,0x5F,0,0}, {0,7,0,7,0}, {0x14,0x7F,0x14,0x7F,0x14}, {0x24,0x2A,0x7F,0x2A,0x12}, 
// {0x23,0x13,8,0x64,0x62}, {0x36,0x49,0x55,0x22,0x50}, {0,5,3,0,0}, {0,0x1C,0x22,0x41,0}, {0,0x41,0x22,0x1C,0}, {0x14,8,0x3E,8,0x14}, 
// {8,8,0x3E,8,8}, {0,0x50,0x30,0,0}, {8,8,8,8,8}, {0,0x60,0x60,0,0}, {0x20,0x10,8,4,2}, {0x3E,0x51,0x49,0x45,0x3E}, {0,0x42,0x7F,0x40,0}, 
// {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31}, {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39}, {0x3C,0x4A,0x49,0x49,0x30}, 
// {1,0x71,9,5,3}, {0x36,0x49,0x49,0x49,0x36}, {6,0x49,0x49,0x29,0x1E}, {0,0x36,0x36,0,0}, {0,0x56,0x36,0,0}, {8,0x14,0x22,0x41,0}, 
// {0x14,0x14,0x14,0x14,0x14}, {0,0x41,0x22,0x14,8}, {2,1,0x51,9,6}, {0x32,0x49,0x79,0x41,0x3E}, {0x7E,0x11,0x11,0x11,0x7E}, 
// {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22}, {0x7F,0x41,0x41,0x22,0x1C}, {0x7F,0x49,0x49,0x49,0x41}, {0x7F,9,9,9,1}, 
// {0x3E,0x41,0x49,0x49,0x7A}, {0x7F,8,8,8,0x7F}, {0,0x41,0x7F,0x41,0}, {0x20,0x40,0x41,0x3F,1}, {0x7F,8,0x14,0x22,0x41}, 
// {0x7F,0x40,0x40,0x40,0x40}, {0x7F,2,0x0C,2,0x7F}, {0x7F,4,8,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E}, {0x7F,9,9,9,6}, 
// {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,9,0x19,0x29,0x46}, {0x46,0x49,0x49,0x49,0x31}, {1,1,0x7F,1,1}, {0x3F,0x40,0x40,0x40,0x3F}, 
// {0x1F,0x20,0x40,0x20,0x1F}, {0x3F,0x40,0x38,0x40,0x3F}, {0x63,0x14,8,0x14,0x63}, {7,8,0x70,8,7}, {0x61,0x51,0x49,0x45,0x43}, 
// {0,0x7F,0x41,0x41,0}, {2,4,8,0x10,0x20}, {0,0x41,0x41,0x7F,0}, {4,2,1,2,4}, {0x40,0x40,0x40,0x40,0x40}, {0,1,2,4,0}, 
// {0x20,0x54,0x54,0x54,0x78}, {0x7F,0x48,0x44,0x44,0x38}, {0x38,0x44,0x44,0x44,0x20}, {0x38,0x44,0x44,0x48,0x7F}, {0x38,0x54,0x54,0x54,0x18}, 
// {8,0x7E,9,1,2}, {0x0C,0x52,0x52,0x52,0x3E}, {0x7F,8,4,4,0x78}, {0,0x44,0x7D,0x40,0}, {0x20,0x40,0x44,0x3D,0}, {0x7F,0x10,0x28,0x44,0}, 
// {0,0x41,0x7F,0x40,0}, {0x7C,4,0x18,4,0x7C}, {0x7C,8,4,4,0x78}, {0x38,0x44,0x44,0x44,0x38}, {0x7C,0x14,0x14,0x14,8}, {8,0x14,0x14,0x14,0x7C}, 
// {0x7C,8,4,4,8}, {0x48,0x54,0x54,0x54,0x20}, {4,0x3F,0x44,0x40,0x20}, {0x3C,0x40,0x40,0x20,0x7C}, {0x1C,0x20,0x40,0x20,0x1C}, 
// {0x3C,0x40,0x30,0x40,0x3C}, {0x44,0x28,0x10,0x28,0x44}, {0x0C,0x50,0x50,0x50,0x3C}, {0x44,0x64,0x54,0x4C,0x44} };

// /* =====================================================
//    I2C + OLED FUNCTIONS (UNCHANGED)
// ===================================================== */
// esp_err_t i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t data){
//     uint8_t buf[2]={reg,data};
//     return i2c_master_write_to_device(I2C_PORT,addr,buf,2,1000/portTICK_PERIOD_MS);
// }

// esp_err_t i2c_read_reg(uint8_t addr,uint8_t reg,uint8_t *data,size_t len){
//     return i2c_master_write_read_device(I2C_PORT,addr,&reg,1,data,len,1000/portTICK_PERIOD_MS);
// }

// void oled_cmd(uint8_t c){
//     uint8_t buf[2]={0x00,c};
//     i2c_master_write_to_device(I2C_PORT,SSD1306_ADDR,buf,2,10);
// }

// void oled_data(uint8_t *data,size_t len){
//     i2c_cmd_handle_t cmd=i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd,SSD1306_ADDR<<1,true);
//     i2c_master_write_byte(cmd,0x40,true);
//     i2c_master_write(cmd,data,len,true);
//     i2c_master_stop(cmd);
//     i2c_master_cmd_begin(I2C_PORT,cmd,50/portTICK_PERIOD_MS);
//     i2c_cmd_link_delete(cmd);
// }

// void oled_update(){
//     oled_cmd(0x21); oled_cmd(0); oled_cmd(127);
//     oled_cmd(0x22); oled_cmd(0); oled_cmd(7);
//     oled_data(oled_buffer,sizeof(oled_buffer));
// }

// void oled_clear(){
//     memset(oled_buffer,0,sizeof(oled_buffer));
// }

// /* =====================================================
//    DRAWING
// ===================================================== */
// void oled_pixel(int x,int y){
//     if(x<0||x>=128||y<0||y>=64) return;
//     oled_buffer[x+(y>>3)*128]|=1<<(y&7);
// }

// void oled_text(int x,int y,const char *s){
//     while(*s){
//         char c=*s++;
//         if(c<32||c>127) continue;
//         for(int i=0;i<5;i++){
//             uint8_t col=font5x7[c-32][i];
//             for(int j=0;j<7;j++)
//                 if(col&(1<<j)) oled_pixel(x+i,y+j);
//         }
//         x+=6;
//     }
// }

// void oled_line(int x0,int y0,int x1,int y1){
//     int dx=abs(x1-x0),sx=x0<x1?1:-1;
//     int dy=-abs(y1-y0),sy=y0<y1?1:-1;
//     int err=dx+dy;
//     while(1){
//         oled_pixel(x0,y0);
//         if(x0==x1&&y0==y1) break;
//         int e2=2*err;
//         if(e2>=dy){err+=dy;x0+=sx;}
//         if(e2<=dx){err+=dx;y0+=sy;}
//     }
// }

// void oled_circle(int cx,int cy,int r){
//     int x=r,y=0,err=0;
//     while(x>=y){
//         oled_pixel(cx+x,cy+y); oled_pixel(cx+y,cy+x);
//         oled_pixel(cx-y,cy+x); oled_pixel(cx-x,cy+y);
//         oled_pixel(cx-x,cy-y); oled_pixel(cx-y,cy-x);
//         oled_pixel(cx+y,cy-x); oled_pixel(cx+x,cy-y);
//         y++; err<=0 ? (err+=2*y+1) : (x--,err-=2*x+1);
//     }
// }

// /* =====================================================
//    MPU6050
// ===================================================== */
// void mpu_init(){ i2c_write_reg(MPU6050_ADDR,REG_PWR_MGMT_1,0x00); }

// float mpu_gyro_z(){
//     uint8_t b[2];
//     i2c_read_reg(MPU6050_ADDR,REG_GYRO_ZOUT_H,b,2);
//     return ((int16_t)(b[0]<<8|b[1]))/131.0f;
// }

// /* =====================================================
//    ENCODER ISRs (QUADRATURE)
// ===================================================== */
// static void IRAM_ATTR encoder_left_isr(void *arg){
//     bool a=gpio_get_level(ENCODER_LEFT_A_PIN);
//     bool b=gpio_get_level(ENCODER_LEFT_B_PIN);
//     if(a==b){encoder_left_count++;encoder_left_direction=true;}
//     else {encoder_left_count--;encoder_left_direction=false;}
// }

// static void IRAM_ATTR encoder_right_isr(void *arg){
//     bool a=gpio_get_level(ENCODER_RIGHT_A_PIN);
//     bool b=gpio_get_level(ENCODER_RIGHT_B_PIN);
//     if(a==b){encoder_right_count++;encoder_right_direction=true;}
//     else {encoder_right_count--;encoder_right_direction=false;}
// }

// /* =====================================================
//    SPLASH (FROM ESP-IDF CODE)
// ===================================================== */
// void splash(const char *s){
//     oled_clear();
//     oled_text(30,28,s);
//     oled_update();
//     vTaskDelay(pdMS_TO_TICKS(800));
// }

// /* =====================================================
//    MAIN
// ===================================================== */
// void app_main(){

//     /* I2C init */
//     i2c_config_t cfg={
//         .mode=I2C_MODE_MASTER,
//         .sda_io_num=SDA_PIN,
//         .sda_pullup_en=GPIO_PULLUP_ENABLE,
//         .scl_io_num=SCL_PIN,
//         .scl_pullup_en=GPIO_PULLUP_ENABLE,
//         .master.clk_speed=400000
//     };
//     i2c_param_config(I2C_PORT,&cfg);
//     i2c_driver_install(I2C_PORT,I2C_MODE_MASTER,0,0,0);

//     /* OLED init */
//     oled_cmd(0xAE);
//     oled_cmd(0x20); oled_cmd(0x00);
//     oled_cmd(0xA6);
//     oled_cmd(0xAF);

//     /* Splash sequence */
//     splash("Welcome");
//     splash("Vacuum");
//     splash("Robot");
//     splash("Starting On");
//     splash("Ready");

//     /* MPU init */
//     mpu_init();

//     /* Encoder GPIO */
//     gpio_config_t io={
//         .pin_bit_mask=(1ULL<<ENCODER_LEFT_A_PIN)|(1ULL<<ENCODER_LEFT_B_PIN)|
//                        (1ULL<<ENCODER_RIGHT_A_PIN)|(1ULL<<ENCODER_RIGHT_B_PIN),
//         .mode=GPIO_MODE_INPUT,
//         .pull_up_en=GPIO_PULLUP_ENABLE,
//         .intr_type=GPIO_INTR_ANYEDGE
//     };
//     gpio_config(&io);
//     gpio_install_isr_service(0);
//     gpio_isr_handler_add(ENCODER_LEFT_A_PIN,encoder_left_isr,NULL);
//     gpio_isr_handler_add(ENCODER_RIGHT_A_PIN,encoder_right_isr,NULL);

//     last_time_us=esp_timer_get_time();

//     while(1){
//         oled_clear();

//         /* --- Gyro integration (Arduino logic) --- */
//         int64_t now=esp_timer_get_time();
//         float dt=(now-last_time_us)/1000000.0f;
//         last_time_us=now;

//         float z=mpu_gyro_z();
//         zDeg+=z*dt;
//         if(zDeg<0)zDeg+=360;
//         if(zDeg>=360)zDeg-=360;

//         /* --- IMU display --- */
//         char buf[32];
//         snprintf(buf,sizeof(buf),"IMU Z %.2f",z);
//         oled_text(0,0,buf);

//         /* --- Axis --- */
//         oled_line(0,32,127,32);
//         oled_line(64,0,64,63);

//         /* --- Rotation --- */
//         oled_circle(64,32,25);
//         float rad=zDeg*0.0174533f;
//         oled_line(64,32,64+cosf(rad)*20,32+sinf(rad)*20);

//         /* --- Movement logic (EXACT Arduino) --- */
//         int l=encoder_left_count;
//         int r=encoder_right_count;
//         const char *move="Motors Idle";

//         if(l>r){
//             if((l-r)>10 && !encoder_right_direction) move="Moving Right";
//             else move="Moving Forward";
//         }else if(l<r){
//             if((r-l)>10 && !encoder_left_direction) move="Moving Left";
//             else move="Moving Forward";
//         }else if(l==r && l>0) move="Moving Forward";
//         else if(l==r && l<0) move="Moving Backward";

//         oled_text(0,56,move);

//         oled_update();
//         vTaskDelay(pdMS_TO_TICKS(100));
//     }
// }
