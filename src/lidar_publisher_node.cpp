#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <sensor_msgs/msg/imu.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rmw_microros/rmw_microros.h> 
#include <std_msgs/msg/int16_multi_array.h>
#include <Wire.h>
#include <vector>

#include "lidar_header.hpp"
#include "motor_header.hpp"

// --- CONFIGURATION ---
const int MPU_ADDR = 0x68;
#define WIFI_SSID "Arecetri"
#define WIFI_PASSWORD "arece1234"
#define AGENT_IP IPAddress(10, 49, 241, 183) 
#define AGENT_PORT 8888
#define MAX_POINTS 460 

typedef struct {
    float acc[3];
    float gyro[3];
    uint64_t timestamp;
} imu_data_t;

// Structure LaserScan modifiée
typedef struct {
    float ranges[MAX_POINTS];
    float intensities[MAX_POINTS];
    uint64_t timestamp;
} lidar_scan_t;

QueueHandle_t imu_queue;
QueueHandle_t lidar_queue; 

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_publisher_t lidar_pub;
rcl_publisher_t imu_pub;
rcl_subscription_t motor_sub;

sensor_msgs__msg__LaserScan laser_msg;
float laser_ranges_buffer[MAX_POINTS];
float laser_intensities_buffer[MAX_POINTS];

sensor_msgs__msg__Imu imu_msg;
std_msgs__msg__Int16MultiArray motor_msg;

enum states { WAITING_AGENT, AGENT_CONNECTED, AGENT_DISCONNECTED } state; 

float gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;
float accel_bias_x = 0, accel_bias_y = 0, accel_bias_z = 0;


void init_imu_msg() {
    rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");
    imu_msg.orientation_covariance[0] = -1;
}

void init_laser_scan_msg() {
    rosidl_runtime_c__String__assign(&laser_msg.header.frame_id, "laser_link"); 
    laser_msg.ranges.data = laser_ranges_buffer;
    laser_msg.ranges.capacity = MAX_POINTS;
    laser_msg.intensities.data = laser_intensities_buffer;
    laser_msg.intensities.capacity = MAX_POINTS;
}

void configure_mpu6050() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); Wire.write(0x00);
    Wire.endTransmission();
    delay(100);
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B); Wire.write(0x00);
    Wire.endTransmission();
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1C); Wire.write(0x00);
    Wire.endTransmission();
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1A); Wire.write(0x03);
    Wire.endTransmission();
}

void calibrate_mpu6050(int samples = 500) {
    long acc_x = 0, acc_y = 0, acc_z = 0, gyr_x = 0, gyr_y = 0, gyr_z = 0;
    for (int i = 0; i < samples; i++) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x3B);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, 14, true);
        int16_t AcX = Wire.read() << 8 | Wire.read();
        int16_t AcY = Wire.read() << 8 | Wire.read();
        int16_t AcZ = Wire.read() << 8 | Wire.read();
        Wire.read(); Wire.read();
        int16_t GyX = Wire.read() << 8 | Wire.read();
        int16_t GyY = Wire.read() << 8 | Wire.read();
        int16_t GyZ = Wire.read() << 8 | Wire.read();
        acc_x += AcX; acc_y += AcY; acc_z += AcZ - 16384;
        gyr_x += GyX; gyr_y += GyY; gyr_z += GyZ;
        delay(5);
    }
    accel_bias_x = (float)acc_x / samples; accel_bias_y = (float)acc_y / samples; accel_bias_z = (float)acc_z / samples;
    gyro_bias_x = (float)gyr_x / samples; gyro_bias_y = (float)gyr_y / samples; gyro_bias_z = (float)gyr_z / samples;
}

void motor_pwm_callback(const void * msg_in) {
    const std_msgs__msg__Int16MultiArray * msg = (const std_msgs__msg__Int16MultiArray *)msg_in;
    if (msg->data.size < 2) return;  

    int left_pwm = msg->data.data[0];  
    int right_pwm = msg->data.data[1]; 

    channel_A_Ctrl(left_pwm);   
    channel_B_Ctrl(right_pwm);  
}

bool create_entities() {
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    
    if (rclc_node_init_default(&node, "esp32_node", "", &support) != RCL_RET_OK) return false;

    rclc_publisher_init_default(&lidar_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "scan");
    rclc_publisher_init_default(&imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu_data");
    
    rclc_subscription_init_default(&motor_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray), "motor_pwm");

    static int16_t motor_data_buffer[4]; 
    motor_msg.data.capacity = 4;
    motor_msg.data.data = motor_data_buffer;
    motor_msg.data.size = 0;

    executor = rclc_executor_get_zero_initialized_executor();
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    
    rclc_executor_add_subscription(&executor, &motor_sub, &motor_msg, &motor_pwm_callback, ON_NEW_DATA);

    init_laser_scan_msg(); 
    init_imu_msg();
    rmw_uros_sync_session(1000);
    return true;
}

void destroy_entities() {
    rcl_publisher_fini(&lidar_pub, &node);
    rcl_publisher_fini(&imu_pub, &node);
    rcl_node_fini(&node);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);
}

// Fonction utilitaire pour réinitialiser un tableau scan (vide = distance 0)
void reset_scan(lidar_scan_t* scan) {
    for(int i = 0; i < MAX_POINTS; i++) {
        scan->ranges[i] = 0.0f;
        scan->intensities[i] = 0.0f;
    }
}

void sensor_motor_task(void * pvParameters) {
    float last_angle = 0;
    unsigned long last_imu_time = 0;
    const unsigned long IMU_INTERVAL = 50; 

    lidar_scan_t* current_scan = new lidar_scan_t();
    reset_scan(current_scan);

    for (;;) {
        // IMU SAMPLING
        if (millis() - last_imu_time > IMU_INTERVAL) {
            imu_data_t imu_raw;
            Wire.beginTransmission(MPU_ADDR);
            Wire.write(0x3B);
            Wire.endTransmission(false);
            if (Wire.requestFrom(MPU_ADDR, 14, true) == 14) { 
                int16_t AcX = Wire.read() << 8 | Wire.read();
                int16_t AcY = Wire.read() << 8 | Wire.read();
                int16_t AcZ = Wire.read() << 8 | Wire.read();
                Wire.read(); Wire.read();
                int16_t GyX = Wire.read() << 8 | Wire.read();
                int16_t GyY = Wire.read() << 8 | Wire.read();
                int16_t GyZ = Wire.read() << 8 | Wire.read();

                imu_raw.acc[0] = ((float)AcX - accel_bias_x) / 16384.0f * 9.806f;
                imu_raw.acc[1] = ((float)AcY - accel_bias_y) / 16384.0f * 9.806f;
                imu_raw.acc[2] = ((float)AcZ - accel_bias_z) / 16384.0f * 9.806f;
                imu_raw.gyro[0] = ((float)GyX - gyro_bias_x) / 131.0f * (PI / 180.0f);
                imu_raw.gyro[1] = ((float)GyY - gyro_bias_y) / 131.0f * (PI / 180.0f);
                imu_raw.gyro[2] = ((float)GyZ - gyro_bias_z) / 131.0f * (PI / 180.0f);
                imu_raw.timestamp = rmw_uros_epoch_nanos();

                xQueueSend(imu_queue, &imu_raw, 0);
            }
            last_imu_time = millis();
        }

        // LIDAR SAMPLING
        while (Serial2.available() > 47) {
            std::vector<LidarPoint> newPoints = getPoints();
            if (newPoints.empty()) continue;

            float current_angle = newPoints.back().angle();
            
            for (auto &p : newPoints) {
                float angle_rad = (p.angle() / 100.0f) * (PI / 180.0f);
                float dist_m = p.distance() / 1000.0f;
                
                // On mappe l'angle (0 à 2*PI) sur un index (0 à MAX_POINTS-1)
                int index = round((angle_rad / (2.0f * PI)) * MAX_POINTS);
                if (index >= MAX_POINTS) index = 0; // Sécurité si angle = 360° pile
                if (index < 0) index = 0;

                // On ne garde que les points valides
                if (dist_m > 0.05f && dist_m < 12.0f) {
                    current_scan->ranges[index] = dist_m;
                    current_scan->intensities[index] = (float)p.intensity();
                }
            }

            // Détection de fin de tour (chute drastique de l'angle)
            if (current_angle < last_angle - 20000) {
                current_scan->timestamp = rmw_uros_epoch_nanos();
                if(xQueueSend(lidar_queue, &current_scan, 0) == pdPASS) { 
                    current_scan = new lidar_scan_t(); 
                    reset_scan(current_scan);
                } else {
                    reset_scan(current_scan); // Nettoyage si la queue est pleine
                }
            }
            last_angle = current_angle;
        }
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}

void microros_task(void * pvParameters) {
    imu_data_t incoming_imu;
    lidar_scan_t* incoming_lidar = NULL;

    for (;;) {
        switch (state) {
            case WAITING_AGENT:
                if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
                    if (create_entities()) state = AGENT_CONNECTED;
                }
                break;

            case AGENT_CONNECTED:
                if (WiFi.status() != WL_CONNECTED) {
                    state = AGENT_DISCONNECTED;
                    break;
                }

                // Publish IMU
                if (xQueueReceive(imu_queue, &incoming_imu, 0) == pdPASS) {
                    imu_msg.header.stamp.sec = incoming_imu.timestamp / 1000000000;
                    imu_msg.header.stamp.nanosec = incoming_imu.timestamp % 1000000000;
                    imu_msg.linear_acceleration.x = incoming_imu.acc[0];
                    imu_msg.linear_acceleration.y = incoming_imu.acc[1];
                    imu_msg.linear_acceleration.z = incoming_imu.acc[2];
                    imu_msg.angular_velocity.x = incoming_imu.gyro[0];
                    imu_msg.angular_velocity.y = incoming_imu.gyro[1];
                    imu_msg.angular_velocity.z = incoming_imu.gyro[2];
                    rcl_publish(&imu_pub, &imu_msg, NULL);
                }

                // Publish LaserScan FIXÉ
                if (xQueueReceive(lidar_queue, &incoming_lidar, 0) == pdPASS) {
                    laser_msg.header.stamp.sec = incoming_lidar->timestamp / 1000000000;
                    laser_msg.header.stamp.nanosec = incoming_lidar->timestamp % 1000000000;
                    
                    // On fixe la géométrie du scan en dur (un cercle complet)
                    laser_msg.angle_min = 0.0f;
                    laser_msg.angle_max = 2.0f * PI;
                    laser_msg.angle_increment = (2.0f * PI) / MAX_POINTS;
                    
                    laser_msg.time_increment = 0.0;
                    laser_msg.scan_time = 0.0;
                    laser_msg.range_min = 0.05f;  
                    laser_msg.range_max = 12.0f;  
                    
                    laser_msg.ranges.size = MAX_POINTS;
                    laser_msg.intensities.size = MAX_POINTS;

                    memcpy(laser_msg.ranges.data, incoming_lidar->ranges, MAX_POINTS * sizeof(float));
                    memcpy(laser_msg.intensities.data, incoming_lidar->intensities, MAX_POINTS * sizeof(float));

                    rcl_publish(&lidar_pub, &laser_msg, NULL);
                    delete incoming_lidar; 
                }

                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
                break;

            case AGENT_DISCONNECTED:
                destroy_entities();
                state = WAITING_AGENT;
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(230400, SERIAL_8N1, 16, 17);
    
    set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT);
    Wire.begin(4, 5);
    Wire.setClock(400000);
    configure_mpu6050();
    calibrate_mpu6050();

    pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
    ledcSetup(channel_A, freq, resolution); ledcAttachPin(PWMA, channel_A);
    ledcSetup(channel_B, freq, resolution); ledcAttachPin(PWMB, channel_B);

    imu_queue = xQueueCreate(10, sizeof(imu_data_t)); 
    lidar_queue = xQueueCreate(2, sizeof(lidar_scan_t*)); 

    state = WAITING_AGENT;

    xTaskCreatePinnedToCore(sensor_motor_task, "Sensors", 8192, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(microros_task, "microRos", 12288, NULL, 2, NULL, 0);
}

void loop() {
    vTaskDelete(NULL);
}