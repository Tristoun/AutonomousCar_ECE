#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <vector>

#include <ESP32Servo.h>
#define PIN_SG90 4 // Broche de sortie utilisée

const char MODE = 'V'; //S = mode servo moteur / V = mode voiture

Servo sg90;
bool brake_active = false;

unsigned long lastBrakeTime = 0;
const unsigned long BRAKE_HOLD_TIME = 500; // ms

int pos_servo = 0;
// ========== CONFIGURATION HARDWARE ==========
// Pins moteurs TB6612
//Motor A
const uint16_t PWMA = 25;         
const uint16_t AIN2 = 17;        
const uint16_t AIN1 = 21;

//Motor B        
const uint16_t BIN1 = 22;       
const uint16_t BIN2 = 23;        
const uint16_t PWMB = 26;   


// LIDAR Serial
#define SerialLidar Serial2

// Motor PWM Channels
const int channel_A = 5; // Moteur droit
const int channel_B = 6; // Moteur gauche
const uint16_t ANALOG_WRITE_BITS = 8;
const uint16_t MAX_PWM = 255;
const uint16_t MIN_PWM = MAX_PWM / 5;
const int freq = 100000;             // 20 kHz recommandé
const int resolution = ANALOG_WRITE_BITS;
// ESP01S MAC Address
uint8_t esp01MAC[] = {0x34, 0x5F, 0x45, 0x59, 0x74, 0xE6};
uint8_t myMAC[6];

// LIDAR Safety Parameters
const uint16_t BRAKE_DISTANCE = 600;     // Distance en mm pour freinage
const float FRONT_ANGLE_MIN = 315.0;     // Zone avant 
const float FRONT_ANGLE_MAX = 45.0;      
const uint16_t MIN_VALID_DISTANCE = 50;  // Distance minimale valide en mm

// Timing
const unsigned long SPEED_SEND_INTERVAL = 500;
const unsigned long HB_TIMEOUT = 1000;

// ========== TABLE CRC POUR LD19 ==========
static const uint8_t crcTable[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 
    0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 
    0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8, 
    0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43, 
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93, 
    0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90, 
    0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40, 
    0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb, 
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04, 
    0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 
    0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab, 
    0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20, 
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 
    0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd, 
    0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d, 
    0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96, 
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 
    0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45, 
    0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a, 
    0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01, 
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 
    0x7f, 0x32, 0xe5, 0xa8
};

// ========== CLASSE LIDAR POINT (ton code original) ==========
class LidarPoint {
private:
    uint16_t _distance;
    uint8_t _intensity;
    float _angle;

public:
    LidarPoint(uint16_t distance, uint8_t intensity, float angle)
        : _distance(distance), _intensity(intensity), _angle(angle) {}

    inline uint16_t distance() const { return _distance; }
    inline uint8_t intensity() const { return _intensity; }
    inline float angle() const { return _angle; }

    String toString() const {
        String result = "(distance=";
        result += String(_distance);
        result += ", intensity=";
        result += String(_intensity);
        result += ", angle=";
        result += String(_angle / 100.0, 2);
        result += "°)";
        return result;
    }
};

// ========== VARIABLES GLOBALES ==========
int speedValue = 0;

// Communication
unsigned long lastHB = 0;
unsigned long lastSpeedSend = 0;
bool esp01_present = false;

// Joystick
typedef struct {
    int joy_X;
    int joy_Y;
} joystick_data;

joystick_data joystick_values = {2048, 2048}; // Centre par défaut

// LIDAR
bool obstacleDetected = false;
uint16_t closestObstacleDistance = 9999;

// Mutex et tâches
SemaphoreHandle_t xMutexJoystick;
SemaphoreHandle_t xMutexObstacle;
TaskHandle_t CommunicationTask;
TaskHandle_t LidarTask;

// ========== FONCTIONS LIDAR (ton code original) ==========
uint8_t _calCRC8FromBuffer(uint8_t* p, uint8_t lenWithoutCRCCheckValue) {
    uint8_t crc = 0xD8;
    for (uint16_t i = 0; i < lenWithoutCRCCheckValue; i++) {
        crc = crcTable[(crc ^ *p++) & 0xff];
    }
    return crc;
}

uint16_t _get2BytesLsbMsb(byte buffer[], int index) {
    return (buffer[index + 1] << 8) | buffer[index];
}

uint16_t angleStep(uint16_t startAngle, uint16_t endAngle, unsigned int lenMinusOne = 11) {
    if (startAngle <= endAngle) {
        return (endAngle - startAngle) / lenMinusOne;
    } else {
        return (36000 + endAngle - startAngle) / lenMinusOne;
    }
}

uint16_t angleFromStep(uint16_t startAngle, uint16_t step, unsigned int indice) {
    return (startAngle + (step * indice)) % 36000;
}

std::vector<LidarPoint> getPoints() {
    std::vector<LidarPoint> points;
    
    // Attendre que des données soient disponibles
    if (SerialLidar.available() < 47) {
        return points; // Pas assez de données
    }
    
    if (!SerialLidar.find("\x54\x2C")) {
        return points;
    } else {
        byte buffer[45];
        size_t nbrBytesReceived = SerialLidar.readBytes(buffer, 45);
        if (nbrBytesReceived != 45) {
            Serial.println("lidar_reader.getPoints : error, wrong number of bytes received (" + String((uint32_t) nbrBytesReceived) + ")");
        } else {
            uint16_t speed = _get2BytesLsbMsb(buffer, 0);
            uint16_t startAngle = _get2BytesLsbMsb(buffer, 2);

            LidarPoint data[] = {
                LidarPoint(_get2BytesLsbMsb(buffer, 4), buffer[6], 0),
                LidarPoint(_get2BytesLsbMsb(buffer, 7), buffer[9], 0),
                LidarPoint(_get2BytesLsbMsb(buffer, 10), buffer[12], 0),
                LidarPoint(_get2BytesLsbMsb(buffer, 13), buffer[15], 0),
                LidarPoint(_get2BytesLsbMsb(buffer, 16), buffer[18], 0),
                LidarPoint(_get2BytesLsbMsb(buffer, 19), buffer[21], 0),
                LidarPoint(_get2BytesLsbMsb(buffer, 22), buffer[24], 0),
                LidarPoint(_get2BytesLsbMsb(buffer, 25), buffer[27], 0),
                LidarPoint(_get2BytesLsbMsb(buffer, 28), buffer[30], 0),
                LidarPoint(_get2BytesLsbMsb(buffer, 31), buffer[33], 0),
                LidarPoint(_get2BytesLsbMsb(buffer, 34), buffer[36], 0),
                LidarPoint(_get2BytesLsbMsb(buffer, 37), buffer[39], 0)
            };

            uint16_t endAngle = _get2BytesLsbMsb(buffer, 40);
            uint16_t timestamp = _get2BytesLsbMsb(buffer, 42);
            uint8_t crcCheck = buffer[44];

            if (_calCRC8FromBuffer(buffer, 44) == crcCheck) {
                uint16_t step = angleStep(startAngle, endAngle);
                for (unsigned int i = 0; i < 12; i++) {
                    points.push_back(
                        LidarPoint(
                            data[i].distance(),
                            data[i].intensity(),
                            angleFromStep(startAngle, step, i)
                        )
                    );
                }
            }
        }
    }
    return points;
}

std::vector<LidarPoint> getFullScan() {
    std::vector<LidarPoint> scan;
    while (true) {
        auto points = getPoints();
        if (!points.empty()) {
            for (auto& p : points) {
                scan.push_back(p);
            }
            if (points.back().angle() < points.front().angle()) {
                break;
            }
        }
    }
    return scan;
}

// ========== MODE PROMISCUOUS (RSSI) ==========
extern "C" {
    void wifi_promiscuous_cb(void* buf, wifi_promiscuous_pkt_type_t type) {
        if (type != WIFI_PKT_MGMT && type != WIFI_PKT_DATA) return;
        
        wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;
        uint8_t *payload = pkt->payload;
        uint8_t *mac_src = payload + 10;
        
        if (memcmp(mac_src, esp01MAC, 6) == 0) {
            int8_t rssi = pkt->rx_ctrl.rssi;
            char msg[32];
            snprintf(msg, sizeof(msg), "RSSI:%d", (int)rssi);
            esp_now_send(esp01MAC, (uint8_t *)msg, strlen(msg));
        }
    }
}

// ========== ESP-NOW CALLBACKS ==========
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    String msg;
    for (int i = 0; i < len; i++) {
        msg += (char)incomingData[i];
    }
    
    if (msg == "HB") {
        lastHB = millis();
        esp01_present = true;
        return;
    }
    
    Serial.print("RX←ESP01S: ");
    Serial.println(msg);
    
    if (msg.startsWith("JOY:")) {
        int commaIndex = msg.indexOf(',');
        if (commaIndex > 0) {
            int VRx = msg.substring(4, commaIndex).toInt();
            int VRy = msg.substring(commaIndex + 1).toInt();
            
            if (xSemaphoreTake(xMutexJoystick, portMAX_DELAY) == pdTRUE) {
                joystick_values.joy_X = VRx;
                joystick_values.joy_Y = VRy;
                xSemaphoreGive(xMutexJoystick);
            }
        }
    }
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // Callback optionnel
}

// ========== CONTRÔLE MOTEURS ==========
void channel_A_Ctrl(float pwmInputA) {
    int pwmIntA = round(pwmInputA);
    if (pwmIntA == 0) {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, LOW);
        return;
    }

    if (pwmIntA > 0) {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        ledcWrite(channel_A, constrain(pwmIntA, MIN_PWM, MAX_PWM));
    } else {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        ledcWrite(channel_A, -constrain(pwmIntA, -MAX_PWM, -MIN_PWM));
    }
}

void channel_B_Ctrl(float pwmInputB) {
    int pwmIntB = round(pwmInputB);
    if (pwmIntB == 0) {
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, LOW);
        return;
    }

    if (pwmIntB > 0) {
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
        ledcWrite(channel_B, constrain(pwmIntB, MIN_PWM, MAX_PWM));
    } else {
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
        ledcWrite(channel_B, -constrain(pwmIntB, -MAX_PWM, -MIN_PWM));
    }
}



// ========== TASK 1: COMMUNICATION (Core 0) ==========
void CommunicationTaskCode(void *pvParameters) {
    Serial.println("✓ Communication Task on Core " + String(xPortGetCoreID()));
    
    while (1) {
        unsigned long t = millis();
        
        // Timeout ESP01S
        if (esp01_present && (t - lastHB > HB_TIMEOUT)) {
            String msg = "RSSI:0";
            esp_now_send(esp01MAC, (uint8_t*)msg.c_str(), msg.length());
            esp01_present = false;
        }
        
        // Vérification obstacle
        bool obstacleNow = false;
        uint16_t closestDist = 9999;
        
        if (xSemaphoreTake(xMutexObstacle, 10 / portTICK_PERIOD_MS) == pdTRUE) {
            obstacleNow = obstacleDetected;
            closestDist = closestObstacleDistance;
            xSemaphoreGive(xMutexObstacle);
        }
        
        if (obstacleNow) {
            // BRAKE !
            String msg = "BRAKE";
            brake_active = true;
            esp_now_send(esp01MAC, (uint8_t *)msg.c_str(), msg.length());
            Serial.print("🛑 BRAKE! Obstacle @ ");
            Serial.print(closestDist);
            Serial.println("mm");
            channel_A_Ctrl(0);
            channel_B_Ctrl(0);
            speedValue = 0;
            lastBrakeTime = millis();  
        }  else {
            // On ne libère le brake que si assez de temps s'est écoulé
            if (brake_active && millis() - lastBrakeTime > BRAKE_HOLD_TIME) {
                brake_active = false;
                Serial.println("✅ Brake released");
            }
        }
        int currentX, currentY;
        if (xSemaphoreTake(xMutexJoystick, 10 / portTICK_PERIOD_MS) == pdTRUE) {
            currentX = joystick_values.joy_X;
            currentY = joystick_values.joy_Y;
            xSemaphoreGive(xMutexJoystick);
        }
        if (!brake_active || currentY <= 500) {

            
            float speedA, speedB;
            int speed_servo = 0;

            if(MODE == 'V') {

                int deadzone = 15;
                int X = currentX - 512;
                int Y = currentY - 512;

                if (abs(X) < deadzone) X = 0;
                if (abs(Y) < deadzone) Y = 0;

                // Map maintenant de -511 → 511 vers -255 → 255
                X = map(X, -511, 511, -255, 255);
                Y = map(Y, -511, 511, 255, -255);


                // Calcul vitesse moteurs
                int motorA = Y + X;
                int motorB = Y - X;

                // Clamp
                motorA = constrain(motorA, -255, 255);
                motorB = constrain(motorB, -255, 255);
                channel_A_Ctrl(motorA);
                channel_B_Ctrl(motorB);


                // Calcul magnitude
                int absA = abs(motorA);
                int absB = abs(motorB);

                // Choisir la valeur la plus "rapide"
                int maxMotor = max(absA, absB);

                // Convertir en %
                speedValue = map(maxMotor, 0, MAX_PWM, 0, 100);
            }

            else if(MODE == 'S') {

                if(currentY > 520) {
                    speed_servo = 15 * currentY / 1023;
                }
                else if(currentY < 520) {
                    speed_servo = 15 * (currentY+1023) / 1023;
                }
                else {
                    speed_servo = 0;
                }
                if(!brake_active) {

                if(!obstacleNow) {
                    if(currentY == 1023 && pos_servo <= 180) {
                        pos_servo += speed_servo;
                        sg90.write(pos_servo);
                        delay(10);
                    }
                    else if (currentY == 0 && pos_servo >= 0) {
                        pos_servo -= speed_servo;
                        sg90.write(pos_servo);
                        delay(10);
                    }
                
                }
                }


                speedValue = 100*speed_servo/15;
            }

        }
        
        // Envoi périodique vitesse
        if (t - lastSpeedSend > SPEED_SEND_INTERVAL) {
            lastSpeedSend = t;
            char msg[16];
            snprintf(msg, sizeof(msg), "SPEED:%d", speedValue);
            esp_now_send(esp01MAC, (uint8_t*)msg, strlen(msg));
        }
        
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

// ========== TASK 2: LIDAR (Core 1) - VERSION RAPIDE ==========
void LidarTaskCode(void *pvParameters) {
    Serial.println("✓ LIDAR Task on Core " + String(xPortGetCoreID()));
    
    unsigned long lastValidRead = millis();
    int successCount = 0;
    int errorCount = 0;
    
    while (1) {
        // Lecture paquet par paquet (12 points) au lieu d'attendre un scan complet
        std::vector<LidarPoint> points = getPoints();
        
        if (!points.empty()) {
            successCount++;
            lastValidRead = millis();
            
            bool obstacleFound = false;
            uint16_t minDistance = 9999;
            
            // Traitement immédiat des 12 points reçus
            for (const auto& p : points) {
                float angle = p.angle() / 100.0; // Conversion en degrés
                uint16_t dist = p.distance();
                
                // Vérification zone avant (330° à 30°)
                bool inFrontZone = (angle >= FRONT_ANGLE_MIN && angle <= 360) || (angle <= FRONT_ANGLE_MAX && angle >= 0);
                
                if (inFrontZone && dist > MIN_VALID_DISTANCE && dist < BRAKE_DISTANCE) {
                    obstacleFound = true;
                    if (dist < minDistance) {
                        minDistance = dist;
                    }
                    
                    // Log détaillé
                    // Serial.print("⚠️  ");
                    // Serial.print(dist);
                    // Serial.print("mm @ ");
                    // Serial.print(angle, 1);
                    // Serial.println("°");
                }
            }
            
            // Mise à jour immédiate du flag obstacle
            if (xSemaphoreTake(xMutexObstacle, 10 / portTICK_PERIOD_MS) == pdTRUE) {
                if (obstacleFound) {
                    obstacleDetected = true;
                    closestObstacleDistance = minDistance;
                } else {
                    // Ne désactive le flag que si vraiment pas d'obstacle
                    // (évite les faux négatifs entre deux paquets)
                    if (!obstacleDetected || minDistance > BRAKE_DISTANCE + 100) {
                        obstacleDetected = false;
                        closestObstacleDistance = 9999;
                    }
                }
                xSemaphoreGive(xMutexObstacle);
            }
            
            // Afficher stats toutes les 200 lectures (environ toutes les 2 secondes)
            if (successCount % 200 == 0) {
                Serial.print("📊 LIDAR: ");
                Serial.print(successCount);
                Serial.println(" packets OK");
            }
        } else {
            errorCount++;
            
            // Avertir si pas de données pendant 2 secondes
            if (millis() - lastValidRead > 2000) {
                Serial.println("❌ LIDAR: No data for 2s!");
                Serial.print("   Errors: ");
                Serial.print(errorCount);
                Serial.print(" | Available: ");
                Serial.println(SerialLidar.available());
                lastValidRead = millis();
            }
        }
        
        // Pas de delay ! Lecture en continu pour réactivité maximale
        vTaskDelay(1 / portTICK_PERIOD_MS); // Yield minimal pour le scheduler
    }
}

// ========== SETUP ==========
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n╔═══════════════════════════════╗");
    Serial.println("║   ESP32 ROBOT LIDAR LD19      ║");
    Serial.println("╚═══════════════════════════════╝\n");
    
    // Configuration LIDAR
    SerialLidar.begin(230400, SERIAL_8N1, 16, 17);
    Serial.println("⏳ Waiting for LIDAR to stabilize (3s)...");
    delay(3000);
    
    // Test connexion LIDAR
    Serial.print("📡 LIDAR Serial available bytes: ");
    Serial.println(SerialLidar.available());
    
    if (SerialLidar.available() > 0) {
        Serial.println("✓ LIDAR is sending data!");
        // Afficher les 20 premiers octets en hexa
        Serial.print("   First bytes (hex): ");
        for (int i = 0; i < min(20, SerialLidar.available()); i++) {
            byte b = SerialLidar.read();
            if (b < 0x10) Serial.print("0");
            Serial.print(b, HEX);
            Serial.print(" ");
        }
        Serial.println();
    } else {
        Serial.println("⚠️  WARNING: No data from LIDAR!");
        Serial.println("   Check:");
        Serial.println("   - TX LIDAR → GPIO16 ESP32");
        Serial.println("   - 5V power connected");
        Serial.println("   - LIDAR motor is spinning");
    }
    
    // Mutex
    xMutexJoystick = xSemaphoreCreateMutex();
    xMutexObstacle = xSemaphoreCreateMutex();
    
    if (!xMutexJoystick || !xMutexObstacle) {
        Serial.println("❌ Mutex creation failed");
        while (1);
    }
    
    // WiFi + ESP-NOW
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    esp_read_mac(myMAC, ESP_MAC_WIFI_STA);
    Serial.printf("My MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
            myMAC[0], myMAC[1], myMAC[2], myMAC[3], myMAC[4], myMAC[5]);
    
    if (esp_now_init() != ESP_OK) {
        Serial.println("❌ ESP-NOW init failed");
        while (1);
    }
    
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, esp01MAC, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        Serial.println("✓ ESP01S peer added");
    }
    
    esp_now_register_recv_cb(onDataRecv);
    esp_now_register_send_cb(onDataSent);
    
    // Promiscuous mode
    wifi_promiscuous_filter_t filt;
    filt.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA;
    esp_wifi_set_promiscuous_filter(&filt);
    esp_wifi_set_promiscuous_rx_cb(&wifi_promiscuous_cb);
    esp_wifi_set_promiscuous(true);
    
    Serial.println("✓ ESP-NOW + Promiscuous enabled");
    
    // Moteurs
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);
    
    ledcSetup(channel_A, freq, resolution);
    //Bind PWMA pin to A-channel  
    ledcAttachPin(PWMA, channel_A);

    ledcSetup(channel_B, freq, resolution);
    ledcAttachPin(PWMB, channel_B);
    
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    
    Serial.println("✓ Motors initialized");
    
    // Tâches multi-cœurs
    xTaskCreatePinnedToCore(
        CommunicationTaskCode,
        "Communication",
        10000,
        NULL,
        1,
        &CommunicationTask,
        0 // Core 0
    );
    
    xTaskCreatePinnedToCore(
        LidarTaskCode,
        "LIDAR",
        10000,
        NULL,
        1,
        &LidarTask,
        1 // Core 1
    );

    sg90.setPeriodHertz(50); // Fréquence PWM pour le SG90
    sg90.attach(PIN_SG90, 500, 2400); // Largeur minimale et maximale de l'impulsion (en µs) pour aller de 0° à 180°
    sg90.write(0);

    Serial.println("✓ Dual-core tasks created");
    Serial.println("\n🚀 ROBOT READY!\n");
}

// ========== LOOP ==========
void loop() {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}