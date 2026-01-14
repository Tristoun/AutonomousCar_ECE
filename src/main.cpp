#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <vector>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>

// ================= HARDWARE CONFIGURATION =================

// --- I2C (MPU6050) ---
#define I2C_SDA 32
#define I2C_SCL 33

// --- SERVO ---
#define PIN_SG90 4 
Servo sg90;
int pos_servo = 90; 

// --- MOTORS (TB6612) ---
const uint16_t PWMA = 25;         
const uint16_t AIN2 = 17;     
const uint16_t AIN1 = 21;
const uint16_t BIN1 = 22;      
const uint16_t BIN2 = 23;        
const uint16_t PWMB = 26;   

const int channel_A = 5; 
const int channel_B = 6; 
const int freq = 10000; 
const int resolution = 8;

// --- LIDAR (LD19) ---
#define SerialLidar Serial2
#define LIDAR_RX_PIN 16

// ================= ORIGINAL LIDAR LOGIC =================

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
};

uint8_t _calCRC8FromBuffer(uint8_t* p, uint8_t lenWithoutCRCCheckValue) {
    uint8_t crc = 0xD8;
    for (uint16_t i = 0; i < lenWithoutCRCCheckValue; i++) crc = crcTable[(crc ^ *p++) & 0xff];
    return crc;
}

uint16_t _get2BytesLsbMsb(byte buffer[], int index) {
    return (buffer[index + 1] << 8) | buffer[index];
}

uint16_t angleStep(uint16_t startAngle, uint16_t endAngle, unsigned int lenMinusOne = 11) {
    if (startAngle <= endAngle) return (endAngle - startAngle) / lenMinusOne;
    else return (36000 + endAngle - startAngle) / lenMinusOne;
}

uint16_t angleFromStep(uint16_t startAngle, uint16_t step, unsigned int indice) {
    return (startAngle + (step * indice)) % 36000;
}

std::vector<LidarPoint> getPoints() {
    std::vector<LidarPoint> points;
    if (SerialLidar.available() < 47) return points; 
    if (!SerialLidar.find("\x54\x2C")) return points; 
    
    byte buffer[45];
    size_t nbrBytesReceived = SerialLidar.readBytes(buffer, 45);
    
    if (nbrBytesReceived == 45) {
        uint16_t startAngle = _get2BytesLsbMsb(buffer, 2); 
        uint16_t endAngle = _get2BytesLsbMsb(buffer, 40);
        struct RawPoint { uint16_t dist; uint8_t intens; };
        RawPoint rawData[12];
        for(int i=0; i<12; i++) {
            int base = 4 + (i * 3);
            rawData[i].dist = _get2BytesLsbMsb(buffer, base);
            rawData[i].intens = buffer[base + 2];
        }
        uint8_t crcCheck = buffer[44];
        if (_calCRC8FromBuffer(buffer, 44) == crcCheck) {
            uint16_t step = angleStep(startAngle, endAngle);
            for (unsigned int i = 0; i < 12; i++) {
                points.push_back(LidarPoint(rawData[i].dist, rawData[i].intens, angleFromStep(startAngle, step, i)));
            }
        }
    }
    return points;
}

// ================= GLOBALS & WEB =================

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

Adafruit_MPU6050 mpu;
bool mpu_ready = false;
float robot_yaw = 0.0;
unsigned long last_gyro_time = 0;

SemaphoreHandle_t xMutexLidar;
std::vector<LidarPoint> sharedLidarScan;

// ================= HTML/JS (BINARY OPTIMIZED) =================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>ESP32 Rover Lidar</title>
    <style>
        body { font-family: Arial; background-color: #111; color: #eee; text-align: center; overflow: hidden; }
        canvas { background-color: #222; border-radius: 50%; border: 2px solid #555; margin-top: 10px; cursor: crosshair; }
        .controls { margin: 10px auto; padding: 10px; width: 80%; max-width: 600px; display: flex; justify-content: center; align-items: center; gap: 20px; }
        .stats { font-size: 1.2rem; margin-bottom: 5px; }
        .red { color: #ff4444; font-weight: bold; }
        label { font-size: 1rem; color: #aaa; }
        input[type=range] { width: 200px; }
    </style>
</head>
<body>
    <div class="stats">
        Yaw: <span id="yawVal" class="red">0.0</span>&deg; | 
        Points: <span id="ptsVal">0</span>
    </div>
    
    <div class="controls">
        <label>Zoom:</label>
        <input type="range" id="zoomSlider" min="0.05" max="1.5" step="0.05" value="0.15">
        <label id="zoomLabel">0.15x</label>
    </div>

    <canvas id="radar" width="600" height="600"></canvas>

    <script>
        var gateway = `ws://${window.location.hostname}/ws`;
        var websocket;
        var canvas = document.getElementById('radar');
        var ctx = canvas.getContext('2d');
        var centerX = canvas.width / 2;
        var centerY = canvas.height / 2;
        var zoomSlider = document.getElementById('zoomSlider');
        var zoomLabel = document.getElementById('zoomLabel');
        var scale = 0.15; // pixels per mm

        zoomSlider.oninput = function() {
            scale = parseFloat(this.value);
            zoomLabel.innerText = scale.toFixed(2) + "x";
        }

        function initWebSocket() {
            websocket = new WebSocket(gateway);
            websocket.binaryType = "arraybuffer"; // IMPORTANT: Enable Binary Receive
            
            websocket.onopen = function(event) { console.log('Connection opened'); };
            websocket.onclose = function(event) { setTimeout(initWebSocket, 2000); };
            
            websocket.onmessage = function(event) {
                if (event.data instanceof ArrayBuffer) {
                    // --- BINARY DATA (LIDAR) ---
                    parseLidarPacket(event.data);
                } else {
                    // --- TEXT DATA (IMU JSON) ---
                    var data = JSON.parse(event.data);
                    if (data.type === "imu") {
                        document.getElementById('yawVal').innerText = data.yaw.toFixed(1);
                    }
                }
            };
        }

        function parseLidarPacket(buffer) {
            var dataView = new DataView(buffer);
            var points = [];
            // Each point is 4 bytes: 2 bytes Angle (x10), 2 bytes Distance
            var count = buffer.byteLength / 4;
            
            document.getElementById('ptsVal').innerText = count;

            // Clear Screen
            ctx.fillStyle = 'rgba(34, 34, 34, 1)';
            ctx.fillRect(0, 0, canvas.width, canvas.height);
            
            // Draw Grid
            drawGrid();

            // Draw Robot
            ctx.fillStyle = '#0f0';
            ctx.fillRect(centerX - 4, centerY - 4, 8, 8);

            // Draw Points
            ctx.fillStyle = '#f00';
            for (var i = 0; i < count; i++) {
                // Read 16-bit integers (Little Endian)
                var angleInt = dataView.getUint16(i * 4, true); 
                var dist = dataView.getUint16(i * 4 + 2, true);
                
                var angle = (angleInt / 10.0) * (Math.PI / 180.0);
                
                var x = centerX + (dist * scale) * Math.cos(angle); 
                var y = centerY + (dist * scale) * Math.sin(angle);
                
                ctx.fillRect(x, y, 2, 2);
            }
        }

        function drawGrid() {
            ctx.strokeStyle = '#444';
            ctx.lineWidth = 1;
            ctx.textAlign = "center";
            ctx.font = "12px Arial";
            ctx.fillStyle = "#aaa";
            
            var maxRadius = Math.max(canvas.width, canvas.height) / 2;
            for (let dist = 1000; dist * scale < maxRadius; dist += 1000) {
                let r = dist * scale;
                ctx.beginPath(); 
                ctx.arc(centerX, centerY, r, 0, 2 * Math.PI); 
                ctx.stroke();
                ctx.fillText((dist/1000) + "m", centerX, centerY - r - 2);
            }
        }
        window.onload = initWebSocket;
    </script>
</body>
</html>
)rawliteral";

// ================= TASKS =================

void LidarTask(void *pvParameters) {
    std::vector<LidarPoint> currentScan;
    currentScan.reserve(400); 
    float lastAngle = 0;

    while (1) {
        std::vector<LidarPoint> newPoints = getPoints();
        
        if (!newPoints.empty()) {
            for (const auto& p : newPoints) {
                if(p.distance() > 50 && p.distance() < 8000) {
                    currentScan.push_back(p);
                }
            }
            
            float currentAngle = newPoints.back().angle() / 100.0;
            // Scan Complete Logic
            if (currentAngle < lastAngle - 100.0) { 
                if (xSemaphoreTake(xMutexLidar, 5) == pdTRUE) {
                    sharedLidarScan = currentScan;
                    xSemaphoreGive(xMutexLidar);
                }
                currentScan.clear();
                currentScan.reserve(400);
            }
            lastAngle = currentAngle;
        }
        vTaskDelay(1); 
    }
}

void WebSensorTask(void *pvParameters) {
    // 1. WiFi & Server Setup
    WiFi.softAP("ESP32_Rover_Lidar", "12345678");
    Serial.print("AP Started. IP: ");
    Serial.println(WiFi.softAPIP());

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/html", index_html);
    });
    server.addHandler(&ws);
    server.begin();

    // 2. Hardware Setup
    Wire.begin(I2C_SDA, I2C_SCL);
    
    // MPU Setup
    if (mpu.begin()) {
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        mpu_ready = true;
        last_gyro_time = millis();
        Serial.println("MPU6050 Ready");
    }

    // Reuse Buffer for Binary Data (Max 500 points * 4 bytes = 2000 bytes)
    uint8_t binBuffer[2048]; 

    while (1) {
        // OPTIMIZATION: Check if anyone is connected before doing heavy work
        bool clientConnected = (ws.count() > 0);

        if (mpu_ready) {
            sensors_event_t a, g, temp;
            mpu.getEvent(&a, &g, &temp);
            unsigned long now = millis();
            float dt = (now - last_gyro_time) / 1000.0;
            last_gyro_time = now;
            
            float gyroZ = g.gyro.z * 57.2958; 
            if (abs(gyroZ) > 0.5) robot_yaw += gyroZ * dt;

            // Send IMU Data (Low bandwidth, keep as JSON)
            if(clientConnected) {
                JsonDocument jsonIMU; 
                jsonIMU["type"] = "imu";
                jsonIMU["yaw"] = (int)(robot_yaw * 10) / 10.0; 
                String output;
                serializeJson(jsonIMU, output);
                ws.textAll(output);
            }
        }

        // --- LIDAR BINARY SEND (OPTIMIZED) ---
        if (clientConnected) {
            std::vector<LidarPoint> scanToSend;
            if (xSemaphoreTake(xMutexLidar, 5) == pdTRUE) {
                scanToSend = sharedLidarScan;
                xSemaphoreGive(xMutexLidar);
            }

            if (!scanToSend.empty()) {
                size_t packetSize = 0;
                
                // Pack into Binary Buffer: [Angle LSB, Angle MSB, Dist LSB, Dist MSB] ...
                for (const auto& p : scanToSend) {
                    if (packetSize + 4 >= sizeof(binBuffer)) break; // Safety check

                    uint16_t angleFixed = (uint16_t)(p.angle() / 10.0); // 0-3600 (Decidegree)
                    uint16_t distFixed = p.distance();

                    // Little Endian packing
                    binBuffer[packetSize++] = angleFixed & 0xFF;
                    binBuffer[packetSize++] = (angleFixed >> 8) & 0xFF;
                    binBuffer[packetSize++] = distFixed & 0xFF;
                    binBuffer[packetSize++] = (distFixed >> 8) & 0xFF;
                }
                
                // Send Binary!
                ws.binaryAll(binBuffer, packetSize);
            }
        }
        
        ws.cleanupClients();
        // Slightly reduced refresh rate to allow WiFi stack to breathe
        vTaskDelay(120 / portTICK_PERIOD_MS); 
    }
}

void setup() {
    Serial.begin(115200);
    xMutexLidar = xSemaphoreCreateMutex();

    // Motor Pins
    pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
    ledcSetup(channel_A, freq, resolution); ledcAttachPin(PWMA, channel_A);
    ledcSetup(channel_B, freq, resolution); ledcAttachPin(PWMB, channel_B);
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);

    // Servo
    sg90.setPeriodHertz(50);
    sg90.attach(PIN_SG90, 500, 2400);
    sg90.write(pos_servo);

    // Lidar
    SerialLidar.begin(230400, SERIAL_8N1, LIDAR_RX_PIN, 17);
    pinMode(AIN2, OUTPUT); 
    digitalWrite(AIN2, LOW); 

    // Tasks
    xTaskCreatePinnedToCore(LidarTask, "Lidar", 10000, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(WebSensorTask, "Web", 20000, NULL, 1, NULL, 0);
}

void loop() {
    vTaskDelay(1000);
}