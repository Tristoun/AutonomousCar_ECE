#include <Arduino.h>

// Pins moteurs TB6612
//Motor A
const uint16_t PWMA = 25;         
const uint16_t AIN2 = 17;        
const uint16_t AIN1 = 21;

//Motor B        
const uint16_t BIN1 = 22;       
const uint16_t BIN2 = 23;        
const uint16_t PWMB = 26;   


// Motor PWM Channels
const int channel_A = 5; // Moteur droit
const int channel_B = 6; // Moteur gauche
const uint16_t ANALOG_WRITE_BITS = 8;
const uint16_t MAX_PWM = 255;
const uint16_t MIN_PWM = MAX_PWM / 5;
const int freq = 100000;             // 20 kHz recommandé
const int resolution = ANALOG_WRITE_BITS;


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