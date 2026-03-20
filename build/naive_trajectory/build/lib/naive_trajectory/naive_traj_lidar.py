#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16MultiArray, Int32
import time

class CorridorStepMaster(Node):
    def __init__(self):
        super().__init__("corridor_step_master")

        # Config PWM
        self.PWM_FORWARD = 100
        self.PWM_TURN    = 115 # Puissance de rotation contrôlée
        
        # Seuils
        self.FRONT_DANGER = 0.7
        self.SIDE_THRESHOLD = 0.5 

        # Machine à états
        self.STATE_DRIVE = "DRIVE"
        self.STATE_WAIT  = "WAIT"
        self.state = self.STATE_DRIVE
        self.state_end_time = 0.0

        # ROS
        self.ranges = []
        self.motor_pub = self.create_publisher(Int16MultiArray, '/motor_pwm', 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(Int32, '/lap_count', self.lap_callback, 10)

        self.create_timer(0.05, self.control_loop)
        self.lap_count = 1

        self.get_logger().info("🐢 Mode Step-by-Step activé pour SLAM stable")

    def lidar_callback(self, msg):
        self.ranges = msg.ranges
    
    def lap_callback(self, msg) :
        self.lap_count = msg.data

    def get_dist(self, idx):
        if not self.ranges: return 10.0
        # Moyenne sur 10 points pour stabiliser
        points = [self.ranges[i % len(self.ranges)] for i in range(idx-5, idx+5)]
        valid = [p for p in points if 0.05 < p < 10.0]
        return sum(valid)/len(valid) if valid else 10.0

    def control_loop(self):
        if not self.ranges: return
        if self.lap_count > 1 : 
            return
        now = self.get_clock().now().nanoseconds / 1e9

        # --- GESTION DE L'ATTENTE (STABILISATION SLAM) ---
        if self.state == self.STATE_WAIT:
            if now < self.state_end_time:
                self.send_motors(0, 0) # On ne bouge plus du tout
                return
            else:
                self.state = self.STATE_DRIVE # On repart
                # self.get_logger().info("▶️ Reprise de la marche")

        # --- LECTURE CAPTEURS ---
        d_front = self.get_dist(0)
        d_left  = self.get_dist(55)
        d_right = self.get_dist(405)

        # --- LOGIQUE DE DÉCISION ---
        l_pwm, r_pwm = 0, 0

        # Si obstacle devant ou gros déséquilibre sur les côtés
        if d_front < self.FRONT_DANGER or abs(d_right - d_left) > self.SIDE_THRESHOLD:
            # 1. On décide du sens de rotation
            if d_left > d_right:
                l_pwm, r_pwm = -self.PWM_TURN, self.PWM_TURN # Gauche
            else:
                l_pwm, r_pwm = self.PWM_TURN, -self.PWM_TURN # Droite
            
            # 2. On lance une rotation TRÈS courte (0.2 seconde)
            self.send_motors(l_pwm, r_pwm)
            time.sleep(0.2) # Exceptionnellement court, toléré ici pour l'impulsion
            
            # 3. On passe en mode ATTENTE pour stabiliser le mapping
            self.send_motors(0, 0)
            self.state = self.STATE_WAIT
            self.state_end_time = now + 0.8 # On attend 0.8s sans bouger
            # self.get_logger().warn("🛑 Pivot bref -> Pause stabilisation SLAM")
            return

        # Sinon, marche avant classique
        else:
            self.send_motors(self.PWM_FORWARD, self.PWM_FORWARD)

    def send_motors(self, left, right):
        msg = Int16MultiArray(data=[int(left), int(right)])
        self.motor_pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(CorridorStepMaster())
    rclpy.shutdown()

if __name__ == '__main__':
    main()