#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data # 센서 데이터 QoS 프로파일 임포트

class DataRelayAndStatePublisher(Node):
    def __init__(self):
        super().__init__('data_relay_and_state_publisher')

        # 1. 발행자 설정 (기존 DummyDataPublisher의 발행 토픽 유지)
        self.event_publisher_ = self.create_publisher(NavSatFix, '/scooter', 10)
        self.realtime_publisher_ = self.create_publisher(NavSatFix, '/gps_data', qos_profile_sensor_data)
        self.state_publisher_ = self.create_publisher(String, '/Event_msg', 10)

        self.flag = 0

        # 2. 구독자 설정 (MapSend에서 참고한 구독 부분)
        # 킥보드 위치 이벤트 구독 (외부에서 들어오는 이벤트 GPS 데이터)
        self.event_subscription_ = self.create_subscription(
            NavSatFix,
            '/scooter_location', # 이 토픽에서 데이터를 구독합니다.
            self.event_callback,
            10)
        self.get_logger().info('Subscribing to /scooter_location for event GPS.')

        # 로봇 현재 위치 구독 (외부에서 들어오는 실시간 GPS 데이터)
        self.realtime_subscription_ = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global', # 이 토픽에서 데이터를 구독합니다.
            self.realtime_callback,
            qos_profile_sensor_data) # 실시간 데이터이므로 센서 QoS 적용
        self.get_logger().info('Subscribing to /mavros/global_position/global for realtime GPS.')

        # 3. 로봇 상태 관리 (기존 DummyDataPublisher의 상태 로직 유지)
        self.states = ['00', '01', '10', '11']
        self.state_index = 0
        # 2초마다 로봇 상태를 발행하는 타이머
        self.state_publish_timer_ = self.create_timer(2.0, self.publish_robot_state)

        self.get_logger().info('DataRelayAndStatePublisher node started, acting as data relay and state generator.')

    # 구독: /scooter_location 콜백 함수
    def event_callback(self, msg: NavSatFix):
        # 수신된 이벤트 GPS 데이터를 /scooter 토픽으로 그대로 발행
        self.event_publisher_.publish(msg)
        self.get_logger().info(f'Relayed Event_GPS from /scooter_location: lat={msg.latitude}, lon={msg.longitude}')
        # 이 곳에서는 MapSend와 달리 Robot_State를 강제로 '10'으로 바꾸고 타이머로 '00'으로 리셋하는 로직이 없습니다.
        # Robot_State는 아래 publish_robot_state 함수에 의해 주기적으로 발행됩니다.

    # 구독: /mavros/global_position/global 콜백 함수
    def realtime_callback(self, msg: NavSatFix):
        # 수신된 실시간 GPS 데이터를 /gps_data 토픽으로 그대로 발행
        self.realtime_publisher_.publish(msg)
        self.get_logger().info(f'Relayed RealTime_GPS from /mavros/global_position/global: lat={msg.latitude}, lon={msg.longitude}')

    # 로봇 상태를 주기적으로 발행하는 함수 (기존 DummyDataPublisher 로직)
    def publish_robot_state(self):
        state_msg = String()
        state_msg.data = self.states[self.state_index]
        self.state_publisher_.publish(state_msg)
        self.get_logger().info(f'Publishing Robot_State: state={state_msg.data}')
        self.state_index = 2

def main(args=None):
    rclpy.init(args=args)
    node = DataRelayAndStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()