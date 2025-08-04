import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool
from mavros_msgs.msg import State
from rclpy.callback_groups import ReentrantCallbackGroup
import subprocess
import time

class GPSLocalization(Node):
    def __init__(self):
        super().__init__('GPS_Localization')

        self.get_logger().info("MAVROS를 백그라운드로 실행 중...")
        self.mavros_process = subprocess.Popen(
            ["ros2", "run", "mavros", "mavros_node", "--ros-args", "-p", "fcu_url:=/dev/ttyACM0:57600"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

        time.sleep(5)  # MAVROS가 뜨는 시간 기다림

        self.cb_group = ReentrantCallbackGroup()
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming', callback_group=self.cb_group)
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.timer = self.create_timer(1.0, self.send_arm_request)

        self.armed = False
        self.armed_logged = False

        self.get_logger().info("GPS_Localization 노드 시작됨: ARM 요청 중...")

    #mavros 상태 수신 및 armed 콜
    def state_callback(self, msg):
        self.armed = msg.armed

        if self.armed and not self.armed_logged:
            self.get_logger().info("ARMED 되었습니다.")
            self.armed_logged = True
            self.timer.cancel()  # 더 이상 ARM 요청 안 보냄

            self.get_logger().info("Robot set")
            subprocess.Popen(
                ["ros2", "launch", "scout_mini_base", "marin_launch.py"])
            
    def send_arm_request(self):
        if self.armed_logged:
            return  # 이미 ARMED 되었다면 더 이상 요청하지 않음

        if not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("ARM 서비스가 아직 준비되지 않았습니다.")
            return

        req = CommandBool.Request()
        req.value = True

        future = self.arm_client.call_async(req)
        future.add_done_callback(self.handle_arm_response)

    def handle_arm_response(self, future):
        try:
            res = future.result()
            if res.success:
                self.get_logger().info("ARM 요청 보냄")
            else:
                self.get_logger().warn(f"ARM 요청 실패 (result: {res.result})")
        except Exception as e:
            self.get_logger().error(f"ARM 요청 중 예외 발생: {e}")

    def destroy_node(self):
        super().destroy_node()
        if self.mavros_process.poll() is None:
            self.get_logger().info("MAVROS 프로세스 종료 중...")
            self.mavros_process.terminate()

def main(args=None):
    rclpy.init(args=args)
    node = GPSLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
