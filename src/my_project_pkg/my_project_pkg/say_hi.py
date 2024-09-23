import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('say_hi')
        timer_period = 1.0 #sec
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        print('saludos from my_project_pkg.')    

def main(args=None):
    try:
        rclpy.init(args=args)
        say_hi = MyNode()
        rclpy.spin(say_hi)
    except KeyboardInterrupt:
        print('exit node')
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
