import rclpy
from rclpy.node import Node
#importar standarmsgs para usar un String
from std_msgs.msg import String


class MyPublisher(Node):
    def __init__(self):
        super().__init__('my_publisher')
        #agregar publicador a la instanci (tópico)
        self.publisher = self.create_publisher(String, 'my_topic_uno', 10)
        timer_period = 1.0 #sec
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'message # %d' %self.i
        self.publisher.publish(msg)   
        self.get_logger().info("publishing: %s" % msg.data)
        self.i += 1

def main(args=None):
    try:
        rclpy.init(args=args)
        publisher = MyPublisher()
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        print('exit node')
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()