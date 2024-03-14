import rclpy
from rclpy.node import Node
import PyQt5
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QLabel, QMainWindow

from std_msgs.msg import Bool, String

class Gui(QMainWindow, Node):
    def __init__(self) -> None:
        Node.__init__("GUI")
        self.test_publisher = self.create_publisher(Bool, "test", 10)
        self.test_label = self.create_subscription(String, "test_str", self.display_test, 10)
    
    def send_test(self):
        self.test_publisher.publish(Bool(data=True))
    
    def display_test(self, msg:String):
        print(msg)
        self.lable1.setText(msg)
        
    

def main():
    rclpy.init()
    gui = Gui()
    rclpy.spin(gui)
    
    
if __name__ == "__main__":
    main()