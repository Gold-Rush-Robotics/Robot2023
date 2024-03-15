from PyQt5.QtCore import Qt, QEventLoop
import rclpy
from rclpy.node import Node
import PyQt5
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QLabel, QMainWindow, QHBoxLayout, QListWidget, QListWidgetItem

from std_msgs.msg import Bool, String

class GRR_Window(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        
        self.setWindowTitle("GRR SECON APP")
        self.button = QPushButton("Ready to start? No ")
        self.button.setCheckable(True)
        
        self.label = QLabel()
        
        horiz_layout = QHBoxLayout()
        
        self.node_list = QListWidget()
        
        main_pane = QVBoxLayout()
        main_pane.addWidget(self.label)
        main_pane.addWidget(self.button)
    
        horiz_layout.addLayout(main_pane)
        horiz_layout.addWidget(self.node_list)
        
        container = QWidget()
        container.setLayout(horiz_layout)
        self.setCentralWidget(container)
        self.showMaximized()
        self.show()

class Gui(Node):
    def __init__(self, application:QApplication, window:GRR_Window) -> None:
        super().__init__("GUI")
        self.test_publisher = self.create_publisher(Bool, "test", 10)
        self.test_label = self.create_subscription(String, "test_str", self.display_test, 10)
        self.gui_loop_timer = self.create_timer(.1, self.gui_loop)
        self.app = application
        self.window = window
        self.previous_nodes = []
        self.bind()
        
    def bind(self):
        self.window.button.clicked.connect(self.send_test)
        
    def send_test(self, checked):
        self.window.button.setText(f"Ready to Start? {'Yes' if checked else 'No '}")
        self.test_publisher.publish(Bool(data=checked))
    
    def display_test(self, msg:String):
        print(msg)
        self.window.label.setText(msg.data)
        
    def gui_loop(self):
        self.app.processEvents(flags=QEventLoop.ProcessEventsFlag.AllEvents)     
        nodes = [((y + "/") if not y  == "/" else "/") + x for x,y in self.get_node_names_and_namespaces()]
        if not nodes == self.previous_nodes:
            self.window.node_list.clear()
            self.window.node_list.addItems(nodes)
            self.previous_nodes = nodes
            
        
        
    

def main():
    app = QApplication([])
    window = GRR_Window()
    rclpy.init()
    gui = Gui(app, window)
    rclpy.spin(gui)
    
    
if __name__ == "__main__":
    main()