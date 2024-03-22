from PyQt5.QtCore import Qt, QEventLoop, QUrl
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import PyQt5
from PyQt5.QtWidgets import QSizePolicy, QApplication, QWidget, QPushButton, QVBoxLayout, QLabel, QMainWindow, QHBoxLayout, QListWidget, QListWidgetItem, QFileDialog
from PyQt5.QtMultimedia import QMediaContent, QMediaPlayer
from PyQt5.QtMultimediaWidgets import QVideoWidget
import sys
from std_msgs.msg import Bool, String, Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from PyQt5.QtGui import QPixmap, QColor



class GRR_Window(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        
        self.setWindowTitle("GRR SECON APP")
        self.start_button = QPushButton("Ready to start? No ")
        self.start_button.setFixedHeight(60)
        self.start_button.setCheckable(True)     
        
        self.kill_motors_button = QPushButton("Drivetrain Goals: Yes")  
        self.kill_motors_button.setCheckable(True)
        self.kill_motors_button.setFixedHeight(120)
        
        self.reset_servos_button = QPushButton("Reset Servos")
        self.reset_servos_button.setFixedHeight(60)
        
        self.state_machine_button = QPushButton("Launch State Machine")
        self.state_machine_button.setCheckable(True)
        self.state_machine_button.setFixedHeight(60)
        
        self.debug_label = QLabel("DEBUG VALUES")
        
        #media player instance None, QMediaPlayer.VideoSurface
        self.player = QMediaPlayer(None, QMediaPlayer.VideoSurface)

        horiz_layout = QHBoxLayout()
        
        
        self.videoWidget = QVideoWidget()


        self.middle_pane = QVBoxLayout()
        self.node_list = QListWidget()
        self.node_list.setFixedWidth(200)
        
        self.label = QLabel("State Machine not running")
        
        self.middle_pane.addWidget(self.node_list)
        self.middle_pane.addWidget(self.label)
        
        self.main_pane = QVBoxLayout()
        self.main_pane.addWidget(self.debug_label)
        self.main_pane.addWidget(self.kill_motors_button)
        self.main_pane.addWidget(self.reset_servos_button)
        self.main_pane.addWidget(self.state_machine_button)
        self.main_pane.addWidget(self.start_button)
        
        # self.main_pane.addWidget(self.videoWidget)
        horiz_layout.addLayout(self.main_pane)
        horiz_layout.addLayout(self.middle_pane)
        horiz_layout.addWidget(self.videoWidget)

        container = QWidget()
        container.setLayout(horiz_layout)
        self.player.setVideoOutput(self.videoWidget)
        #self.setCentralWidget(videoWidget)
        # Connect the error signal to a slot
        #self.player.error.connect(self.handleError)

        self.setCentralWidget(container)
        self.showMaximized()
        self.show()

    # Define a method to handle errors
    def handleError(self, error):
        print("Error:", error)

    def promoVid2(self):
        # fileName,_ = QFileDialog.getOpenFileName(self, ".","Images (*.mp4) (*.avi) (*.png)")
        fileName = get_package_share_directory('grr_guis') + '/multimedia/grr_logo.mp4'
        print(fileName)
        # if fileName != '':
        content = QUrl.fromLocalFile(fileName)
        print(content)              
        self.player.setMedia(QMediaContent(content))
        self.player.play()
        
    
class Gui(Node):
    def __init__(self, application:QApplication, window:GRR_Window) -> None:
        super().__init__("GUI")
        self.start_button_pub = self.create_publisher(Bool, "/gui/start", 10)
        self.drivetrain_enable = self.create_publisher(Bool, "/drivetrain/publish", 10)
        self.test_label = self.create_subscription(String, "/grr/state_name", self.display_test, 10)
        self.debug_label_sub = self.create_subscription(String, "/grr/debug", self.display_debug, 10)
        self.gui_loop_timer = self.create_timer(.1, self.gui_loop)
        self.gui_run_video = self.create_subscription(Bool, "/grr/promo_display", self.run_video, 10)
        self.app = application
        self.window = window
        self.previous_nodes = []
        self.drive_train_enable = True
        
        self.reset_command = JointState(name=['small_package_sweeper_joint', 'bridge_latch_joint', 'mechanism_package_joint', 'mechanism_thruster_joint', 'mechanism_lift_joint'], position=[0.0, 100.0, 50.0, 0.0, 100.0])

        self.servo_pub = self.create_publisher(JointState, "/grr/joint_command", 10)
        
        self.effort_pub = self.create_publisher(Float64MultiArray, '/effort_controller/commands', 10)
        self.raw_cmd = self.create_publisher(Twist, "/grr_cmake_controller/cmd_vel_unstamped", 10) 

        
        self.expected_nodes = ["/roboclaw_wrapper", "/controller_manager", "/isaac_hardware_interface", "/effort_controller", "/joint_state_broadcaster", "/grr_cmake_controller", "/grr/robot", "/grr/drivetrain", "/grr/gui", "/robot_state_publisher", "/odometry", "/State_Machine", "/grr/fake"]
        self.bind()
        
    def bind(self):
        self.window.start_button.clicked.connect(self.send_start)
        self.window.kill_motors_button.clicked.connect(self.kill_motor)
        self.window.reset_servos_button.clicked.connect(self.reset_servos)
        
    def reset_servos(self):
        self.servo_pub.publish(self.reset_command)
        
    def kill_motor(self, checked):
        self.window.kill_motors_button.setText(f"Drivetrain Goals: {'No ' if checked else 'Yes'}")
        self.drive_train_enable = not checked
        self.effort_pub.publish(Float64MultiArray(data=[0.0, 0.0]))
        self.raw_cmd.publish(Twist())
        
    def send_start(self, checked):
        self.window.start_button.setText(f"Ready to Start? {'Yes' if checked else 'No '}")
        self.start_button_pub.publish(Bool(data=checked))
        
    def run_video(self, msg:Bool):
        if msg.data:
            self.window.promoVid2()
    
    def display_test(self, msg:String):
        self.window.label.setText(msg.data)
    
    def display_debug(self, msg:String):
        self.window.debug_label.setText(msg.data)
        
    def display_nodes(self, nodes: list[str]):
        self.window.node_list.clear()
        maluable_nodes = nodes.copy()
        for node in self.expected_nodes:
            item = QListWidgetItem(node)
            if not node in maluable_nodes:
                item.setBackground(QColor(255, 0, 0, 100))
            else:
                maluable_nodes.pop(maluable_nodes.index(node))
            self.window.node_list.addItem(item)
        for node in maluable_nodes:
            item = QListWidgetItem(node)
            item.setBackground(QColor(255, 255, 0, 100))
            self.window.node_list.addItem(item)
        self.previous_nodes = nodes
        
    def gui_loop(self):
        self.app.processEvents(flags=QEventLoop.ProcessEventsFlag.AllEvents)     
        nodes = [((y + "/") if not y  == "/" else "/") + x for x,y in self.get_node_names_and_namespaces()]
        if not nodes == self.previous_nodes:
            self.display_nodes(nodes)
        self.drivetrain_enable.publish(Bool(data=self.drive_train_enable))
            

def main():
    # create application instance
    app = QApplication([])
    #instance of widget
    window = GRR_Window()
    rclpy.init()
    gui = Gui(app, window)
    rclpy.spin(gui)




if __name__ == "__main__":
    main()