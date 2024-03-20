from PyQt5.QtCore import Qt, QEventLoop, QUrl
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import PyQt5
from PyQt5.QtWidgets import QSizePolicy, QApplication, QWidget, QPushButton, QVBoxLayout, QLabel, QMainWindow, QHBoxLayout, QListWidget, QListWidgetItem, QFileDialog
from PyQt5.QtMultimedia import QMediaContent, QMediaPlayer
from PyQt5.QtMultimediaWidgets import QVideoWidget
import sys
from std_msgs.msg import Bool, String 
from PyQt5.QtGui import QPixmap



class GRR_Window(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        
        self.setWindowTitle("GRR SECON APP")
        self.button = QPushButton("Ready to start? No ")
        self.button.setCheckable(True)


        self.label = QLabel()
        
        
        #media player instance None, QMediaPlayer.VideoSurface
        self.player = QMediaPlayer(None, QMediaPlayer.VideoSurface)

        horiz_layout = QHBoxLayout()
        
        
        self.videoWidget = QVideoWidget()
        self.videoWidget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        

        self.node_list = QListWidget()
        
        self.main_pane = QVBoxLayout()
        self.main_pane.addWidget(self.label)
        self.main_pane.addWidget(self.button)
        
        # self.main_pane.addWidget(self.videoWidget)

        self.loadVideoButton = QPushButton('Load and Play Video')
        self.main_pane.addWidget(self.loadVideoButton)
        

        horiz_layout.addLayout(self.main_pane)
        horiz_layout.addWidget(self.node_list)
        horiz_layout.addWidget(self.videoWidget)

        container = QWidget()
        container.setLayout(horiz_layout)
        self.player.setVideoOutput(self.videoWidget)
        #self.setCentralWidget(videoWidget)


        self.loadVideoButton.clicked.connect(self.promoVid2)

        # Connect the error signal to a slot
        #self.player.error.connect(self.handleError)

        self.setCentralWidget(container)
        self.showMaximized()
        self.show()

    def promoVid(self,main_pane):
        pixmap = QPixmap('Logo_MainGreenBorder.png')
        self.label.setPixmap(pixmap)
        self.resize(pixmap.width(), pixmap.height())
        self.main_pane.addWidget(self.label)
        
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
        self.test_label = self.create_subscription(String, "/gui/info", self.display_test, 10)
        self.gui_loop_timer = self.create_timer(.1, self.gui_loop)
        self.app = application
        self.window = window
        self.previous_nodes = []
        self.bind()
        
    def bind(self):
        self.window.button.clicked.connect(self.send_start)
        
    def send_start(self, checked):
        self.window.button.setText(f"Ready to Start? {'Yes' if checked else 'No '}")
        self.start_button_pub.publish(Bool(data=checked))
        
    def run_video(self, msg:Bool):
        if msg.data:
            self.window.promoVid2()
    
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
    # create application instance
    app = QApplication([])
    #instance of widget
    window = GRR_Window()
    rclpy.init()
    gui = Gui(app, window)
    rclpy.spin(gui)




if __name__ == "__main__":
    main()