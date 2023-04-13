import sys
import rospy
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton
from std_msgs.msg import String

class RosButton(QWidget):
    
    def __init__(self, msg):
        super().__init__()
        self.initUI()
        self.pub = rospy.Publisher('button_topic', String, queue_size=10)
        self.sub = rospy.Subscriber('button_topic', String, self.callback)
        self.message = String()
        self.message.data = msg
        
    def initUI(self):
        self.setWindowTitle('ROS Noetic Button Example')
        self.setGeometry(100, 100, 200, 150)
        self.button = QPushButton('Click me!', self)
        self.button.setToolTip('This is a ROS Noetic button!')
        self.button.clicked.connect(self.buttonClicked)
        self.show()
        
    def buttonClicked(self):        
        self.pub.publish(self.message)

    def callback(self, data):
        rospy.loginfo(data.data)

if __name__ == '__main__':
    rospy.init_node('button_node', anonymous=True)
    app = QApplication(sys.argv)
    ex = RosButton("Hello World!")
    sys.exit(app.exec_())