import sys
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

Z_MIN = 0
Z_MAX = 100
STEP = 1

class sliderdemo(QWidget):
    def __init__(self, parent = None):
        super(sliderdemo, self).__init__(parent)

        

        layout = QVBoxLayout()
        
        self.l1 = QLabel("linear_z")
        # self.l1.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.l1)
        
        # Sliders
        self.sl = QSlider(Qt.Vertical)
        self.sl.setMinimum(Z_MIN)
        self.sl.setMaximum(Z_MAX)
        self.sl.setValue(Z_MIN)
        self.sl.setTickPosition(QSlider.TicksBelow)
        self.sl.setTickInterval(STEP)        
        layout.addWidget(self.sl)

        self.sl.valueChanged.connect(self.valuechange)
        self.setLayout(layout)
        self.setWindowTitle("Drone controller")
        self.setGeometry(100, 100, 50, 400)

    def valuechange(self):
        v_z = self.sl.value()/Z_MAX
        print(v_z)

		
def main():
    app = QApplication(sys.argv)
    ex = sliderdemo()
    ex.show()
    sys.exit(app.exec_())
	
if __name__ == '__main__':
    main()