import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5 import QtGui
from GUI import Ui_Dialog
from function import Function_UI
import serial, serial.tools.list_ports
import cv2

class MainWindow():
    def __init__(self):
        self.main_win = QMainWindow()
        self.uic = Ui_Dialog()
        self.uic.setupUi(self.main_win)

        self.serial = Function_UI()
        self.serialPort = serial.Serial()

        self.uic.cmbBAUD.addItems(self.serial.baudList)
        self.uic.cmbBAUD.setCurrentText('9600')
        self.update_ports()
        self.uic.btnConnect.clicked.connect(self.connect_serial)
        self.uic.btnSend.clicked.connect(self.send_data)
        self.uic.btnCamera.clicked.connect(self.on_cam)
        # self.uic.clear_Button.clicked.connect(self.clear)
        self.uic.btnUpdate.clicked.connect(self.update_ports)
        self.serial.data_available.connect(self.update_view)

    def on_cam(self):
        self.vid = cv2.VideoCapture(0)

        while True:
            _, self.frame = self.vid.read()
            self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
            self.update()
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    def update(self):
        self.setPhoto(self.frame)
    def setPhoto(self, image):
        print(image.shape)
        image = cv2.resize(image, (471,341))
        img = QtGui.QImage(image, image.shape[1], image.shape[0], image.strides[0],
                           QtGui.QImage.Format_RGB888)
        self.uic.viewCamera.setPixmap(QtGui.QPixmap.fromImage(img))

    def update_view(self, data):
        self.uic.lstView.append(data)

    def connect_serial(self):
        if (self.uic.btnConnect.isChecked()):
            port = self.uic.cmbCOM.currentText()
            baud = self.uic.cmbBAUD.currentText()
            self.serial.serialPort.port = port
            self.serial.serialPort.baudrate = baud
            self.serial.connect_serial()
            if (self.serial.serialPort.is_open):
                self.uic.btnConnect.setText("Disconnect")
        else:
            self.serial.disconnect_serial()
            self.uic.btnConnect.setText("CONNECT")

    def send_data(self):
        data_send = self.uic.txtSend.toPlainText()
        print(data_send)
        self.serial.send_data(data_send)

    def update_ports(self):
        self.serial.update_port()
        self.uic.cmbCOM.clear()
        self.uic.cmbCOM.addItems(self.serial.portList)

    def clear(self):
        self.uic.lstView.clear()

    def show(self):
        # command to run
        self.main_win.show()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_win = MainWindow()
    main_win.show()
    sys.exit(app.exec())