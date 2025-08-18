import sys
from PyQt6 import QtWidgets
from mainwindow import MainWindow  # Ana pencere sınıfını içe aktar

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())