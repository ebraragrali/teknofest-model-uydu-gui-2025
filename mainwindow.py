import sys 
import serial
import serial.tools.list_ports
import random
import re
import numpy as np
import cv2
from threading import Thread
from queue import Queue
from stl import mesh
from mpl_toolkits import mplot3d
from PyQt6 import QtWidgets, QtGui, QtCore
from PyQt6.QtCore import QTimer, QDateTime, pyqtSignal, QObject
from PyQt6.QtWidgets import QVBoxLayout, QHeaderView, QMessageBox
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from ui_mainwindow import Ui_MainWindow
from mpl_toolkits.mplot3d import art3d as mplot3d
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from PyQt6.QtGui import QPixmap, QImage


class CameraThread(QObject):
    frame_ready = pyqtSignal(QImage)
    recording_status = pyqtSignal(bool)
    
    def __init__(self):
        super().__init__()
        self.cap = None
        self.recording = False
        self.video_writer = None
        self.frame_queue = Queue(maxsize=32)
        self.running = True
        self.thread = None  # Thread nesnesi için referans
        
    def start_camera(self, url):
        try:
            # Windows için DSHOW backend'i kullan
            if url == "0":
                self.cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
            else:
                self.cap = cv2.VideoCapture(url, cv2.CAP_DSHOW)
            
            if not self.cap.isOpened():
                raise Exception("Kamera açılamadı")
                
            # Frame genişlik ve yüksekliğini kontrol et
            width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            print(f"Kamera çözünürlüğü: {width}x{height}")
            
            self.running = True
            self.thread = Thread(target=self.process_frames)
            self.thread.daemon = True
            self.thread.start()
            
        except Exception as e:
            print(f"Kamera başlatma hatası: {str(e)}")
            self.frame_ready.emit(QImage())
            
    def process_frames(self):
        while self.running and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                break
                
            # Convert to RGB and create QImage
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
            
            # Emit the frame
            self.frame_ready.emit(qt_image)
            
            # Save frame if recording
            if self.recording and self.video_writer is not None:
                self.video_writer.write(frame)
                
        self.cap.release()
        if self.video_writer is not None:
            self.video_writer.release()
            
    def start_recording(self, filename):
        if self.cap is None or not self.cap.isOpened():
            return
            
        # Get video properties
        fps = int(self.cap.get(cv2.CAP_PROP_FPS))
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        # Initialize video writer
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.video_writer = cv2.VideoWriter(filename, fourcc, fps, (width, height))
        self.recording = True
        self.recording_status.emit(True)
        
    def stop_recording(self):
        self.recording = False
        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None
        self.recording_status.emit(False)
        
    def stop(self):
        self.running = False
        if self.thread is not None and self.thread.is_alive():  # thread nesnesini kontrol et
            self.thread.join(timeout=1.0)
        if self.cap is not None:
            self.cap.release()
class MainWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

         # Motor ve Servo Kontrol Sistemleri
        self.setup_motor_control()
        self.setup_servo_control()

        self.load_map_image()  # Harita resmini yükle
        
        # Paket numarası için sayaç
        self.packet_counter = 1  # 1'den başlayacak

        # 3D Gyro Modeli 
        self.setup_3d_gyro()  

        # Kamera sistemi
        self.setup_camera_system()

        # GPS Widget'larını tanımla
        self.widget_gps = self.findChild(QtWidgets.QWidget, "widget_gps")
        self.lbl_latitude = self.findChild(QtWidgets.QLabel, "lbl_latitude")
        self.lbl_longitude = self.findChild(QtWidgets.QLabel, "lbl_longitude")
        self.lbl_altitude = self.findChild(QtWidgets.QLabel, "lbl_altitude")
        self.btn_gps_update = self.findChild(QtWidgets.QPushButton, "pushButton")

        # GPS Timer'ını başlat
        self.gps_timer = QTimer(self)
        self.gps_timer.timeout.connect(self.update_gps_data)
        self.gps_timer.start(1000)  # 1 saniyede bir güncelle
        
        # GPS Güncelleme butonu
        self.btn_gps_update.clicked.connect(self.manual_gps_update)

        self.previous_errors = set()  # Önceki hataları saklamak için

         # Görev kontrol butonlarını tanımla
        self.btn_start_mission = self.findChild(QtWidgets.QPushButton, "pushButton_2")  # Başlat butonu
        self.btn_stop_mission = self.findChild(QtWidgets.QPushButton, "btn_stop_mission")  # Durdur butonu
        
        # Buton fonksiyonlarını bağla
        self.btn_start_mission.clicked.connect(self.gorevi_baslat)
        self.btn_stop_mission.clicked.connect(self.gorevi_durdur)
        
        # Görev durumu değişkeni
        self.mission_active = False  # Görev başlangıçta durdurulmuş
        # Eksik olan değişkeni ekleyin
        self.mission_elapsed_time = 0  # Saniye cinsinden görev süresi
        self.mission_start_time = None  # Görevin başlama zamanı

        # Görev süresi için timer
        self.mission_time_timer = QTimer(self)
        self.mission_time_timer.timeout.connect(self.update_mission_time)
        self.mission_start_time = None
        self.mission_elapsed_time = 0
        
        # Görev süresi label'ını bul
        self.lbl_mission_time = self.findChild(QtWidgets.QLabel, "lbl_mission_time")
        self.lbl_mission_time.setText("00:00:00")  # Başlangıç değeri


        
        # Yeni düğmeleri tanımla
        self.btn_verileri_sil = self.findChild(QtWidgets.QPushButton, "btn_verileri_sil")
        self.btn_kalibre_et = self.findChild(QtWidgets.QPushButton, "btn_kalibre_et")
        self.btn_grafikleri_temizle = self.findChild(QtWidgets.QPushButton, "btn_grafikleri_temizle")
        
        # Düğme fonksiyonlarını bağla
        self.btn_verileri_sil.clicked.connect(self.verileri_temizle)
        self.btn_kalibre_et.clicked.connect(self.kalibrasyon_yap)
        self.btn_grafikleri_temizle.clicked.connect(self.grafikleri_sifirla)


        # Uydu statüsü widget'ını tanımla
        self.lbl_uydu_status = self.findChild(QtWidgets.QLabel, "lbl_uydu_status")  # Qt Designer'da eklenmiş olmalı
        self.lbl_uydu_status.setText("")  # Başlangıç metni

        self.status_mapping = {
            'Uçuşa Hazır': '0',
            'Yükselme': '1',
            'Model Uydu İniş': '2',
            'Ayrılma': '3',
            'Görev Yükü İniş': '4',
            'Kurtarma': '5'
        }

        self.setup_ui_style()
        self.setup_table()

        self.hata_widgets = [
            self.textedit_hata_kodu_14,
            self.textedit_hata_kodu_13,
            self.textedit_hata_kodu_15,
            self.textedit_hata_kodu_16,
            self.textedit_hata_kodu_17,
            self.textedit_hata_kodu_18
        ]
        
        # Timer ve filtre ayarları
        self.filter_timer = QTimer(self)
        self.filter_timer.timeout.connect(self.update_filter_timer)
        self.current_filter = None
        self.filter_remaining_time = 0
        self.btn_send_filter.clicked.connect(self.send_filter_command)

         # Seri port ve ayrılma durumu
        self.serial_port = None
        self.separation_status = False  
        self.pushButton_3.clicked.connect(self.send_separation_command)  
        

        self.try_connect_arduino()
        self.setup_all_graphs()
        self.init_timer()
        self.setWindowTitle("Model Uydu Arayüzü")
        self.counter = 0

    def setup_motor_control(self):
        """Şartname Gereksinim 35: Motor kontrol sistemini başlat"""
        self.btn_motor_kontrol = self.findChild(QtWidgets.QPushButton, "btn_motor_kontrol")
        self.btn_motor_kontrol.clicked.connect(self.open_motor_control)
        
        # Motor kontrol penceresi için değişkenler
        self.motor_control_window = None
        self.current_motor_speed = 0

    def setup_servo_control(self):
        """Şartname Gereksinim 35: Servo kontrol sistemini başlat"""
        self.btn_servo_kontrol = self.findChild(QtWidgets.QPushButton, "btn_servo_kontrol")
        self.btn_servo_kontrol.clicked.connect(self.open_servo_control)
        
        # Servo kontrol penceresi için değişkenler
        self.servo_control_window = None
        self.current_servo_angle = 90  # 90 derece başlangıç pozisyonu

    def open_motor_control(self):
        """Motor kontrol penceresini aç"""
        if self.motor_control_window is None:
            self.motor_control_window = QtWidgets.QWidget()
            self.motor_control_window.setWindowTitle("Motor Kontrol Paneli")
            self.motor_control_window.setFixedSize(300, 200)
            
            layout = QtWidgets.QVBoxLayout()
            
            # Hız kontrol slider
            self.slider_motor = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
            self.slider_motor.setRange(0, 100)
            self.slider_motor.setValue(self.current_motor_speed)
            self.slider_motor.valueChanged.connect(self.update_motor_speed)
            
            # Hız göstergesi
            self.lbl_motor_speed = QtWidgets.QLabel(f"Hız: {self.current_motor_speed}%")
            
            # Gönder butonu
            btn_send = QtWidgets.QPushButton("Motoru Kontrol Et")
            btn_send.clicked.connect(self.send_motor_command)
            
            # Durum etiketi
            self.lbl_motor_status = QtWidgets.QLabel("Durum: Hazır")
            
            layout.addWidget(QtWidgets.QLabel("Motor Hız Kontrolü:"))
            layout.addWidget(self.slider_motor)
            layout.addWidget(self.lbl_motor_speed)
            layout.addWidget(btn_send)
            layout.addWidget(self.lbl_motor_status)
            
            self.motor_control_window.setLayout(layout)
            self.motor_control_window.show()
        else:
            self.motor_control_window.show()

    def open_servo_control(self):
        """Servo kontrol penceresini aç"""
        if self.servo_control_window is None:
            self.servo_control_window = QtWidgets.QWidget()
            self.servo_control_window.setWindowTitle("Servo Kontrol Paneli")
            self.servo_control_window.setFixedSize(300, 200)
            
            layout = QtWidgets.QVBoxLayout()
            
            # Açı kontrol slider
            self.slider_servo = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
            self.slider_servo.setRange(0, 180)  # 0-180 derece aralığı
            self.slider_servo.setValue(self.current_servo_angle)
            self.slider_servo.valueChanged.connect(self.update_servo_angle)
            
            # Açı göstergesi
            self.lbl_servo_angle = QtWidgets.QLabel(f"Açı: {self.current_servo_angle}°")
            
            # Gönder butonu
            btn_send = QtWidgets.QPushButton("Servo Pozisyonunu Ayarla")
            btn_send.clicked.connect(self.send_servo_command)
            
            # Durum etiketi
            self.lbl_servo_status = QtWidgets.QLabel("Durum: Hazır")
            
            layout.addWidget(QtWidgets.QLabel("Servo Açı Kontrolü:"))
            layout.addWidget(self.slider_servo)
            layout.addWidget(self.lbl_servo_angle)
            layout.addWidget(btn_send)
            layout.addWidget(self.lbl_servo_status)
            
            self.servo_control_window.setLayout(layout)
            self.servo_control_window.show()
        else:
            self.servo_control_window.show()

    def update_motor_speed(self, speed):
        """Motor hızını güncelle (UI)"""
        self.current_motor_speed = speed
        self.lbl_motor_speed.setText(f"Hız: {speed}%")

    def update_servo_angle(self, angle):
        """Servo açısını güncelle (UI)"""
        self.current_servo_angle = angle
        self.lbl_servo_angle.setText(f"Açı: {angle}°")

    def send_motor_command(self):
        """Şartname Gereksinim 35: Motor kontrol komutunu gönder"""
        if self.serial_port and self.serial_port.is_open:
            try:
                command = f"MOTOR:{self.current_motor_speed}\n"
                self.serial_port.write(command.encode('utf-8'))
                self.lbl_motor_status.setText("Durum: Komut Gönderildi")
                QMessageBox.information(self, "Başarılı", 
                    f"Motor hızı {self.current_motor_speed}% olarak ayarlandı!")
            except Exception as e:
                self.lbl_motor_status.setText("Durum: Hata!")
                QMessageBox.critical(self, "Hata", 
                    f"Motor komutu gönderilemedi:\n{str(e)}")
        else:
            self.lbl_motor_status.setText("Durum: Bağlantı Yok!")
            QMessageBox.warning(self, "Uyarı", 
                "Seri bağlantı aktif değil! Simülasyon modunda çalışılıyor.")
            # Simülasyon modu
            QMessageBox.information(self, "Simülasyon", 
                f"Motor hızı {self.current_motor_speed}% olarak ayarlandı (Simülasyon)")

    def send_servo_command(self):
        """Şartname Gereksinim 35: Servo kontrol komutunu gönder"""
        if self.serial_port and self.serial_port.is_open:
            try:
                command = f"SERVO:{self.current_servo_angle}\n"
                self.serial_port.write(command.encode('utf-8'))
                self.lbl_servo_status.setText("Durum: Komut Gönderildi")
                QMessageBox.information(self, "Başarılı", 
                    f"Servo açısı {self.current_servo_angle}° olarak ayarlandı!")
            except Exception as e:
                self.lbl_servo_status.setText("Durum: Hata!")
                QMessageBox.critical(self, "Hata", 
                    f"Servo komutu gönderilemedi:\n{str(e)}")
        else:
            self.lbl_servo_status.setText("Durum: Bağlantı Yok!")
            QMessageBox.warning(self, "Uyarı", 
                "Seri bağlantı aktif değil! Simülasyon modunda çalışılıyor.")
            # Simülasyon modu
            QMessageBox.information(self, "Simülasyon", 
                f"Servo açısı {self.current_servo_angle}° olarak ayarlandı (Simülasyon)")

    def handle_serial_response(self, response):
        """Seri porttan gelen yanıtları işle (motor/servo durumları)"""
        if response.startswith("MOTOR:"):
            # Motor yanıtı işleme
            status = response[6:].strip()
            if status == "ACK":
                self.lbl_motor_status.setText("Durum: Onaylandı")
            elif status == "ERR":
                self.lbl_motor_status.setText("Durum: Hata!")
                
        elif response.startswith("SERVO:"):
            # Servo yanıtı işleme
            status = response[6:].strip()
            if status == "ACK":
                self.lbl_servo_status.setText("Durum: Onaylandı")
            elif status == "ERR":
                self.lbl_servo_status.setText("Durum: Hata!")

    def setup_camera_system(self):
        """Kamera sistemini başlat (Şartname Gereksinim 17-18-29)"""
        # Kamera widget'larını bul
        self.lbl_kamera = self.findChild(QtWidgets.QLabel, "lbl_kamera")
        self.btn_kameraAc = self.findChild(QtWidgets.QPushButton, "btn_kameraAc")
        self.btn_kameraKapat = self.findChild(QtWidgets.QPushButton, "btn_kameraKapat")
        self.lineEdit_kamera_url = self.findChild(QtWidgets.QLineEdit, "lineEdit")
        
        # Kamera URL'si için placeholder metni
        self.lineEdit_kamera_url.setPlaceholderText("Kamera URL'sini girin veya 0 yazın (varsayılan kamera)")
        
        # Kamera thread'i oluştur
        self.camera_thread = CameraThread()
        self.camera_thread.frame_ready.connect(self.update_camera_frame)
        self.camera_thread.recording_status.connect(self.update_recording_status)
        
        # Buton bağlantıları
        self.btn_kameraAc.clicked.connect(self.start_camera)
        self.btn_kameraKapat.clicked.connect(self.stop_camera)
        
        # Kayıt durumu
        self.recording = False
        self.recording_file = "kamera_kaydi.avi"
        
        # Kamera panelini başlangıçta devre dışı bırak
        self.btn_kameraKapat.setEnabled(False)
        
    def start_camera(self):
        """Kamerayı başlat (Şartname Gereksinim 18)"""
        url = self.lineEdit_kamera_url.text().strip()
        if not url:
            url = "0"  # Varsayılan kamera
        
        try:
            # Kamera thread'ini başlat
            self.camera_thread.start_camera(url)
            
            # Görev başladıysa kaydı başlat (Şartname Gereksinim 17)
            if self.mission_active:
                self.start_recording()
            
            # UI güncelleme
            self.btn_kameraAc.setEnabled(False)
            self.btn_kameraKapat.setEnabled(True)
            self.lineEdit_kamera_url.setEnabled(False)
            
            QMessageBox.information(self, "Kamera", "Kamera başarıyla başlatıldı!")
            
        except Exception as e:
            QMessageBox.critical(self, "Kamera Hatası", f"Kamera başlatılamadı:\n{str(e)}")
            
    def stop_camera(self):
        """Kamerayı durdur"""
        # Kayıt varsa durdur
        if self.recording:
            self.stop_recording()
            
        # Kamera thread'ini durdur
        self.camera_thread.stop()
        
        # UI güncelleme
        self.btn_kameraAc.setEnabled(True)
        self.btn_kameraKapat.setEnabled(False)
        self.lineEdit_kamera_url.setEnabled(True)
        
        # Kamera görüntüsünü temizle
        self.lbl_kamera.clear()
        self.lbl_kamera.setText("Kamera Paneli")
        
        QMessageBox.information(self, "Kamera", "Kamera durduruldu!")
        
    def start_recording(self):
        """Video kaydını başlat (Şartname Gereksinim 17)"""
        if not self.recording:
            timestamp = QDateTime.currentDateTime().toString("yyyyMMdd_hhmmss")
            self.recording_file = f"kamera_kaydi_{timestamp}.avi"
            self.camera_thread.start_recording(self.recording_file)
            self.recording = True
            
    def stop_recording(self):
        """Video kaydını durdur"""
        if self.recording:
            self.camera_thread.stop_recording()
            self.recording = False
            QMessageBox.information(self, "Kayıt", f"Video kaydı tamamlandı:\n{self.recording_file}")
            
    def update_camera_frame(self, image):
        """Kamera görüntüsünü güncelle"""
        if not image.isNull():
            pixmap = QPixmap.fromImage(image)
            self.lbl_kamera.setPixmap(pixmap.scaled(
                self.lbl_kamera.size(), 
                QtCore.Qt.AspectRatioMode.KeepAspectRatio,
                QtCore.Qt.TransformationMode.SmoothTransformation
            ))
            
    def update_recording_status(self, recording):
        """Kayıt durumunu güncelle"""
        self.recording = recording
        if recording:
            self.btn_kameraKapat.setText("Kaydı Durdur ve Kapat")
        else:
            self.btn_kameraKapat.setText("Kamerayı Kapat")

    def load_map_image(self):
        """Harita görselini yükle"""
        self.lbl_harita = self.findChild(QtWidgets.QLabel, "lbl_harita")
        pixmap = QPixmap("image.png")  # image.png proje dizininde olmalı
         
        if not pixmap.isNull():
            self.lbl_harita.setPixmap(pixmap)
            self.lbl_harita.setScaledContents(True)  # Resmi label boyutuna sığdır
        else:
            print("Hata: image.png bulunamadı!")

    def load_mission_time(self):
        """Kayıtlı görev zamanını dosyadan yükle"""
        try:
            with open('mission_time.dat', 'r') as f:
                self.mission_elapsed_time = float(f.read())
                print(f"Yüklenen görev zamanı: {self.mission_elapsed_time}s")
        except (FileNotFoundError, ValueError):
            self.mission_elapsed_time = 0
            print("Görev zamanı dosyası bulunamadı, sıfırdan başlatılıyor")

    def save_mission_time(self):
        """Görev zamanını dosyaya kaydet"""
        try:
            with open('mission_time.dat', 'w') as f:
                f.write(str(self.mission_elapsed_time))
        except Exception as e:
            print(f"Görev zamanı kaydedilemedi: {e}")

    def format_mission_time(self, seconds):
        """Saniyeyi saat:dakika:saniye formatına çevir"""
        hours = int(seconds // 3600)
        minutes = int((seconds % 3600) // 60)
        secs = int(seconds % 60)
        return f"{hours:02d}:{minutes:02d}:{secs:02d}"
    

    def update_gps_data(self):
        """Şartname Gereksinim 12-13-24: GPS verilerini otomatik güncelle"""
        if self.serial_port and self.serial_port.is_open:
            try:
                # Gerçek veriyi oku (Örnek format: GPS:37.000,28.000,500.5)
                line = self.serial_port.readline().decode().strip()
                if line.startswith("GPS:"):
                    lat, lon, alt = line.split(":")[1].split(",")
                    self.lbl_latitude.setText(f"Latitude: {lat}")
                    self.lbl_longitude.setText(f"Longitude: {lon}")
                    self.lbl_altitude.setText(f"Altitude: {alt} m")
            except:
                self.simulate_gps_data()
        else:
            self.simulate_gps_data()

    def manual_gps_update(self):
        """Şartname Gereksinim 24: Manuel GPS güncelleme"""
        QMessageBox.information(self, "GPS Güncelleme", "GPS verileri yenileniyor...")
        self.update_gps_data()

    def simulate_gps_data(self):
        """Şartname Gereksinim 12: Simüle GPS verileri"""
        lat = 37.0 + random.uniform(-0.01, 0.01)
        lon = 28.0 + random.uniform(-0.01, 0.01)
        alt = random.uniform(0, 500)
        
        self.lbl_latitude.setText(f"Latitude: {lat:.6f}°")
        self.lbl_longitude.setText(f"Longitude: {lon:.6f}°")
        self.lbl_altitude.setText(f"Altitude: {alt:.2f} m")

    def gorevi_baslat(self):
        """Görev başlatma ve zaman sayacını başlat"""
        if not self.mission_active:
            # Görev başlangıç zamanını ayarla
            self.mission_start_time = QDateTime.currentDateTime()
            self.mission_active = True
            
            # Timer'ı başlat
            self.mission_time_timer.start(1000)  # 1 saniyede bir güncelle
            
            # Arduino'ya başlatma komutu gönder
            if self.serial_port and self.serial_port.is_open:
                try:
                    self.serial_port.write("MISSION_START\n".encode('utf-8'))
                except Exception as e:
                    print(f"Başlatma komutu gönderilemedi: {e}")
            
            # Kamera açıksa kaydı başlat (Şartname Gereksinim 17)
            if self.btn_kameraKapat.isEnabled():
                self.start_recording()
            
            self.btn_start_mission.setStyleSheet("background-color: green; color: white;")
            QMessageBox.information(self, "Görev Durumu", "Görev başlatıldı!")
        else:
            QMessageBox.warning(self, "Uyarı", "Görev zaten aktif!")

    def gorevi_durdur(self):
        """Görev durdurma ve zaman sayacını durdur"""
        if self.mission_active:
            # Geçen süreyi hesapla ve kaydet
            current_elapsed = self.mission_start_time.secsTo(QDateTime.currentDateTime())
            self.mission_elapsed_time += current_elapsed
            self.mission_active = False
            
            # Timer'ı durdur
            self.mission_time_timer.stop()
            
            # Kamera kaydını durdur (Şartname Gereksinim 17)
            if self.recording:
                self.stop_recording()
                
            self.btn_start_mission.setStyleSheet("background-color: #3c3f41; color: white;")
            QMessageBox.information(self, "Görev Durumu", "Görev durduruldu!")
        else:
            QMessageBox.warning(self, "Uyarı", "Görev zaten durdurulmuş!")

    def update_mission_time(self):
        """Görev süresini güncelle ve label'a yaz"""
        if self.mission_active and self.mission_start_time:
            current_elapsed = self.mission_start_time.secsTo(QDateTime.currentDateTime())
            total_seconds = self.mission_elapsed_time + current_elapsed
        else:
            total_seconds = self.mission_elapsed_time
        
        # Saniyeyi saat:dakika:saniye formatına çevir
        hours = int(total_seconds // 3600)
        minutes = int((total_seconds % 3600) // 60)
        seconds = int(total_seconds % 60)
        
        # Label'ı güncelle
        self.lbl_mission_time.setText(f"{hours:02d}:{minutes:02d}:{seconds:02d}")

           

    def verileri_temizle(self):
        """Şartname Gereksinim 28: Telemetri verilerinin temizlenmesi"""
        # Tabloyu temizle
        self.table_telemetri.setRowCount(0)
        
        # Grafik verilerini sıfırla
        for i in range(len(self.data_times)):
            self.data_times[i].clear()
            self.data_values[i].clear()
            self.update_graph(i, 0, "", 'b')  # Grafikleri yeniden çiz

        QMessageBox.information(self, "Başarılı", "Tüm veriler ve grafikler temizlendi!")

    def kalibrasyon_yap(self):
        """Şartname Gereksinim 12: Sensör kalibrasyonu"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.write("CALIBRATE\n".encode())
            QMessageBox.information(self, "Kalibrasyon", "Sensörler kalibre ediliyor...")
        else:
            # Simülasyon modunda kalibrasyon
            for i in range(6):
                self.data_values[i] = [0] * len(self.data_values[i])
                self.update_graph(i, 0, "", 'b')
            QMessageBox.information(self, "Kalibrasyon", "Sensörler sıfırlandı (Simülasyon)")

    def grafikleri_sifirla(self):
        """Şartname Gereksinim 28: Grafiklerin gerçek zamanlı sıfırlanması"""
        for i in range(len(self.data_times)):
            self.data_times[i].clear()
            self.data_values[i].clear()
            self.update_graph(i, 0, "", 'b')
        
        QMessageBox.information(self, "Grafikler", "Grafik verileri sıfırlandı!")

    def setup_ui_style(self):
        self.setStyleSheet("""
            QMainWindow { background-color: #2b2b2b; }
            QLabel { color: white; font-size: 12px; }
            QPushButton { 
                background-color: #3c3f41; 
                color: white; 
                border: 1px solid #555; 
                padding: 5px;
                border-radius: 3px;
            }
            QPushButton:hover { background-color: #4d4f51; }
            QPushButton:disabled { background-color: #2d2f30; color: #555; }
            QTableWidget { 
                background-color: #323232; 
                color: white; 
                gridline-color: #555;
            }
            QHeaderView::section { 
                background-color: #3c3f41; 
                color: white; 
                padding: 5px;
            }
            QGroupBox {
                border: 1px solid #555;
                border-radius: 5px;
                margin-top: 10px;
                font-weight: bold;
                color: white;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 3px;
            }
            QSlider::groove:horizontal {
            height: 8px;
            background: #3a3f44;
            border-radius: 4px;
            }
            QSlider::handle:horizontal {
                width: 18px;
                margin: -5px 0;
                background: #00ffff;
                border-radius: 9px;
            }
            QSlider::sub-page:horizontal {
                background: #00aaaa;
                border-radius: 4px;
            }
            /* Motor/Servo kontrol pencereleri */
            QWidget#motor_control_window, QWidget#servo_control_window {
                background-color: #323232;
            }
        
        """)

    def send_separation_command(self):
        if not self.separation_status:
            reply = QMessageBox.question(
                self, 'Onay',
                'Ayrılma komutunu göndermek istediğinize emin misiniz?',
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No
            )
            if reply == QMessageBox.StandardButton.Yes:
                if self.serial_port and self.serial_port.is_open:
                    try:
                        self.serial_port.write("SEPARATE\n".encode('utf-8'))
                        self.lbl_ayrilma_durumu.setText("Ayrılma Durumu: Komut Gönderildi")
                        self.lbl_ayrilma_durumu.setStyleSheet("color: orange; font-weight: bold;")
                        print("Ayrılma komutu gönderildi.")
                    except Exception as e:
                        QMessageBox.critical(self, "Hata", f"Ayrılma komutu gönderilemedi:\n{str(e)}")
                else:
                    QMessageBox.warning(self, "Bağlantı Yok", "Seri bağlantı aktif değil!")
        else:
            QMessageBox.information(self, "Bilgi", "Ayrılma zaten gerçekleşmiş.")

    def update_data(self):
        """Telemetri verilerini güncelle ve görev zamanını ekle"""
        # Önceki değerleri saklamak için sınıf değişkeni kontrolü
        if not hasattr(self, 'prev_values'):
            self.prev_values = {
                'y1': 0.0,          # Yükseklik 1 (m)
                'y2': 0.0,          # Yükseklik 2 (m)
                'press1': 1013.25,  # Basınç 1 (hPa)
                'press2': 1013.25,  # Basınç 2 (hPa)
                'vel1': 0.0,        # İniş Hızı 1 (m/s)
                'vel2': 0.0,        # İniş Hızı 2 (m/s)
                'temp': 25.0,       # Sıcaklık (°C)
                'bat': 4.2,         # Pil Gerilimi (V)
                'status_counter': 0 # Durum geçiş sayacı
            }

        # Görev zamanını güncelle
        if self.mission_active and self.mission_start_time:
            current_elapsed = self.mission_start_time.secsTo(QDateTime.currentDateTime())
            total_time = self.mission_elapsed_time + current_elapsed
            formatted_time = self.format_mission_time(total_time)
        else:
            formatted_time = self.format_mission_time(self.mission_elapsed_time)

        # Motor ve servo durumlarını kontrol et
        if self.serial_port and self.serial_port.in_waiting:
            try:
                response = self.serial_port.readline().decode('utf-8').strip()
                self.handle_serial_response(response)
            except Exception as e:
                print(f"Seri okuma hatası: {e}")

        if self.serial_port and self.serial_port.is_open:
            # Gerçek veri okuma kodu
            try:
                line = self.serial_port.readline().decode('utf-8').strip()
                parts = dict(item.split(":") for item in line.split(";") if ":" in item)
                
                # Paket numarasını ve görev zamanını ekle
                parts['PAKET'] = str(self.packet_counter)
                parts['GOREV_ZAMANI'] = formatted_time
                self.packet_counter += 1

                # Uydu statüsünü güncelle
                raw_status = parts.get('STATU', 'Bekleme')
                status_code = self.status_mapping.get(raw_status, '0')
                self.lbl_uydu_status.setText(f"Statü: {status_code} ({raw_status})")

                # Hata kodunu hesapla
                error_code, colors = self.calculate_error_code(parts)
                self.update_error_display(error_code, colors)
                parts['HATA'] = error_code

                # Grafikleri güncelle
                self.update_graph(0, float(parts.get('Y1', 0)), "Yükseklik 1 (m)", '#1f77b4')
                self.update_graph(1, float(parts.get('Y2', 0)), "Yükseklik 2 (m)", '#ff7f0e')
                self.update_graph(2, float(parts.get('PRESS1', 0)), "Basınç 1 (hPa)", '#2ca02c')
                self.update_graph(3, float(parts.get('PRESS2', 0)), "Basınç 2 (hPa)", '#d62728')
                self.update_graph(4, float(parts.get('VEL', 0)), "İniş Hızı (m/s)", '#9467bd')
                self.update_graph(5, float(parts.get('TEMP', 0)), "Sıcaklık (°C)", '#8c564b')
                self.update_graph(6, float(parts.get('IOT1', 0)), "IoT S1 Sıcaklık (°C)", '#e377c2')
                self.update_graph(7, float(parts.get('IOT2', 0)), "IoT S2 Sıcaklık (°C)", '#7f7f7f')

                # Gyro simülasyonu
                pitch = float(parts.get('PITCH', 0))
                roll = float(parts.get('ROLL', 0))
                yaw = float(parts.get('YAW', 0))
                self.plot_gyro(pitch=pitch, roll=roll, yaw=yaw)

                # Tabloya veri ekle
                if parts:
                    row_position = self.table_telemetri.rowCount()
                    self.table_telemetri.insertRow(row_position)
                    
                    # İrtifa farkı hesaplama
                    y1 = max(0, float(parts.get('Y1', 0)))
                    y2 = max(0, float(parts.get('Y2', 0)))
                    altitude_diff = round(abs(y1 - y2), 2)
                    
                    # Zaman formatı
                    current_time = QDateTime.currentDateTime()
                    formatted_time = current_time.toString("dd/MM/yyyy, hh:mm:ss")
                    
                    # Ayrılma durumu
                    separation_status = "Evet" if raw_status in ['Ayrılma', 'Görev Yükü İniş', 'Kurtarma'] else "Hayır"
                    
                    columns = [
                        parts.get('PAKET', '0'),
                        parts.get('STATU', 'Bekleme'),
                        parts.get('HATA', '000000'),
                        formatted_time,
                        parts.get('PRESS1', '0'),
                        parts.get('PRESS2', '0'),
                        str(y1),
                        str(y2),
                        str(altitude_diff),
                        parts.get('VEL', '0'),
                        parts.get('TEMP', '0'),
                        parts.get('BAT', '0'),
                        parts.get('LAT', '0'),
                        parts.get('LON', '0'),
                        parts.get('GPS_ALT', '0'),
                        parts.get('PITCH', '0'),
                        parts.get('ROLL', '0'),
                        parts.get('YAW', '0'),
                        parts.get('RHRH', '-'),
                        parts.get('IOT1', '-'),
                        parts.get('IOT2', '-'),
                        parts.get('TAKIM', '-'),
                        separation_status
                    ]
                    
                    for col, value in enumerate(columns):
                        item = QtWidgets.QTableWidgetItem(str(value))
                        self.table_telemetri.setItem(row_position, col, item)

            except Exception as e:
                print(f"Veri okuma hatası: {e}")
                parts = {}
        else:
            # Simülasyon verisi üret
            current_status = self.get_simulated_status()
            vel1, vel2 = self.get_simulated_velocities(current_status)
            press1, press2 = self.get_simulated_pressures(current_status)
            y1, y2 = self.get_simulated_altitudes(current_status)
            
            # Fiziksel model parametreleri
            delta_time = 1.0  # 1 saniyelik zaman adımı
            
            # Yükseklik simülasyonu (Duruma göre dinamik davranış)
            if current_status == 'Yükselme':
                self.prev_values['y1'] += random.uniform(8.0, 12.0)
            elif current_status == 'Model Uydu İniş':
                self.prev_values['y1'] -= random.uniform(12.0, 14.0)
                self.prev_values['y1'] = max(0, self.prev_values['y1'])
            elif current_status == 'Görev Yükü İniş':
                self.prev_values['y1'] -= random.uniform(6.0, 8.0)
                self.prev_values['y1'] = max(0, self.prev_values['y1'])
            
            # Yükseklik 2 (Taşıyıcı) için ilişkili veri
            self.prev_values['y2'] = max(0, self.prev_values['y1'] * 0.98 + random.uniform(-0.5, 0.5))

            # Basınç simülasyonu
            self.prev_values['press1'] = 1013.25 * (1 - (self.prev_values['y1']/44330)) ** 5.255
            self.prev_values['press2'] = self.prev_values['press1'] * 0.99 + random.uniform(-2, 2)

            # Hız simülasyonu
            current_vel = (self.prev_values['y1'] - self.prev_values.get('last_y1', 0)) / delta_time
            self.prev_values['vel1'] = 0.8*self.prev_values['vel1'] + 0.2*current_vel
            self.prev_values['vel2'] = self.prev_values['vel1'] * 0.7 + random.uniform(-0.1, 0.1)

            # Diğer parametreler
            self.prev_values['temp'] += random.uniform(-0.1, 0.1)
            self.prev_values['temp'] = max(-10, min(50, self.prev_values['temp']))
            self.prev_values['bat'] -= 0.00027
            self.prev_values['bat'] = max(3.3, self.prev_values['bat'])
            self.prev_values['status_counter'] += 1

            # Veri paketini oluştur
            parts = {
                'PAKET': str(self.packet_counter),
                'STATU': current_status,
                'VEL': round(self.prev_values['vel1'], 2),
                'VEL2': round(self.prev_values['vel2'], 2),
                'PRESS1': round(self.prev_values['press1'], 2),
                'PRESS2': round(self.prev_values['press2'], 2),
                'Y1': round(max(0, self.prev_values['y1']), 2),
                'Y2': round(max(0, self.prev_values['y2']), 2),
                'TEMP': round(self.prev_values['temp'], 1),
                'BAT': round(self.prev_values['bat'], 3),
                'LAT': f"37.{random.randint(10000, 99999)}",
                'LON': f"28.{random.randint(10000, 99999)}",
                'GPS_ALT': round(self.prev_values['y1'] + random.uniform(-0.5, 0.5), 2),
                'PITCH': round(random.uniform(-5, 5), 1),
                'ROLL': round(random.uniform(-5, 5), 1),
                'YAW': round(random.uniform(0, 360), 1),
                'RHRH': random.choice(['6G9R', '7Y8P', '8B7P', '9Y6B']),
                'IOT1': round(25 + random.uniform(-0.5, 0.5), 1),
                'IOT2': round(25 + random.uniform(-0.5, 0.5), 1),
                'TAKIM': '600898',
                'TIME': QDateTime.currentDateTime().toString("dd/MM/yyyy, hh:mm:ss")
            }
            self.packet_counter += 1
            self.prev_values['last_y1'] = self.prev_values['y1']

        # Hata kodunu hesapla ve göster
        error_code, colors = self.calculate_error_code(parts)
        self.update_error_display(error_code, colors)
        parts['HATA'] = error_code

        # Grafikleri güncelle
        self.update_graph(0, float(parts.get('Y1', 0)), "Yükseklik 1 (m)", '#1f77b4')
        self.update_graph(1, float(parts.get('Y2', 0)), "Yükseklik 2 (m)", '#ff7f0e')
        self.update_graph(2, float(parts.get('PRESS1', 0)), "Basınç 1 (hPa)", '#2ca02c')
        self.update_graph(3, float(parts.get('PRESS2', 0)), "Basınç 2 (hPa)", '#d62728')
        self.update_graph(4, float(parts.get('VEL', 0)), "İniş Hızı (m/s)", '#9467bd')
        self.update_graph(5, float(parts.get('TEMP', 0)), "Sıcaklık (°C)", '#8c564b')
        self.update_graph(6, float(parts.get('IOT1', 0)), "IoT S1 Sıcaklık (°C)", '#e377c2')
        self.update_graph(7, float(parts.get('IOT2', 0)), "IoT S2 Sıcaklık (°C)", '#7f7f7f')

        # Gyro simülasyonu
        if parts.get('STATU') == 'Yükselme':
            pitch = random.uniform(-2, 2)
            roll = random.uniform(-2, 2)
        elif parts.get('STATU') == 'Model Uydu İniş':
            pitch = random.uniform(-5, 5)
            roll = random.uniform(-5, 5)
        else:
            pitch = 0
            roll = 0
        self.plot_gyro(pitch=pitch, roll=roll, yaw=random.uniform(0, 360))

        # Tabloya veri ekle
        if parts:
            row_position = self.table_telemetri.rowCount()
            self.table_telemetri.insertRow(row_position)
            
            # İrtifa farkı hesaplama
            try:
                y1 = max(0, float(parts.get('Y1', 0)))
                y2 = max(0, float(parts.get('Y2', 0)))
                altitude_diff = round(abs(y1 - y2), 2)
            except:
                altitude_diff = 0.0
                
            columns = [
                parts.get('PAKET', '0'),
                parts.get('STATU', 'Bekleme'),
                parts.get('HATA', '000000'),
                parts.get('TIME', QDateTime.currentDateTime().toString("dd/MM/yyyy, hh:mm:ss")),
                parts.get('PRESS1', '0'),
                parts.get('PRESS2', '0'),
                parts.get('Y1', '0'),
                parts.get('Y2', '0'),
                str(altitude_diff),
                parts.get('VEL', '0'),
                parts.get('TEMP', '0'),
                parts.get('BAT', '0'),
                parts.get('LAT', '0'),
                parts.get('LON', '0'),
                parts.get('GPS_ALT', '0'),
                parts.get('PITCH', '0'),
                parts.get('ROLL', '0'),
                parts.get('YAW', '0'),
                parts.get('RHRH', '-'),
                parts.get('IOT1', '-'),
                parts.get('IOT2', '-'),
                parts.get('TAKIM', '-'),
                "Evet" if parts.get('STATU') in ['Ayrılma', 'Görev Yükü İniş', 'Kurtarma'] else "Hayır"
            ]
            
            for col, value in enumerate(columns):
                item = QtWidgets.QTableWidgetItem(str(value))
                self.table_telemetri.setItem(row_position, col, item)
    def setup_table(self):
        self.table_telemetri.setColumnCount(22)
        self.table_telemetri.setHorizontalHeaderLabels([
            "Paket\nNumarası", "Uydu\nStatüsü", "Hata\nKodu", "Gönderme\nSaati",
            "Basınç\n1 (hPa)", "Basınç\n2 (hPa)", "Yükseklik\n1 (m)", "Yükseklik\n2 (m)",
            "İrtifa\nFarkı (m)", "İniş\nHızı (m/s)", "Sıcaklık (°C)", "Pil\nGerilimi (V)",
            "GPS1\nLatitude", "GPS1\nLongitude", "GPS1\nAltitude (m)",
            "Pitch (°)", "Roll (°)", "Yaw (°)", "RHRH", "IoT S1\nData (°C)", "IoT S2\nData (°C)",
            "Takım\nNo"
        ])
        self.table_telemetri.horizontalHeader().setStretchLastSection(True)
        self.table_telemetri.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.table_telemetri.verticalHeader().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        self.table_telemetri.setAlternatingRowColors(True)

    def init_timer(self):
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_data)
        self.timer.start(1000)

    def try_connect_arduino(self):
        try:
            ports = serial.tools.list_ports.comports()
            for port in ports:
                if "Arduino" in port.description or "USB" in port.description:
                    self.serial_port = serial.Serial(port.device, baudrate=9600, timeout=1)
                    print(f"Arduino'ya bağlandı: {port.device}")
                     # Motor ve servo kontrol için başlangıç komutları
                    self.serial_port.write("INIT_MOTOR\n".encode('utf-8'))
                    self.serial_port.write("INIT_SERVO\n".encode('utf-8'))
                    return
            print("Arduino bulunamadı, simülasyon başlatılıyor.")
        except Exception as e:
            print(f"Arduino bağlantı hatası: {e}")
            # Simülasyon modunda devam et
            self.serial_port = None

    def setup_all_graphs(self):
        """Şartname Gereksinim 28: Grafiklerin doğru mühendislik birimleriyle oluşturulması"""
        self.graphs = []
        self.data_times = []
        self.data_values = []
        
        # Şartname 2.4'e göre grafik konfigürasyonları
        graph_configs = [
            {"title": "Yükseklik 1", "unit": "m", "color": "#1f77b4", "min": 0, "max": 500, "normal_range": (0, 400)},
            {"title": "Yükseklik 2", "unit": "m", "color": "#ff7f0e", "min": 0, "max": 500, "normal_range": (0, 400)},
            {"title": "Basınç 1", "unit": "hPa", "color": "#2ca02c", "min": 800, "max": 1100, "normal_range": (900, 1100)},
            {"title": "Basınç 2", "unit": "hPa", "color": "#d62728", "min": 800, "max": 1100, "normal_range": (900, 1100)},
            {"title": "İniş Hızı", "unit": "m/s", "color": "#9467bd", "min": 0, "max": 20, "normal_range": (6, 14)},
            {"title": "Sıcaklık", "unit": "°C", "color": "#8c564b", "min": -10, "max": 50, "normal_range": (10, 30)},
            {"title": "IoT S1 Sıcaklık", "unit": "°C", "color": "#e377c2", "min": -10, "max": 50, "normal_range": (10, 30)},
            {"title": "IoT S2 Sıcaklık", "unit": "°C", "color": "#7f7f7f", "min": -10, "max": 50, "normal_range": (10, 30)}
        ]
        
        for i, config in enumerate(graph_configs):
            figure = Figure(figsize=(5, 3), facecolor='#1e1e1e')
            canvas = FigureCanvas(figure)
            ax = figure.add_subplot(111)
            
            # Grafik görünümünü bul
            graphics_view = getattr(self, f'graphicsView_{i + 1}', None)
            if graphics_view is not None:
                graphics_view.setFixedSize(250, 180)
                layout = QVBoxLayout(graphics_view)
                layout.setContentsMargins(0, 0, 0, 0)
                layout.addWidget(canvas)
                
                # Eksen ayarları
                ax.set_xlabel('Zaman (s)', color='white', fontsize=8)
                ax.set_ylabel(config["unit"], color='white', fontsize=8)
                ax.set_title(f"{config['title']} ({config['unit']})", color='white', fontsize=10)
                
                # Eksen limitlerini ayarla
                ax.set_ylim(config["min"], config["max"])
                
                # Normal aralık çizgileri
                if "normal_range" in config:
                    y_min, y_max = config["normal_range"]
                    ax.axhspan(y_min, y_max, color='#00ff00', alpha=0.1)  # Yeşil bölge
                
                # Eksen renkleri ve grid ayarları
                ax.tick_params(axis='x', colors='white', labelsize=8)
                ax.tick_params(axis='y', colors='white', labelsize=8)
                ax.grid(True, color='#3a3f44', linestyle='--', alpha=0.7)
                
                self.graphs.append((canvas, ax, config))
                self.data_times.append([])
                self.data_values.append([])
            else:
                print(f"HATA: graphicsView_{i + 1} bulunamadı!")

    def update_graph(self, index, new_value, title, color):
        """Daha yumuşak geçişler için veri filtreleme"""
        if index >= len(self.graphs): return
        
        # Low-pass filtre uygula (α = 0.3)
        if len(self.data_values[index]) > 0:
            prev_value = self.data_values[index][-1]
            new_value = 0.7*prev_value + 0.3*new_value
        
        time_list = self.data_times[index]
        value_list = self.data_values[index]
        
        # Veri ekleme ve sınırlama
        time_list.append(len(time_list) + 1)
        value_list.append(new_value)
        
        if len(time_list) > 100:
            time_list.pop(0)
            value_list.pop(0)
            
        canvas, ax, config = self.graphs[index]
        ax.clear()
        
        # Grafik stil ayarları
        ax.set_facecolor('#1e1e1e')
        ax.tick_params(axis='x', colors='white', labelsize=8)
        ax.tick_params(axis='y', colors='white', labelsize=8)
        ax.grid(True, color='#3a3f44', linestyle='--', alpha=0.7)
        
        # Normal aralık çizgilerini yeniden çiz
        if "normal_range" in config:
            y_min, y_max = config["normal_range"]
            ax.axhspan(y_min, y_max, color='#00ff00', alpha=0.1)
        
        # Çizgi grafiği oluştur
        line, = ax.plot(time_list, value_list, color or config['color'], linewidth=2)
        
        # Anlık değeri grafik üzerinde göster
        ax.annotate(f"{new_value:.2f}", 
                    xy=(time_list[-1], value_list[-1]),
                    xytext=(10, 10), textcoords='offset points',
                    color='white', fontsize=8,
                    bbox=dict(boxstyle='round,pad=0.5', fc='black', alpha=0.5))
        
        # Eksen limitlerini ayarla
        ax.set_xlim(max(0, len(time_list)-50), len(time_list)+1)
        ax.set_ylim(config["min"], config["max"])
        
        # Eksen etiketlerini ayarla
        ax.set_xlabel('Zaman (s)', color='white', fontsize=8)
        ax.set_ylabel(config["unit"], color='white', fontsize=8)
        ax.set_title(f"{config['title']} ({config['unit']})", color='white', fontsize=10)
        canvas.draw()

    def calculate_error_code(self, parts):
        """Şartname 2.2'ye göre hata kodunu hesapla ve durum değişikliklerinde sesli/görsel uyarı ver"""
        error_bits = []
        colors = []
        current_errors = set()
        error_messages = []
        warning_messages = []

        # Hata durumlarını kontrol et
        # 1. Model Uydu İniş Hızı (12-14 m/s)
        vel1 = float(parts.get('VEL', 0))
        in_range = (12 <= vel1 <= 14) if parts.get('STATU') == 'Model Uydu İniş' else True
        error_bits.append('0' if in_range else '1')
        colors.append('#00ff00' if in_range else '#ff0000')
        if not in_range and parts.get('STATU') == 'Model Uydu İniş':
            current_errors.add("vel1")
            msg = f"KRİTİK: Model uydu iniş hızı {vel1:.1f} m/s (12-14 m/s aralığı dışında)"
            if "vel1" not in self.previous_errors:
                warning_messages.append(msg)
            error_messages.append(msg)

        # 2. Görev Yükü İniş Hızı (6-8 m/s)
        vel2 = float(parts.get('VEL2', 0))
        in_range = (6 <= vel2 <= 8) if parts.get('STATU') in ['Görev Yükü İniş', 'Kurtarma'] else True
        error_bits.append('0' if in_range else '1')
        colors.append('#00ff00' if in_range else '#ff0000')
        if not in_range and parts.get('STATU') in ['Görev Yükü İniş', 'Kurtarma']:
            current_errors.add("vel2")
            msg = f"KRİTİK: Görev yükü iniş hızı {vel2:.1f} m/s (6-8 m/s aralığı dışında)"
            if "vel2" not in self.previous_errors:
                warning_messages.append(msg)
            error_messages.append(msg)

        # 3. Taşıyıcı Basınç Verisi
        press2 = float(parts.get('PRESS2', 0))
        has_data = press2 != 0 if parts.get('STATU') in ['Uçuşa Hazır', 'Yükselme', 'Model Uydu İniş'] else True
        error_bits.append('0' if has_data else '1')
        colors.append('#00ff00' if has_data else '#ff0000')
        if not has_data and parts.get('STATU') in ['Uçuşa Hazır', 'Yükselme', 'Model Uydu İniş']:
            current_errors.add("press2")
            msg = "UYARI: Taşıyıcı basınç verisi alınamıyor!"
            if "press2" not in self.previous_errors:
                warning_messages.append(msg)
            error_messages.append(msg)

        # 4. Görev Yükü Konum Verisi
        gps_alt = float(parts.get('GPS_ALT', 0))
        has_data = gps_alt != 0
        error_bits.append('0' if has_data else '1')
        colors.append('#00ff00' if has_data else '#ff0000')
        if not has_data:
            current_errors.add("gps")
            msg = "UYARI: GPS konum verisi alınamıyor!"
            if "gps" not in self.previous_errors:
                warning_messages.append(msg)
            error_messages.append(msg)

        # 5. Ayrılmama Durumu
        status = parts.get('STATU', '')
        separated = status in ['Ayrılma', 'Görev Yükü İniş', 'Kurtarma']
        should_separate = float(parts.get('Y1', 0)) >= 390  # 400m ±10m
        error_bits.append('0' if (separated or not should_separate) else '1')
        colors.append('#00ff00' if (separated or not should_separate) else '#ff0000')
        if not separated and should_separate:
            current_errors.add("separation")
            msg = f"KRİTİK: Ayrılma gerçekleşmedi! Yükseklik: {float(parts.get('Y1', 0)):.1f}m"
            if "separation" not in self.previous_errors:
                warning_messages.append(msg)
            error_messages.append(msg)

        # 6. Multi-spektral Sistem Çalışma Durumu
        filter_status = parts.get('FILTER_STATUS', '1')  # Varsayılan: çalışıyor
        error_bits.append('0' if filter_status == '1' else '1')
        colors.append('#00ff00' if filter_status == '1' else '#ff0000')
        if filter_status != '1':
            current_errors.add("filter")
            msg = "UYARI: Multi-spektral sistem çalışmıyor!"
            if "filter" not in self.previous_errors:
                warning_messages.append(msg)
            error_messages.append(msg)

        # Düzelen hataları kontrol et (önce vardı şimdi yok)
        resolved_errors = self.previous_errors - current_errors
        for error in resolved_errors:
            if error == "vel1":
                msg = "BİLGİ: Model uydu iniş hızı normale döndü"
            elif error == "vel2":
                msg = "BİLGİ: Görev yükü iniş hızı normale döndü"
            elif error == "press2":
                msg = "BİLGİ: Taşıyıcı basınç verisi alınmaya başlandı"
            elif error == "gps":
                msg = "BİLGİ: GPS konum verisi alınmaya başlandı"
            elif error == "separation":
                msg = "BİLGİ: Ayrılma gerçekleşti"
            elif error == "filter":
                msg = "BİLGİ: Multi-spektral sistem çalışmaya başladı"
            warning_messages.append(msg)
            error_messages.append(msg)

        # Hata mesajları varsa sesli ve görsel uyarı ver
        if error_messages:
            self.play_alert_sound("\n".join(error_messages))
            
        # Yeni hatalar için popup göster
        if warning_messages:
            self.show_alert_popup("\n".join(warning_messages))

        # Önceki hataları güncelle
        self.previous_errors = current_errors

        return ''.join(error_bits), colors

    def show_alert_popup(self, message):
        """Şartname 2.2'ye göre görsel uyarı penceresi göster"""
        try:
            msg_box = QMessageBox(self)
            msg_box.setIcon(QMessageBox.Icon.Warning)
            msg_box.setWindowTitle("ARAS Uyarı Sistemi")
            msg_box.setText(message)
            
            # Mesaj türüne göre ikon ayarla
            if "KRİTİK" in message:
                msg_box.setIcon(QMessageBox.Icon.Critical)
                msg_box.setStyleSheet("QLabel{ color: red; }")
            elif "UYARI" in message:
                msg_box.setIcon(QMessageBox.Icon.Warning)
                msg_box.setStyleSheet("QLabel{ color: orange; }")
            else:
                msg_box.setIcon(QMessageBox.Icon.Information)
                msg_box.setStyleSheet("QLabel{ color: green; }")
                
            msg_box.setStandardButtons(QMessageBox.StandardButton.Ok)
            msg_box.exec()
            
        except Exception as e:
            print(f"Popup gösterim hatası: {e}")

    def play_alert_sound(self, message):
        """Şartname 2.2'ye göre sesli uyarı ver (non-blocking)"""
        try:
            # Mesajı status bar'da göster (arayüzü bloke etmeden)
            self.statusBar().showMessage("ARAS: " + message.split('\n')[0], 5000)  # 5 saniye göster
            
            # Ses çalma işlemini thread'de yap
            def play_sound():
                try:
                    if sys.platform == 'win32':
                        import winsound
                        # Kritik hatalar için farklı ses
                        if "KRİTİK" in message:
                            winsound.Beep(2000, 1000)  # Yüksek frekans, uzun süre
                        elif "UYARI" in message:
                            winsound.Beep(1000, 500)   # Orta frekans, orta süre
                        else:
                            winsound.Beep(500, 300)     # Düşük frekans, kısa süre
                    elif sys.platform == 'darwin':
                        import os
                        if "KRİTİK" in message:
                            os.system('say "Kritik uyarı" -v "Melek"')
                        elif "UYARI" in message:
                            os.system('say "Uyarı" -v "Melek"')
                        else:
                            os.system('say "Bilgi" -v "Melek"')
                    else:  # Linux
                        import os
                        if "KRİTİK" in message:
                            os.system('spd-say "Kritik uyarı" -t child_female')
                        elif "UYARI" in message:
                            os.system('spd-say "Uyarı" -t child_female')
                        else:
                            os.system('spd-say "Bilgi" -t child_female')
                except Exception as e:
                    print(f"Ses çalma hatası: {e}")
            
            # Thread'de çalıştır
            sound_thread = Thread(target=play_sound)
            sound_thread.daemon = True
            sound_thread.start()
            
        except Exception as e:
            print(f"Uyarı gösterim hatası: {e}")
    def update_error_display(self, error_code, colors):
        for i, widget in enumerate(self.hata_widgets):
            widget.setStyleSheet(f"""
                background-color: {colors[i]}; 
                color: white;
                font-weight: bold;
                text-align: center;
            """)
            widget.setText("1" if error_code[i] == '1' else "0")

    def setup_multi_spectral_controls(self):
        """Şartname Gereksinim 35: Multi-Spektral Mekanik Filtreleme Kontrolü"""
        # Multi-spektral kontrol grubu
        self.groupBox_multi_spectral = self.findChild(QtWidgets.QGroupBox, "groupBox_multi_spectral")
        self.groupBox_multi_spectral.setTitle("Multi-Spektral Kontrol")
    
    # Filtre kodu giriş alanı
        self.lineEdit_filter_code = self.findChild(QtWidgets.QLineEdit, "lineEdit_filter_code")
        self.lineEdit_filter_code.setPlaceholderText("6G9R")
        self.lineEdit_filter_code.setMaxLength(4)
        self.lineEdit_filter_code.setValidator(QtGui.QRegularExpressionValidator(QtCore.QRegularExpression("[0-9][A-Za-z][0-9][A-Za-z]")))
    
    # Gönder butonu
        self.btn_send_filter = self.findChild(QtWidgets.QPushButton, "btn_send_filter")
        self.btn_send_filter.setText("Filtre Gönder")
        self.btn_send_filter.setToolTip("4 haneli filtre komutu gönder (Örnek: 6G9R)")
    
    # Durum göstergeleri
        self.lbl_current_filter = self.findChild(QtWidgets.QLabel, "lbl_current_filter")
        self.lbl_current_filter.setText("Aktif Filtre: Yok")
    
        self.lbl_remaining_time = self.findChild(QtWidgets.QLabel, "lbl_remaining_time")
        self.lbl_remaining_time.setText("Kalan Süre: 0s")
    
        self.lbl_filter_status = self.findChild(QtWidgets.QLabel, "lbl_filter_status")
        self.lbl_filter_status.setText("Durum: Hazır")
    
     # Timer ayarları
        self.filter_timer = QtCore.QTimer(self)
        self.filter_timer.timeout.connect(self.update_filter_timer)
        self.current_filter = None
        self.filter_remaining_time = 0
        self.total_filter_time = 0
    
        # Buton bağlantısı
        self.btn_send_filter.clicked.connect(self.send_filter_command)

           
        
    def send_filter_command(self):
        """Şartname Gereksinim 35: Filtre komutu gönderme"""
        code = self.lineEdit_filter_code.text().strip().upper()
    
        # Kod format kontrolü (Rakam-Harf-Rakam-Harf)
        if len(code) != 4 or not re.match(r"^\d[A-Za-z]\d[A-Za-z]$", code):
            QtWidgets.QMessageBox.warning(self, "Hatalı Kod", 
                "Geçersiz filtre kodu formatı!\nÖrnek: 6G9R (Rakam-Harf-Rakam-Harf)")
            return
    
    # Süre ve filtre kodlarını ayır
        try:
            time1 = int(code[0])
            filter1 = code[1]
            time2 = int(code[2])
            filter2 = code[3]
        except:
            QtWidgets.QMessageBox.warning(self, "Hatalı Kod", "Kod ayrıştırma hatası!")
            return
        
        # Toplam süre kontrolü (maksimum 15 saniye)
        total_time = time1 + time2
        if total_time > 15:
            QtWidgets.QMessageBox.warning(self, "Aşım Uyarısı", 
                f"Toplam süre 15 saniyeyi aşıyor: {total_time}s\nKomut gönderilmeyecek!")
            return
        
    # Komut gönderme
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(f"FILTER:{code}\n".encode('utf-8'))
                self.current_filter = code
                self.filter_remaining_time = total_time
                self.total_filter_time = total_time
                self.lbl_current_filter.setText(f"Aktif Filtre: {code}")
                self.lbl_filter_status.setText("Durum: Aktif")
                self.lbl_remaining_time.setText(f"Kalan Süre: {total_time}s")
                self.filter_timer.start(1000)  # 1 saniyede bir güncelle
                
                # Şartname Gereksinim 35: Filtreleme sonrası standart (N) duruma dön
                QtCore.QTimer.singleShot(total_time * 1000 + 2000, self.reset_to_standard_filter)
                
            except Exception as e:
                QtWidgets.QMessageBox.critical(self, "Gönderme Hatası", 
                    f"Filtre komutu gönderilemedi:\n{str(e)}")
        else:
            QtWidgets.QMessageBox.warning(self, "Bağlantı Yok", 
                "Seri bağlantı aktif değil! Simülasyon modunda çalışılıyor.")
            
            # Simülasyon modu
            self.current_filter = code
            self.filter_remaining_time = total_time
            self.total_filter_time = total_time
            self.lbl_current_filter.setText(f"Aktif Filtre: {code} (Sim)")
            self.lbl_filter_status.setText("Durum: Aktif (Sim)")
            self.lbl_remaining_time.setText(f"Kalan Süre: {total_time}s")
            self.filter_timer.start(1000)
            QtCore.QTimer.singleShot(total_time * 1000 + 2000, self.reset_to_standard_filter)


    def validate_filter_code(self, code):
        """Kod formatını kontrol et (Rakam-Harf-Rakam-Harf)"""
        pattern = r"^[0-9][A-Za-z][0-9][A-Za-z]$"
        return re.match(pattern, code) is not None
        
    def get_filter_duration(self, time_part):
        """Filtre süresini hesapla (ilk iki karakter)"""
        try:
            return int(time_part)
        except ValueError:
            return 0
            
    def update_filter_timer(self):
        """Şartname Gereksinim 35: Filtre zamanlayıcısını güncelle"""
        if self.filter_remaining_time > 0:
            self.filter_remaining_time -= 1
            self.lbl_remaining_time.setText(f"Kalan Süre: {self.filter_remaining_time}s")
            
            # İlerleme çubuğu efekti (isteğe bağlı)
            progress = 100 - int((self.filter_remaining_time / self.total_filter_time) * 100)
            self.lbl_filter_status.setText(f"Durum: Çalışıyor (%{progress})")
        else:
            self.filter_timer.stop()
            self.lbl_filter_status.setText("Durum: Tamamlandı")

    def reset_to_standard_filter(self):
        """Şartname Gereksinim 35: Standart (N) filtre durumuna dön"""
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write("FILTER:00NN\n".encode('utf-8'))
            except:
                pass
        
        self.current_filter = None
        self.lbl_current_filter.setText("Aktif Filtre: Yok")
        self.lbl_filter_status.setText("Durum: Hazır")
        self.lbl_remaining_time.setText("Kalan Süre: 0s")
        
        # Şartname Gereksinim 35: 2 saniyeden fazla gecikme olmamalı
        QtWidgets.QMessageBox.information(self, "Filtreleme Tamamlandı", 
            "Multi-spektral filtreleme tamamlandı ve standart (N) durumuna dönüldü.")

    def update_data(self):
        """Telemetri verilerini güncelle ve görev zamanını ekle"""
        # Önceki değerleri saklamak için sınıf değişkeni kontrolü
        if not hasattr(self, 'prev_values'):
            self.prev_values = {
                'y1': 0.0,          # Yükseklik 1 (m)
                'y2': 0.0,          # Yükseklik 2 (m)
                'press1': 1013.25,  # Basınç 1 (hPa)
                'press2': 1013.25,  # Basınç 2 (hPa)
                'vel1': 0.0,        # İniş Hızı 1 (m/s)
                'vel2': 0.0,        # İniş Hızı 2 (m/s)
                'temp': 25.0,       # Sıcaklık (°C)
                'bat': 4.2,         # Pil Gerilimi (V)
                'status_counter': 0 # Durum geçiş sayacı
            }

        # Görev zamanını güncelle
        if self.mission_active and self.mission_start_time:
            current_elapsed = self.mission_start_time.secsTo(QDateTime.currentDateTime())
            total_time = self.mission_elapsed_time + current_elapsed
            formatted_time = self.format_mission_time(total_time)
        else:
            formatted_time = self.format_mission_time(self.mission_elapsed_time)

        if self.serial_port and self.serial_port.is_open:
            # Gerçek veri okuma kodu (orijinal mantık korundu)
            try:
                line = self.serial_port.readline().decode('utf-8').strip()
                parts = dict(item.split(":") for item in line.split(";") if ":" in item)
                
                # Paket numarasını ve görev zamanını ekle
                parts['PAKET'] = str(self.packet_counter)
                parts['GOREV_ZAMANI'] = formatted_time
                self.packet_counter += 1

                # Diğer işlemler...
                
            except Exception as e:
                print(f"Veri okuma hatası: {e}")
                parts = {}
        else:
        
            
            # Durum geçiş kontrolü (Her 20 pakette bir durum değişimi)
            status_sequence = [
                'Uçuşa Hazır', 'Yükselme', 'Yükselme', 'Model Uydu İniş',
                'Ayrılma', 'Görev Yükü İniş', 'Kurtarma'
            ]
            current_status = status_sequence[
                min(self.prev_values['status_counter'] // 20, len(status_sequence)-1)
            ]
            
            # Fiziksel model parametreleri
            delta_time = 1.0  # 1 saniyelik zaman adımı
            
            # Yükseklik simülasyonu (Duruma göre dinamik davranış)
            if current_status == 'Yükselme':
                self.prev_values['y1'] += random.uniform(8.0, 12.0)
            elif current_status == 'Model Uydu İniş':
                self.prev_values['y1'] -= random.uniform(12.0, 14.0)
                self.prev_values['y1'] = max(0, self.prev_values['y1'])  # Negatif koruması
            elif current_status == 'Görev Yükü İniş':
                self.prev_values['y1'] -= random.uniform(6.0, 8.0)
                self.prev_values['y1'] = max(0, self.prev_values['y1'])  # Negatif koruması
            
            # Yükseklik 2 (Taşıyıcı) için ilişkili veri
            self.prev_values['y2'] = max(0, self.prev_values['y1'] * 0.98 + random.uniform(-0.5, 0.5))
    
            # Basınç simülasyonu (Uluslararası Standart Atmosfer modeli)
            self.prev_values['press1'] = 1013.25 * (1 - (self.prev_values['y1']/44330)) ** 5.255
            self.prev_values['press2'] = self.prev_values['press1'] * 0.99 + random.uniform(-2, 2)

            # Hız simülasyonu (Yüksekliğin türevi)
            current_vel = (self.prev_values['y1'] - self.prev_values.get('last_y1', 0)) / delta_time
            self.prev_values['vel1'] = 0.8*self.prev_values['vel1'] + 0.2*current_vel
            self.prev_values['vel2'] = self.prev_values['vel1'] * 0.7 + random.uniform(-0.1, 0.1)

            # Diğer parametreler
            self.prev_values['temp'] += random.uniform(-0.1, 0.1)
            self.prev_values['temp'] = max(-10, min(50, self.prev_values['temp']))
            self.prev_values['bat'] -= 0.00027  # ~3 saatlik pil ömrü simülasyonu
            self.prev_values['bat'] = max(3.3, self.prev_values['bat'])
            self.prev_values['status_counter'] += 1

            # Veri paketini oluştur
            parts = {
                'PAKET': str(self.packet_counter),
                'STATU': current_status,
                'VEL': round(self.prev_values['vel1'], 2),
                'VEL2': round(self.prev_values['vel2'], 2),
                'PRESS1': round(self.prev_values['press1'], 2),
                'PRESS2': round(self.prev_values['press2'], 2),
                'Y1': round(max(0, self.prev_values['y1']), 2),  # Negatif koruma
                'Y2': round(max(0, self.prev_values['y2']), 2),  # Negatif koruma
                'TEMP': round(self.prev_values['temp'], 1),
                'BAT': round(self.prev_values['bat'], 3),
                'LAT': f"37.{random.randint(10000, 99999)}",
                'LON': f"28.{random.randint(10000, 99999)}",
                'GPS_ALT': round(self.prev_values['y1'] + random.uniform(-0.5, 0.5), 2),
                'PITCH': round(random.uniform(-5, 5), 1),
                'ROLL': round(random.uniform(-5, 5), 1),
                'YAW': round(random.uniform(0, 360), 1),
                'RHRH': random.choice(['6G9R', '7Y8P', '8B7P', '9Y6B']),
                'IOT1': round(25 + random.uniform(-0.5, 0.5), 1),
                'IOT2': round(25 + random.uniform(-0.5, 0.5), 1),
                'TAKIM': '600898',
                'TIME': QDateTime.currentDateTime().toString("dd/MM/yyyy, hh:mm:ss")
            }
            self.packet_counter += 1
            self.prev_values['last_y1'] = self.prev_values['y1']  # Son yüksekliği kaydet

        # Hata kodunu hesapla ve göster
        error_code, colors = self.calculate_error_code(parts)
        self.update_error_display(error_code, colors)
        parts['HATA'] = error_code

        # Grafikleri güncelle
        self.update_graph(0, float(parts.get('Y1', 0)), "Yükseklik 1 (m)", '#1f77b4')
        self.update_graph(1, float(parts.get('Y2', 0)), "Yükseklik 2 (m)", '#ff7f0e')
        self.update_graph(2, float(parts.get('PRESS1', 0)), "Basınç 1 (hPa)", '#2ca02c')
        self.update_graph(3, float(parts.get('PRESS2', 0)), "Basınç 2 (hPa)", '#d62728')
        self.update_graph(4, float(parts.get('VEL', 0)), "İniş Hızı (m/s)", '#9467bd')
        self.update_graph(5, float(parts.get('TEMP', 0)), "Sıcaklık (°C)", '#8c564b')
        self.update_graph(6, float(parts.get('IOT1', 0)), "IoT S1 Sıcaklık (°C)", '#e377c2')
        self.update_graph(7, float(parts.get('IOT2', 0)), "IoT S2 Sıcaklık (°C)", '#7f7f7f')

        # Gyro simülasyonu (Duruma göre açılar)
        if current_status == 'Yükselme':
            pitch = random.uniform(-2, 2)
            roll = random.uniform(-2, 2)
        elif current_status == 'Model Uydu İniş':
            pitch = random.uniform(-5, 5)
            roll = random.uniform(-5, 5)
        else:
            pitch = 0
            roll = 0
        self.plot_gyro(pitch=pitch, roll=roll, yaw=random.uniform(0, 360))

        # Tabloya veri ekle
        if parts:
            row_position = self.table_telemetri.rowCount()
            self.table_telemetri.insertRow(row_position)
            
            # İrtifa farkı hesaplama (mutlak değer ve negatif koruma)
            try:
                y1 = max(0, float(parts.get('Y1', 0)))
                y2 = max(0, float(parts.get('Y2', 0)))
                altitude_diff = round(abs(y1 - y2), 2)
            except:
                altitude_diff = 0.0
            columns = [
                parts['PAKET'],
                parts['STATU'],
                parts['HATA'],
                parts['TIME'],
                parts['PRESS1'],
                parts['PRESS2'],
                parts['Y1'],
                parts['Y2'],
                round(abs(float(parts['Y1']) - float(parts['Y2'])), 2),
                parts['VEL'],
                parts['TEMP'],
                parts['BAT'],
                parts['LAT'],
                parts['LON'],
                parts['GPS_ALT'],
                parts['PITCH'],
                parts['ROLL'],
                parts['YAW'],
                parts['RHRH'],
                parts['IOT1'],
                parts['IOT2'],
                parts['TAKIM']
            ]
            
            for col, value in enumerate(columns):
                item = QtWidgets.QTableWidgetItem(str(value))
                self.table_telemetri.setItem(row_position, col, item)

    def get_simulated_status(self):
        """Gerçekçi durum geçişleri için geliştirilmiş fonksiyon"""
        status_sequence = [
            'Uçuşa Hazır', 'Yükselme', 'Yükselme', 'Model Uydu İniş',
            'Ayrılma', 'Görev Yükü İniş', 'Kurtarma'
        ]
        
        current_index = min(int(self.packet_counter / 20), len(status_sequence)-1)
        return status_sequence[current_index]

    def get_simulated_velocities(self, status):
        """Şartname 2.1'e göre iniş hızlarını simüle et"""
        if status == 'Model Uydu İniş':
            vel1 = random.uniform(12, 14)  # 12-14 m/s
            vel2 = 0
        elif status in ['Görev Yükü İniş', 'Kurtarma']:
            vel1 = 0
            vel2 = random.uniform(6, 8)    # 6-8 m/s
        else:
            vel1 = 0
            vel2 = 0
        return vel1, vel2

    def get_simulated_pressures(self, status):
        """Şartname 2.1'e göre basınç verilerini simüle et"""
        if status == 'Uçuşa Hazır':
            press1 = random.uniform(1010, 1015)  # Yer seviyesi basıncı
            press2 = press1 + random.uniform(-5, 5)
        elif status == 'Yükselme':
            press1 = random.uniform(800, 1000)
            press2 = press1 + random.uniform(-5, 5)
        elif status == 'Model Uydu İniş':
            press1 = random.uniform(900, 1100)
            press2 = press1 + random.uniform(-5, 5)
        else:
            press1 = random.uniform(900, 1100)
            press2 = 0  # Ayrıldıktan sonra taşıyıcı basıncı alınamaz
        return press1, press2

    def get_simulated_altitudes(self, status):
        """Şartname 2.4'e göre yükseklik verilerini simüle et (negatif değerleri önle)"""
        if status == 'Uçuşa Hazır':
            y1 = 0
            y2 = 0
        elif status == 'Yükselme':
            y1 = random.uniform(0, 400)  # Minimum 0m
            y2 = max(0, y1 + random.uniform(-5, 5))  # Negatif önleme
        elif status == 'Model Uydu İniş':
            y1 = random.uniform(0, 400)  # Minimum 0m
            y2 = max(0, y1 + random.uniform(-5, 5))
        elif status == 'Ayrılma':
            y1 = random.uniform(390, 410)
            y2 = max(0, y1 + random.uniform(-5, 5))
        elif status == 'Görev Yükü İniş':
            y1 = random.uniform(0, 300)  # Minimum 0m
            y2 = 0
        else:  # Kurtarma
            y1 = 0
            y2 = 0
        return max(0, y1), max(0, y2)  # Son kontroller


    def handle_filter_response(self, response):
        """Şartname Gereksinim 35: Uydudan gelen filtre yanıtını işle"""
        if response.startswith("FILTER:"):
            code = response[7:].strip()
            if code == "ACK":
                # Başarılı yanıt
                self.lbl_filter_status.setText("Durum: Onaylandı")
            elif code == "ERR":
                # Hata yanıtı
                self.filter_timer.stop()
                self.lbl_filter_status.setText("Durum: Hata!")
                self.lbl_current_filter.setText("Aktif Filtre: Hata!")
                QtWidgets.QMessageBox.warning(self, "Filtre Hatası", 
                    "Uydu filtre komutunu işleyemedi!")
            else:
                # Bilinmeyen yanıt
                print(f"Bilinmeyen filtre yanıtı: {code}")

    def setup_3d_gyro(self):
        """Gyro modelini zeminde dik konumda başlat"""
        try:
            # STL dosyasını yükle ve orijinal yönlendirmeyi koru
            self.gyro_mesh = mesh.Mesh.from_file('gyro1.stl')
    
            # Modeli merkezle ve ölçeklendir
            mesh_vectors = self.gyro_mesh.vectors.copy()
            centroid = np.mean(mesh_vectors, axis=(0,1))
            mesh_vectors -= centroid
    
            # Modeli dikey hizala için X ekseninde -90 derece döndür
            R_x = np.array([[1, 0, 0],
                            [0, np.cos(np.pi/2), -np.sin(np.pi/2)],
                            [0, np.sin(np.pi/2), np.cos(np.pi/2)]])
            mesh_vectors = np.dot(mesh_vectors, R_x.T)
    
            # Ölçeklendirme faktörünü ayarla
            self.gyro_mesh.vectors = mesh_vectors / np.ptp(mesh_vectors) * 3.5
    
    # Matplotlib figürünü yapılandır
            self.figure_3d = Figure(figsize=(7,7), facecolor='#2b2b2b')
            self.canvas_3d = FigureCanvas(self.figure_3d)
            self.toolbar = NavigationToolbar(self.canvas_3d, self)
    
    # 3D eksen ayarları
            self.ax_3d = self.figure_3d.add_subplot(111, projection='3d')
            self.ax_3d.set_facecolor('#2b2b2b')
            self.ax_3d.grid(False)
            self.ax_3d.axis('off')
            self.ax_3d.set_xlim([-2, 2])
            self.ax_3d.set_ylim([-2, 2])
            self.ax_3d.set_zlim([-2, 2])
    
    # Sabit kamera açısı (Top-down view)
            self.ax_3d.view_init(elev=30, azim=45)  # Tam karşıdan bakış0,0
    
    # Layout'a ekle
            layout = QVBoxLayout(self.widget_harita)
            layout.setContentsMargins(10,10,10,10)
            layout.addWidget(self.toolbar)
            layout.addWidget(self.canvas_3d)
    
    # İlk çizimi yap
            self.plot_gyro(0, 0, 0)
    
        except Exception as e:
            print(f"3D Model Hatası: {str(e)}")
            self.widget_harita.setVisible(False)

    def plot_gyro(self, pitch=0, roll=0, yaw=0):
        """Gyro modelini belirtilen açılarda çiz (sabit arka plan)"""
        try:
            self.ax_3d.clear()
        
        # Orijinal mesh vektörlerini kopyala
            rotated_vectors = self.gyro_mesh.vectors.copy()
        
        # Rotasyon sırası: Yaw -> Pitch -> Roll (ZYX konvansiyonu)
            rotated_vectors = self.rotate(rotated_vectors, np.radians(yaw), 'z')
            rotated_vectors = self.rotate(rotated_vectors, np.radians(pitch), 'x')
            rotated_vectors = self.rotate(rotated_vectors, np.radians(roll), 'y')
        
        # 3D çizimi güncelle
            self.ax_3d.add_collection3d(mplot3d.Poly3DCollection(
                rotated_vectors,
                facecolors='#00ffff',
                edgecolors='#333333',
                linewidths=1.0,
                alpha=0.95
            ))
        
        # Eksen limitlerini ve görünümü sabit tut
            self.ax_3d.set_xlim([-1.5, 1.5])
            self.ax_3d.set_ylim([-1.5, 1.5])
            self.ax_3d.set_zlim([-1.5, 1.5])
            self.ax_3d.view_init(elev=30, azim=45)
            self.canvas_3d.draw()
        
        except Exception as e:
            print(f"Çizim Hatası: {str(e)}")

    def rotate(self, vectors, angle, axis):
        """3D rotasyon fonksiyonu (değişmeden kalır)"""
        if axis == 'x':
            R = np.array([
                [1, 0, 0],
                [0, np.cos(angle), -np.sin(angle)],
                [0, np.sin(angle), np.cos(angle)]
            ])
        elif axis == 'y':
            R = np.array([
                [np.cos(angle), 0, np.sin(angle)],
                [0, 1, 0],
                [-np.sin(angle), 0, np.cos(angle)]
            ])
        elif axis == 'z':
            R = np.array([
                [np.cos(angle), -np.sin(angle), 0],
                [np.sin(angle), np.cos(angle), 0],
                [0, 0, 1]
            ])
        return np.dot(vectors, R.T)
    
if __name__ == "__main__":
    app = QtWidgets.QApplication([])
    window = MainWindow()
    window.show()
    app.exec()