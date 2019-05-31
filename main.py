import sys
import numpy as np
import cv2
import math
import serial
import serial.tools.list_ports
import time
from time import sleep
from PyQt5 import QtCore, QtGui, QtWidgets, uic
from PyQt5.QtWidgets import QApplication, QWidget, QLabel
from PyQt5.QtGui import QIcon, QPixmap, QImage
from PyQt5.QtCore import QTimer
from ui_main import *

lucas_kanade_params = dict(winSize=(15, 15), maxLevel=2,
                           criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
Stream = 'rtsp://admin:adminEXCZCN@192.168.137.49:554/Streaming/Channels/101/'

class MainWindow(QWidget):
    interval = 20
    keypoint_accuracy = 0.50
    bounding_rect_accuracy = 0.90
    object_detection_accuracy = 0.80
    cap = cv2.VideoCapture(Stream)
    width = cap.get(3)
    height = cap.get(4)
    prev_frame = None
    prev_gray = None
    prev_points = []
    new_points = []
    mask = None
    tracker_enabled = False
    click_count = 0
    top_left = (0, 0)
    bottom_right = (0, 0)
    frame = None
    frame_gray = None
    selected_image = None
    selected_image_gray = None
    scan_count = 0
    find_keypoints_called = 0
    initial_keypoints = 0
    bounding_rect_top_left = (0, 0)
    bounding_rect_bottom_right = (0, 0)
    coords = []
    error = None
    rect_point1 = (0, 0)
    rect_point2 = (0, 0)
    find_selected_image_keypoints = False
    kp2, des2 = None, None
    center_x = 0
    center_y = 0
    key, key2 = None, None
    center = (0, 0)
    angle_of_center = 0
    distance_of_center = 0


    ################################ initialization function #########################################
    def __init__(self):
        # Take first frame and find points in it
        #COMPORT = 3  # Enter Your COM Port Number Here.
        global ser
        ser = serial.Serial()
        # configure serial port
        ser.bytesize = serial.EIGHTBITS
        ser.parity = serial.PARITY_NONE

        # call QWidget constructor    
        super().__init__()
        self.ui = Ui_Form()
        self.ui.setupUi(self)   
        self.ui.Pause.setCheckable(True)
        self.ui.Reset.setCheckable(True)
        self.ui.Exit.setCheckable(True)
        self.ui.Start.clicked.connect(self.start)
        self.ui.Start.setCheckable(True)
        self.ui.Exit.clicked.connect(lambda: self.close())
        self.ui.Exit.clicked.connect(lambda: self.end()) 
        self.ui.open_btn.clicked.connect(self.OpenPort) 
        self.ui.close_btn.clicked.connect(self.ClosePort)
        self.ui.refresh_btn.clicked.connect(self.RefreshPort)
        self.ui.keypoints_accuracy_slider.valueChanged.connect(lambda: self.slider())
        self.ui.rectangle_accuracy_slider.valueChanged.connect(lambda: self.slider())
        self.ui.object_accuracy_slider.valueChanged.connect(lambda: self.slider())
         # create timers for cam and uart
        self.timer_uart = QTimer()

        self.timer_uart.setInterval(500)
        
        # set timer timeout callback function
        self.timer_uart.timeout.connect(self.send_data) 
        # auto find all serial ports avaiable in computer (doesn't need to click refresh button)
        self.RefreshPort()
                  
          

    #################################### select_roi function #########################################
    def select_roi(self, event, x, y, flags, params):
        if (self.click_count == 1):
            self.rect_point2 = (x, y)
        if event == cv2.EVENT_LBUTTONDOWN and self.click_count < 2:
            self.click_count += 1
            if self.click_count == 1:
                self.top_left = (x, y)
                self.rect_point1 = self.top_left
            elif self.click_count == 2:
                if x < self.top_left[0] or y < self.top_left[1]:
                    self.click_count -= 1
                else:
                    self.bottom_right = (x, y)
                    self.selected_image = self.frame[self.top_left[1]:self.bottom_right[1],
                                          self.top_left[0]:self.bottom_right[0], :]
                    self.selected_image_gray = cv2.cvtColor(self.selected_image, cv2.COLOR_BGR2GRAY)
                    self.find_selected_image_keypoints = True
                    self.find_keypoints()
                    if self.error is None:
                        self.tracker_enabled = True
                    self.find_selected_image_keypoints = False
                    
    #################################### find_keypoints function #######################################
    def find_keypoints(self):
        surf = cv2.xfeatures2d.SURF_create(400)
        kp1, des1 = surf.detectAndCompute(self.frame_gray, None)

        if self.find_selected_image_keypoints:
            self.kp2, self.des2 = surf.detectAndCompute(self.selected_image_gray, None)
            if len(self.kp2) < 3:
                self.error = "No keypoints found, Try again."
                return
            else:
                self.error = None

        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)
        flann = cv2.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(des1, self.des2, k=2)
        
        good = []
        for m, n in matches:
            if m.distance < 0.7 * n.distance:
                good.append(m)

        list_kp1 = []
        
        for mat in good:
            img1_idx = mat.queryIdx
            (x1, y1) = kp1[img1_idx].pt
            list_kp1.append((x1, y1))
            

        self.prev_points = []
        self.prev_points = np.asarray(self.prev_points, dtype=np.float32).reshape(-1, 2)
        for (x, y) in list_kp1:
            x = np.float32(x)
            y = np.float32(y)
            self.prev_points = np.append(self.prev_points, [x, y])
        self.prev_points = self.prev_points.reshape(-1, 1, 2)

        if self.find_keypoints_called == 0:
            self.find_keypoints_called += 1
            self.initial_keypoints = len(list_kp1)

            
    #################################### find_bounding_rect_coords function ############################
    def find_bounding_rect_coords(self):
        coords_count = self.coords.shape[0]
        coords_x = [x for x, y in self.coords]
        coords_y = [y for x, y in self.coords]
        left = min(coords_x)
        right = max(coords_x)
        top = min(coords_y)
        bottom = max(coords_y)

        tolerance = coords_count * (1 - self.bounding_rect_accuracy)
        while left < self.width:
            left += 10
            temp = [x for x, y in self.coords if x < left]
            if len(temp) > tolerance:
                temp = [x for x, y in self.coords if x < left + 10]
                left = max(temp)
                break

        while right > 0:
            right -= 10
            temp = [x for x, y in self.coords if x > right]
            if len(temp) > tolerance:
                temp = [x for x, y in self.coords if x > right - 10]
                right = min(temp)
                break
        while top < self.height:
            top += 10
            temp = [y for x, y in self.coords if y < top]
            if len(temp) > tolerance:
                temp = [y for x, y in self.coords if y < top + 10]
                top = max(temp)
                break
        while bottom > 0:
            bottom -= 10
            temp = [y for x, y in self.coords if y > bottom]
            if len(temp) > tolerance:
                temp = [y for x, y in self.coords if y > bottom - 10]
                bottom = min(temp)
                break
        self.bounding_rect_top_left = (int(left), int(top))
        self.bounding_rect_bottom_right = (int(right), int(bottom))

    def start(self):
        self.timer_uart.start()
        self.ui.Start.toggle()

        _, self.prev_frame = self.cap.read()
        self.prev_frame = self.rescale_frame(self.prev_frame, 50)
        self.prev_gray = cv2.cvtColor(self.prev_frame, cv2.COLOR_BGR2GRAY)
        self.prev_points = np.asarray(self.prev_points, dtype=np.float32).reshape(-1, 1, 2)
        self.new_points = np.asarray(self.new_points, dtype=np.float32).reshape(-1, 1, 2)
        cv2.namedWindow('Object Tracking')
        cv2.setMouseCallback('Object Tracking', self.select_roi)

        while (True):
            ret, self.frame = self.cap.read()
            self.frame = self.rescale_frame(self.frame, 50)
            self.frame_gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
            if ret:
                if self.tracker_enabled:  # and self.prev_points is not None:
                    self.scan_count += 1
                    self.prev_points = self.prev_points.reshape(-1, 1, 2)

                    self.new_points, status, errors = cv2.calcOpticalFlowPyrLK(self.prev_gray, self.frame_gray,
                                                                               self.prev_points, None,
                                                                               **lucas_kanade_params)

                    if self.new_points is not None:
                        temp = self.new_points[status == 1]
                        self.coords = temp.reshape(-1, 2)
                        
                        if len(self.coords) > self.initial_keypoints * (1 - self.object_detection_accuracy):
                            if len(self.coords) != 0:
                                self.find_bounding_rect_coords()
                                cv2.rectangle(self.frame, self.bounding_rect_top_left,
                                                  self.bounding_rect_bottom_right, (255, 0, 0), 2)

                            ### Center of bounding box
                            center_x1 = (self.bounding_rect_top_left[0]+self.bounding_rect_bottom_right[0])/2
                            center_y1 = (self.bounding_rect_top_left[1]+self.bounding_rect_bottom_right[1])/2
                            self.center = (center_x1,center_y1)

                            ### Distance and angle of bounding box's center
                            self.angle_of_center = round(self.angle(self.center,self.width,self.height),4)
                            self.distance_of_center = round(self.distance(self.center,(self.width/2, self.height/2)),4)

                            cv2.circle(self.frame, (int(center_x1), int(center_y1)), 5, (255, 0, 0), 2)
                        else:
                            cv2.putText(self.frame, "Object Not Found", (0, 20), cv2.FONT_HERSHEY_SIMPLEX, 1,
                                        color=(200, 50, 75), thickness=3)

                        self.prev_gray = self.frame_gray.copy()
                        self.prev_points = self.new_points.reshape(-1, 1, 2)
                        
                    cond_1 = self.new_points is None
                    cond_2 = self.prev_points[status == 1].shape[0] < self.initial_keypoints * self.keypoint_accuracy
                    cond_3 = self.scan_count % self.interval == 0
                    if cond_1 or cond_2 or cond_3:
                        self.find_keypoints()
                        self.error = None
                    if self.scan_count == 500:
                        self.scan_count == 0

                error_free_frame = self.frame.copy()
                if self.error is not None:
                    cv2.putText(self.frame, str(self.error), (0, 20), cv2.FONT_HERSHEY_SIMPLEX, 1, color=(200, 50, 75),
                                thickness=3)

                frame_copy = self.frame.copy()
                if self.click_count == 1 and self.rect_point2[0] > self.rect_point1[0] and self.rect_point2[1] > \
                        self.rect_point1[1]:
                    cv2.rectangle(frame_copy, self.rect_point1, self.rect_point2, (255, 0, 0), 2)
                    cv2.imshow('Object Tracking', frame_copy)
                else:
                    cv2.imshow('Object Tracking', self.frame)
                
                
                self.key = cv2.waitKey(10)
                if self.ui.Pause.isChecked():
                    self.ui.Pause.toggle()
                    self.timer_uart.stop()
                    self.key2 = cv2.waitKey(10)
                    while not (self.ui.Start.isChecked()):
                        if self.ui.Reset.isChecked():
                            self.frame = error_free_frame
                            if self.ui.Exit.isChecked():
                                break
                        elif self.ui.Exit.isChecked():
                            break


                        if self.click_count == 1 and self.rect_point2[0] > self.rect_point1[0] and self.rect_point2[1] > \
                                self.rect_point1[1]:
                            frame_copy = self.frame.copy()
                            cv2.rectangle(frame_copy, self.rect_point1, self.rect_point2, (255, 0, 0), 2)
                            cv2.imshow('Object Tracking', frame_copy)
                            
                        else:
                            cv2.imshow('Object Tracking', self.frame)
                            
                        self.key2 = cv2.waitKey(10)
                        continue
                    self.end()

                        
                if self.ui.Reset.isChecked():
                    self.ui.Reset.toggle()
                    self.click_count = 0
                    self.tracker_enabled = False
                    self.distance_of_center = 0
                    self.angle_of_center = 0
                    self.error = None
                if self.ui.Exit.isChecked():  
                    break
        self.end()

    def end(self):
        self.cap.release()
        ser.close()
        cv2.destroyAllWindows()

    def distance(self, point1, point2):
        return math.sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)

    def angle(self, point,width,height):
        if point[0]<width/2 and point[1]<height/2: 
            return math.degrees(math.atan((point[1]-height/2)/(point[0]-width/2)))-180
        elif point[0]<width/2 and point[1]>height/2:
            return math.degrees(math.atan((point[1]-height/2)/(point[0]-width/2)))+180
        elif point[0]==width/2 and point[1]<height/2:
            return -90
        elif point[0]==width/2 and point[1]>height/2:
            return 90   
        else:
            return math.degrees(math.atan((point[1]-height/2)/(point[0]-width/2)))       


    ########################################## UI ###################################################          
    def OpenPort(self):
        ser.port= self.ui.portname.currentText()
        ser.baudrate = self.ui.baudrate.currentText()
        ser.open()
        print('Open: ' + ser.portstr)

    def ClosePort(self):
        ser.close()
        print('Close: ' + ser.portstr)

    def RefreshPort(self):
        self.ui.portname.clear()
        ports = list(serial.tools.list_ports.comports())  
        for p in ports:
            self.ui.portname.addItem(p[0])  

    def slider(self):
    	self.keypoint_accuracy = self.ui.keypoints_accuracy_slider.value()/100
    	self.bounding_rect_accuracy = self.ui.rectangle_accuracy_slider.value()/100
    	self.object_detection_accuracy = self.ui.object_accuracy_slider.value()/100

    def send_data(self):
        try:
            m = ['CAM,', str(self.distance_of_center), ',', str(self.angle_of_center), '\n']
            me = "".join(m)
            ser.write(me.encode('utf-8'))
            text = time.strftime("%H:%M:%S") + ' ' + me
            self.ui.sent_box.append(text)
        except Exception as e:
            pass

    def rescale_frame(self, frame, percent=75):
        width = int(frame.shape[1] * percent / 100)
        height = int(frame.shape[0] * percent / 100)
        dim = (width, height)
        return cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)

if __name__ == '__main__':
    app = QApplication(sys.argv)

    # create and show mainWindow
    mainWindow = MainWindow()
    mainWindow.show()

    sys.exit(app.exec_())
