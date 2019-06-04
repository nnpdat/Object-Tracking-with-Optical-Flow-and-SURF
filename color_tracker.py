import cv2
import numpy as np
import serial
import sys

link = ['rtsp://admin:adminEXCZCN@', sys.argv[1], ':554/Streaming/Channels/101/']
Stream = "".join(link)
serialPort = serial.Serial('COM1', 115200, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)
#Stream = 0 
# width = 960, height = 540
class ColorTracker(object):              
    send_count = 0
    hue_tolerance = 4
    mouse_clicked = False
    #frame = np.zeros((480,640,3), np.uint8)
    b, g, r = 0, 0, 0
    cx, cy = 0, 0
    cap = None
    color_lower_range = (0,0,0)
    color_higher_range = (0,0,0)
    kernel1 = np.ones((3,3),np.uint8)
    kernel2 = np.ones((8,8),np.uint8)

    centerX = 0
    centerY = 0

    take_size_frame = True

    def __init__(self) :
        self.cap = cv2.VideoCapture(Stream)
        cv2.namedWindow("Color Tracker")
        cv2.setMouseCallback('Color Tracker', self.getCoords)

    def getCoords(self,event,x,y,flags,param) :
        if event == cv2.EVENT_LBUTTONDOWN:
            self.cx, self.cy = x, y
            
            self.b, self.g, self.r = self.frame[self.cy,self.cx]
            self.b, self.g, self.r = int(self.b), int(self.g), int(self.r)

            frame_hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
            h,s,v = frame_hsv[self.cy,self.cx]
            h,s,v = int(h),int(s),int(v)
            print(h,s,v)

            h_low = h - self.hue_tolerance
            h_high = h + self.hue_tolerance
            if h_low < 0 :
                h_low = 0
            if h_high > 179 : 
                h_high = 179
            
            s_low = s - 10
            s_high = s + 10
            if s_low < 0 :
                s_low = 0
            if s_high > 255 : 
                s_high = 255

            v_low = v - 15
            v_high = v + 15
            if v_low < 0 :
                v_low = 0
            if v_high > 255 : 
                v_high = 255
                

            self.color_lower_range = (h_low,100,100)
            self.color_higher_range = (h_high,255,255)
            
            if s < 100 :
                self.color_lower_range = (h_low,s_low,100)
                self.color_higher_range = (h_high,s_high,255)
            if v < 100 :
                self.color_lower_range = (h_low,100,v_low)
                self.color_higher_range = (h_high,255,v_high)
            if s<100 and v<100 : 
                self.color_lower_range = (h_low,s_low,v_low)
                self.color_higher_range = (h_high,s_high,v_high)

            self.mouse_clicked = True            


    def start(self) :
        while(True):
            available, self.frame = self.cap.read()
            
            self.frame = self.rescale_frame(self.frame, 50)
            if self.take_size_frame:
                (H, W) = self.frame.shape[:2]
                self.centerX = W // 2
                self.centerY = H // 2
                self.take_size_frame = False

            if not self.mouse_clicked : 
                cv2.imshow("Color Tracker",self.frame)   
            else :
                # median filter
                self.frame = cv2.medianBlur(self.frame,5)

                # convert to HSV  
                frame_hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)

                # create mask from selected color
                mask = cv2.inRange(frame_hsv, self.color_lower_range, self.color_higher_range)
                mask = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)

                # Gaussian blur 
                blur = cv2.GaussianBlur(mask,(3,3),0)
                gray = cv2.cvtColor(blur,cv2.COLOR_BGR2GRAY)
                
                # remove any small regions of noise
                thresh = cv2.threshold(gray, 45, 255, cv2.THRESH_BINARY)[1]
                thresh = cv2.erode(thresh, self.kernel1, iterations=2)
                thresh = cv2.dilate(thresh, self.kernel2, iterations=2)
                
                #edged = cv2.Canny(thresh, 30, 200)
                
                img2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                #cv2.drawContours(self.frame, contours, -1, (0,255,0), 3)
                areas = [cv2.contourArea(c) for c in contours]
                
                if len(areas) == 0:
                    cv2.rectangle(self.frame,(5,5),(20,20),(self.b,self.g,self.r),-1)
                    cv2.imshow("Color Tracker",self.frame)
                    if cv2.waitKey(10) == 27:
                        break
                    continue
                max_index = np.argmax(areas)
                cnt = contours[max_index]
                x,y,w,h = cv2.boundingRect(cnt)
                objX = round(((x + (w / 2.0) - self.centerX) / self.centerX), 3)
                objY = round(((y + (h / 2.0) - self.centerY) / self.centerY), 3)
                areaObj = round(((w * h) / (4.0 * (self.centerX * self.centerY))) * 100, 3)
                if(self.send_count == 1):
                    mess = ['CAMCO,', str(objX), ',', str(objY), ',', str(areaObj), '\n']
                    message = "".join(mess)
                    serialPort.write(message.encode('utf-8'))
                    self.send_count = 0
                self.send_count = self.send_count + 1
                cv2.rectangle(self.frame,(x,y),(x+w,y+h),(0,0,255),2)
                cv2.rectangle(self.frame,(5,5),(20,20),(self.b,self.g,self.r),-1)
                cv2.imshow("Color Tracker",self.frame)
            		
            if cv2.waitKey(10) == 27: # escape character (ESC)
                break

    def end(self) : 
        serialPort.close()
        self.cap.release()
        cv2.destroyAllWindows()

    def rescale_frame(self, frame, percent=75):
        width = int(frame.shape[1] * percent / 100)
        height = int(frame.shape[0] * percent / 100)
        dim = (width, height)
        return cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)


CT = ColorTracker()
CT.start()
CT.end()