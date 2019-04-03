import math
import numpy as np
import cv2

def distance(point1, point2):
 	return math.sqrt((point1[0]-point2[0])**2+(point1[1]-point2[1])**2)
def angle(point,width,height):
	if point[0]<320 and point[1]<240: 
		return math.degrees(math.atan((point[1]-height/2)/(point[0]-width/2)))-180
	elif point[0]<320 and point[1]>240:
		return math.degrees(math.atan((point[1]-height/2)/(point[0]-width/2)))+180
	elif point[0]==320 and point[1]<240:
		return -90
	elif point[0]==320 and point[1]>240:
		return 90	
	else:
		return math.degrees(math.atan((point[1]-height/2)/(point[0]-width/2)))	 	 	

cap = cv2.VideoCapture(0)
width = cap.get(3)
height = cap.get(4)
print(width,height) 
point1 = (0,220)
point2 = (width/2, height/2)
print(angle(point1,width,height)) 




	