import os
import sys
import cv2 
import math 
import glob
import imutils
import numpy as np
from time import sleep, strftime, time
#from yoloDet import YoloTRT

from ultralytics import YOLO

current = os.path.dirname(os.path.realpath(__file__))
parent_directory = os.path.dirname(current)
  
sys.path.append(parent_directory)

from siyi_sdk import SIYISDK
from pymavlink import mavutil


os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = 'rtsp_transport;udp'

# model = YoloTRT(library="yolov5/build/libmyplugins.so", engine="yolov5/build/yolov5s.engine", conf=0.7, yolo_ver="v5")
model = YOLO("yolov8s.engine", task="detect")
print("Model Loaded")

fov = 81
yaw_corrected = 0
yaw_speed = 10
vx = 0 # velcity in x-axis (ms)
vy = 0
vz = 0
speed = 1
Gimble_align = False
Output_Folder = "/home/nvidia/code/Output_Frames"
Object_id = 2 # 41 for Cup, 2 for Car


log_folder = 'log'
gimbalExtLogs = glob.glob('%s/gimbLog_*.txt'%(log_folder))
gimbLogFile = '%s/gimbLog_%04d.txt'%(log_folder, len(gimbalExtLogs)+1)
print('Saving camera logs to: ', gimbLogFile) 

# Write Header of gimble log file
with open(gimbLogFile, 'w') as f:
    f.write('time_stamp\tgimb_yaw\tgimb_pitch\tdrone_pitch\tvx\tvz\n')


det_counter = 0
bbox_list = [0,0,0,0]
init_tracker = False
tracker = False
tolerance = 5
target_hit = False


frameref = 16
yaw_offset = 0
pitch_offset = -25

# Function to send gimbal control command
def send_gimbal_control(pitch, yaw, frameref, pitchrate, yawrate):
    the_connection.mav.command_long_send(
        the_connection.target_system,     		# target_system
        the_connection.target_component,  		# target_component
        mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,  	# command
        0,               # confirmation
        pitch,			# param1 (pitch in degrees)
        yaw,             # param2 (yaw in degrees)
        pitchrate,		# param3 (pitchrate in degrees/sec)
        yawrate,         # param4 (yawrate in degrees/sec)
        frameref, 	     # param4 (gimbal mananger flag) 16 for lock/earth frame 32 is original value 
        0,			# param6 (not used)
        0                # param7 (gimbal id)
    )

def getAttitude(cam): # To get gimble/camera yaw and pitch
    #sleep(1)
    yaw, pitch, roll = cam.getAttitude()
    return yaw, pitch

def read_yaw(): # Getting current heading /yaw of bird
    the_connection.wait_heartbeat()
    msg = the_connection.recv_match(type='HEARTBEAT', blocking=True)
    if msg:
        mode = mavutil.mode_string_v10(msg)
        heading = the_connection.messages['GLOBAL_POSITION_INT'].hdg/100
    print('Current heading: ', heading)
    return heading

def get_new_yaw(gim_yaw): # calculating bird yaw with respect to gimble
    heading = read_yaw()
    if gim_yaw < 0:
        new_heading= heading + abs(gim_yaw)
    elif gim_yaw > 0:
        new_heading = heading - gim_yaw
    elif gim_yaw == 0:
        new_heading = heading
    yaw = new_heading % 360
    return yaw

def control_yaw( yaw, yaw_speed): # Changing bird yaw
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, yaw, yaw_speed, 0 , 0, 0, 0, 0)
    # heading = read_yaw()
    # #print('Now drone heading: ', heading)

def get_rel_alt(connection):  # Getting relative altitude of bird
    msg = the_connection.recv_match(type='HEARTBEAT', blocking=True)
    if msg:
        mode = mavutil.mode_string_v10(msg)
        Rel_alt_vec = the_connection.messages['GLOBAL_POSITION_INT'].relative_alt/1000

    return Rel_alt_vec

def x_position(the_connection): # Getting x postion of bird
    msg = the_connection.recv_match(type='HEARTBEAT', blocking=False)
    x_position =  the_connection.messages['LOCAL_POSITION_NED'].x
    return x_position

def goto_LOS(prev_pitch):

    Current_alt = get_rel_alt(the_connection)

    # Get pitch from drone
    msg = the_connection.recv_match(type='HEARTBEAT', blocking=False)
    drone_pitch = the_connection.messages['ATTITUDE'].pitch
    #print('Drone pitch: ', drone_pitch)  
    print("Curent alt", Current_alt)
           
    #LOS = atan((Current_alt)/(target_x-x_position))
    LOS =  abs(prev_pitch)
    #print("LOS :", LOS)
    vx = speed * (math.cos(math.radians(LOS)))
    vz = speed * (math.sin(math.radians(LOS)))
           
    print('Speed_x :', vx)
    print('Speed_z :', vz)
    if Current_alt > 10.0:
        the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,the_connection.target_system,the_connection.target_component,
                                                                                              mavutil.mavlink.MAV_FRAME_BODY_NED,
                                                                                              int(0b100111000111),0,0,0,abs(vx),vy,abs(vz),0,0,0,0,0))
        #print("Moving to target in :", (time.time()-start))
        print("Moving to target")
        timestamp = strftime("%Y%m%d_%H%M%S")
        with open(gimbLogFile, 'a+') as f:
            f.write('%s\t%0.2f\t%0.2f\t%0.5f\t%.5f\t%0.5f\n'%(timestamp, cam_yaw, cam_pitch, drone_pitch, vx, vz))
            #sleep(1)
    else:
        print("Target Hit")
            #x_position =  x_position(the_connection)
            # get altitude
        the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,the_connection.target_system,the_connection.target_component,mavutil.mavlink.MAV_FRAME_BODY_NED, int(0b100111000111),0,0,0,0,0,0,0,0,0,0,0))
        
        #print(f"Target Hit X position is {x_position}")
        timestamp = strftime("%Y%m%d_%H%M%S")
        with open(gimbLogFile, 'a+') as f:
            f.write('%s\t%0.2f\t%0.2f\t%0.5f\t%.5f\t%0.5f\n'%(timestamp, cam_yaw, cam_pitch, drone_pitch, vx, vz))
        target_hit = True

# ---------------------------------------------------- Main Code -------------------------------------------------------------------------

# Start a connection listening to a UDP port
the_connection = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600)
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

# Creating a camera connection
cam = SIYISDK(server_ip="192.168.1.23", port=37260)
sleep(2)
if not cam.connect():
   print("No connection ")
   exit(1)
   
sleep(1)
#cam.setGimbalRotation(0, 0,1,4)
#send_gimbal_control(0, 0, 5, 5)
send_gimbal_control(pitch_offset, yaw_offset,frameref, 5, 5)
sleep(2)
cam_yaw, pitch = getAttitude(cam)
new_yaw = get_new_yaw(cam_yaw)
print("--------------------------------Gimbal Yaw :",cam_yaw)
print("--------------------------------Drone New Heading :", new_yaw)
#gst_str = (
#        'rtspsrc location=rtsp://192.168.1.31:8554/main.264 ! '
#        'decodebin ! videoconvert ! appsink')

gst_str = (
		'rtspsrc location=rtsp://192.168.1.23:8554/main.264 latency =0 ! rtph265depay ! h265parse ! avdec_h265 ! videoconvert ! appsink drop=True max-buffers=1')
		#'rtspsrc location=rtsp://192.168.1.23:8554/main.264 latency = 0 ! rtph265depay ! h265parse ! nvv4l2decoder ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! appsink drop=True max-buffers=1')

# Create a VideoCapture object with GStreamer pipeline
cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
sleep(2)

if not cap.isOpened():
    print("Error: Unable to open RTSP stream. SYSTEM EXIT")
    sys.exit()

print("Gimble attitude ##############", cam.getAttitude())

#sleep(3)

frame_width = int(cap.get(3)) 
frame_height = int(cap.get(4)) 
   
size = (640, 360) 
result = cv2.VideoWriter('result.avi',  
                         cv2.VideoWriter_fourcc(*'MJPG'), 
                         25, size) 
                                            

cam_yaw, cam_pitch = getAttitude(cam) # Getting Cam yaw and pitch
print("Bird yaw",get_new_yaw(cam_yaw))


print('cam_yaw', cam_yaw)
'''timestamp = strftime("%Y%m%d_%H%M%S")
with open(gimbLogFile, 'a+') as f:
    f.write('%s\t%0.2f\t%0.2f\n'%(timestamp, cam_yaw, cam_pitch))'''


#new_yaw = get_new_yaw(cam_yaw) # getting new heading for bird
prev_yaw = cam_yaw
prev_pitch = cam_pitch
#print('Moving drone to: ', new_yaw)
#control_yaw(new_yaw, yaw_speed) # Nove drone to new heading
#sleep(4)

#print("$$$$$$$$$$$$$$$$$$$$$$$$$ YAW CORRECTED WRT GIMBAL $$$$$$$$$$$$$$$$$$$$$$")


car_count = 0
while True:

    if target_hit:
       break
       
    grabbed=cap.grab()
    ret, frame = cap.retrieve()
    print(ret)
    start_time = time()
    
    if ret:
        timestamp = strftime("%Y%m%d_%H%M%S")

        frame = imutils.resize(frame, width=640)
        height, width, _ = frame.shape
        x, y = 640/2, 360/2
        # detections, t = model.Inference(frame)
#        detections = model(frame, save=True)
        detections = model.predict(frame, save= True)

        if detections:
            print("Length of detections:", len(detections))
            detc = detections[0]
#            print("Result:",detc)
#            print("len",len(detc.boxes))
            
#            print(detections)
#            breakpoint()
            try:    
                for obj in detc.boxes: 
                    class_id = obj.cls.item()
                    print("class id:", class_id)
                    
#                    breakpoint()
                    
                  
#                    if obj['class'] == 'persons':
                    if int(class_id) == Object_id : #car=2
                        print("Class id Found")
                        box = obj.xyxy.tolist()

                        x1 = int(box[0][0]) #- box[0][2]/2)
                        y1 = int(box[0][1]) #- box[0][3]/2)
                        x2 = int(box[0][2]) #+ box[0][2]/2)
                        y2 = int(box[0][3]) #+ box[0][3]/2)
                        color = (0,0,255) 
                        cv2.rectangle(frame, (x1,y1),(x2,y2),color,2)
                        cv2.putText(frame, str(class_id),(x1,y1-10), cv2.FONT_HERSHEY_SIMPLEX, 2, color,2)

                        # Yolo center 
                        obj_x, obj_y = (x1+ (x2 - x1)/2), (y1 + (y2- y1)/2)

                        yolo_box_center = (obj_x,obj_y)
                        center_difference = math.sqrt((yolo_box_center[0] - x)**2 + ( yolo_box_center[1] - y)**2)
                        print("center difference", center_difference)
                        delta_x = obj_x - x
                        delta_y = obj_y - y
                        # pixel to yaw and pitch mapping
                        gim_yaw = math.degrees(math.atan(delta_x / (frame.shape[0] / (2 * math.tan(math.radians(fov / 2))))))
                        gim_pitch = math.degrees(math.atan(delta_y / (frame.shape[1] / (2 * math.tan(math.radians(fov / 2))))))                   
                        car_count = car_count + 1
                        #print("gim_yaw :",gim_yaw, gim_pitch)  
                        print("Car Count: ", car_count)
                        image_name = timestamp + ".jpg"
                        folder = os.path.join(Output_Folder, image_name)
                        #print("Image folder :", folder)
                        

                        
                        if Gimble_align == False :

                            if center_difference <= 65 and car_count >=7:
                            	  
                                # Lock Gimbal
                                #yaw_offset = yaw_offset + gim_yaw
                                #pitch_offset =  pitch_offset - gim_pitch 
                                cam_yaw, cam_pitch = getAttitude(cam)
                                new_yaw = get_new_yaw(cam_yaw)
                                print(f'************************************yaw : {cam_yaw} , pitch : {cam_pitch}')
                                print(f'************************************New Yaw Bird : {new_yaw} ')
                                control_yaw(new_yaw, yaw_speed)
                                sleep(4)
                                
#                                prev_new_yaw = new_yaw
                                prev_pitch = pitch_offset  #todo check if prev and offset pitch is same
                                prev_yaw = cam_yaw
                                print("################################## Offset pitch is = ", pitch_offset)
                                print("################################## After heading correction Pitch = ",prev_pitch)
                                print("################################## Offset yaw is = ", yaw_offset)
                                print("################################## After heading correction yaw = ",prev_yaw)
                                Gimble_align = True
                            else:
                                yaw_offset = yaw_offset + gim_yaw
                                pitch_offset =  pitch_offset - gim_pitch
                                print("Pitch offset :", pitch_offset)
                                send_gimbal_control(pitch_offset, yaw_offset,frameref, 5, 5)
                                sleep(1)
                                #print("Gimble is align is True ******************************************************")
                            # follow gimbal 
                        #frame = cv2.rectangle(frame, obj_x, obj_y)
#                        result.write(frame)
                        cv2.imwrite( folder, frame)
                        for _ in range(3):
                            result.write(frame)
                        
                        print("Frame processing time after detections is ", time()-start_time) 
    
                        timestamp = strftime("%Y%m%d_%H%M%S")
                        with open(gimbLogFile, 'a+') as f:
                            f.write('%s\t%0.2f\t%0.2f\n'%(timestamp, cam_yaw, cam_pitch))
                            if abs(prev_pitch)-abs(gim_pitch) >= 3 and Gimble_align == True:
#                                cam.setRotation(gim_yaw, gim_pitch)
                                pitch_offset =  pitch_offset - gim_pitch
                                print("****************************** Pitch offset after 3 degree differece :", pitch_offset)
                                send_gimbal_control(pitch_offset, yaw_offset,frameref, 5, 5)
                                sleep(1)
                                prev_pitch = pitch_offset
                            cam_yaw, cam_pitch = getAttitude(cam)
                            if abs(cam_yaw) >= 7 and Gimble_align == True:
                                cam_yaw, cam_pitch = getAttitude(cam)
                                new_yaw = get_new_yaw(cam_yaw)
                                print(f'************************************yaw : {cam_yaw} , pitch : {cam_pitch}')
                                print(f'************************************New Yaw Bird : {new_yaw} ')
                                control_yaw(new_yaw, yaw_speed)
                                sleep(1)                                                            
                            
#                    cv2.imshow("detections", frame)
#                    cv2.imwrite( timestamp + ".jpg", frame)
                    else:
                    	print("Class id not found")
                    	cv2.imwrite( folder, frame)
                    	result.write(frame)
#                    for _ in range(3):
#                        result.write(frame)
                     
            except Exception as e:
#                cv2.imshow("detections", frame)   
                image_name = timestamp + ".jpg"
                folder = os.path.join(Output_Folder, image_name)
                cv2.imwrite( folder, frame) 
                result.write(frame) 
                print("Exception in detections")  
                print("No Detection! Video Write Continue")              
        else:
#            cv2.imshow("detections", frame)
#            cv2.imwrite( timestamp + ".jpg", frame)
            image_name = timestamp + ".jpg"
            folder = os.path.join(Output_Folder, image_name)
            cv2.imwrite( folder, frame)
            result.write(frame)  
            print("No Detection! Video Write Continue")                 



# Drone Movement Towards LOS
        
    if car_count > 1 and Gimble_align == True:
#         start = time.time()
         goto_LOS(prev_pitch)
#         print(time.time() - start)
         timestamp = strftime("%Y%m%d_%H%M%S")
         with open(gimbLogFile, 'a+') as f:
             f.write('%s\t%0.2f\t%0.2f\n'%(timestamp, cam_yaw, cam_pitch))
           
print("STOP")

cap.set(cv2.CAP_GSTREAMER, 0)
cap.release()
result.release()

#cv2.destroyAllWindows()