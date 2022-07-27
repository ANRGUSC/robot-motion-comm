import cv2
import os
import numpy as np
import time


# return associated binary character for spot on the grid
def return_bin(x, prev_x, prev_char):
    if x>prev_x+50:
        return '1'
    elif x<prev_x-50:
        return '0'
    else:
        return prev_char
    return None

beg = time.time()

# stop sign classifier
stop_data = cv2.CascadeClassifier('stop_data.xml')

vid = cv2.VideoCapture('hello.mov')
fps = vid.get(cv2.CAP_PROP_FPS)
print("FPS:", fps)

frame_no = 0
message = ''

# create folder "data" to store the frames
try:
    if not os.path.exists('data'):
        os.makedirs('data')
  
# if not created then raise error
except OSError:
    print ('Error: Creating directory of data')

prev_x_bin = 0
prev_x = 0 
# start position at character 0
prev_char = '0'
start_flag = False
collect_flag = False
stop_flag = False
char_count = 0

while(vid.isOpened()):
    # read in frames from video
    success, image = vid.read()
    if success:
        if frame_no%15==0:
            
            img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            ms = vid.get(cv2.CAP_PROP_POS_MSEC)
            
            found = stop_data.detectMultiScale(img_gray, minSize =(20, 20))
            print("frame : " ,frame_no, "   timestamp: ", np.round(ms/1000, 2))
            cv2.imwrite('./data/frame' + str(frame_no) + '.jpg', image)
            
            # green rectangle around found stop signs
            amount_found = len(found)
            if amount_found != 0:
                for (x, y, width, height) in found:
                    if frame_no == 0:
                        prev_x = x
                        prev_y = y
            
            # CHECK FOR START MESSAGE
            if x <= prev_x-50 and not start_flag:
                start_flag = True
                start_ms = ms
                start_frame = frame_no
                print("START")
               
            # CHECK FOR FIRST CHARACTER 
            if start_flag and frame_no == start_frame+30:
                first_x = x
                
            # CHECK FOR STOP MESSAGE
            if y >= prev_y+50:
                print("STOP")
                stop_flag = True     
            if stop_flag:
                break
                
            # AFTER START MESSAGE, START COLLECTING DATA
            # 30 frames is the interval between signals (1 sec) 
            # + 15 frames to get the middle of the signal, avoiding errors
            if start_flag and frame_no == start_frame+45:
                collect_flag = True
                collect_frame = frame_no
                
            # COLLECT DATA AT RATE THAT MATCHES TRANSMISSION RATE
            # 30 frames is the interval between signals (1 sec)
            if collect_flag and frame_no == collect_frame+(30*char_count):
                if char_count == 0:
                    if x-50 < first_x < x+50: 
                        char = '0'
                    else:
                        char = '1'
                else:
                    char = return_bin(x, prev_x_bin, prev_char)
                if char != None:
                    message+=char
                prev_char = char
                prev_x_bin = x
                char_count +=1
            prev_x = x
            prev_y = y
            
    else:
        break
    frame_no += 1

vid.release()

msg = ''
msg1 = ''

msg_again1 = ''
msg_again = ''

print("BINARY", message)
for i in range(0, len(message)):
    msg_again1+=message[i]
    if (i+1)%8==0:
        msg_again+=chr(int(msg_again1, 2))
        msg_again1 = ''

print("MESSAGE", msg_again)
print("CHARACTER COUNT", char_count/8)
print("TRANSMISSION RATE 1 signal/sec")
end = time.time()
print("RENDER TIME", end-beg, "secs")
    