import cv2
from datetime import datetime, timedelta
import imageio 

cap = cv2.VideoCapture(1)   # change to 1 for input

t0 = datetime.now()
tf = t0 + timedelta(seconds=10)
dt_string = t0.strftime("%Y%m%d_%H%M%S")

frames = []
while(datetime.now() < tf):
    # ret checks return at each frame
    ret, frame = cap.read()

    # --- save frames for later --- #
    frames.append(frame)

    # The original input frame is shown in the window 
    cv2.imshow('Racelogic Feed', frame)

    # pressing 'a' simulates a new DE during recording 
    if cv2.waitKey(1) & 0xFF == ord('a'):
        tf = tf + timedelta(seconds=10)

with imageio.get_writer(f'{dt_string}.gif', mode='I', fps = 5) as writer:
    for i in range(0, len(frames), 10):
        rgb_frame = cv2.cvtColor(frames[i], cv2.COLOR_BGR2RGB)
        writer.append_data(rgb_frame)

cap.release()               # Close the window / Release webcam
cv2.destroyAllWindows()     # De-allocate any associated memory usage