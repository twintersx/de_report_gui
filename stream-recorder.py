import cv2
from datetime import datetime, timedelta

cap = cv2.VideoCapture(0)   # change to 1 for input

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')

t0 = datetime.now()
tf = t0 + timedelta(seconds=5)
dt_string = t0.strftime("%Y%m%d_%H%M%S")
out = cv2.VideoWriter(f'{dt_string}.avi', fourcc, 20.0, (640, 480))

while(datetime.now() < tf):
    # ret checks return at each frame
    ret, frame = cap.read()

    # output the frame
    out.write(frame)

    # The original input frame is shown in the window 
    cv2.imshow('Original', frame)

    # Wait for 'q' key to stop the program 
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cap.release()               # Close the window / Release webcam
out.release()               # After we release our webcam, we also release the output
cv2.destroyAllWindows()     # De-allocate any associated memory usage