import cv2
import numpy as np
image=cv2.imread("/home/ubuntu/nemo_ws/src/path_finder/temp_workspace/image.jpg")
image=cv2.resize(image,(864,648))
coordinates=[]
cv2.imshow("Destination",image)
def click_event(event, x, y, flags, params):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(x, ' ', y)
        coordinates.append([x,y])
cv2.setMouseCallback('Destination', click_event)
cv2.waitKey(0)
cv2.destroyAllWindows()
np.save("/home/ubuntu/nemo_ws/src/path_finder/temp_workspace/target_coordinates.npy",coordinates[-1])
c=np.load("/home/ubuntu/nemo_ws/src/path_finder/temp_workspace/target_coordinates.npy")
print(c)
