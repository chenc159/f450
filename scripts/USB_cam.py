
from cv_bridge import CvBridge
import  numpy  as  np  # 引入numpy 用於矩陣運算
import  cv2
cap  =  cv2.VideoCapture(1)
print ("Open the camera?\n{}".format(cap.isOpened ()))
img_count =  0
# out = cv2.VideoWriter('usb_cam.mp4', 0x00000021, 30, (640, 480))
while(True):
    ret,frame = cap.read()
    if not ret:
        print ("Failure")
        break
    print(frame.shape)
    # cv2.imshow ('image',frame)
    # key = cv2.waitKey(1)
    # if key == ord('q'):
    #     break
    img_count += 1
    cv2.imwrite('image_{}.png'.format(img_count), frame)
    # out.write(frame)
# out.release()
cap.release()
cv2.destroyAllWindows()


