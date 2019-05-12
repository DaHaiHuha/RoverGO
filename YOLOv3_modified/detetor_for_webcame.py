import sys, os, cv2
import time
import numpy as np

video_path = '/home/nvidia/Downloads/output.avi'
sys.path.append(os.path.join(os.getcwd(),'python/'))

import darknet as dn
import pdb

font = cv2.FONT_HERSHEY_COMPLEX_SMALL
def showPicResult(image):
    #img = cv2.imread(image)
    for i in range(len(r)):
        x1=r[i][2][0]-r[i][2][2]/2
        y1=r[i][2][1]-r[i][2][3]/2
        x2=r[i][2][0]+r[i][2][2]/2
        y2=r[i][2][1]+r[i][2][3]/2
        #img = cv2.imread(image)
        cv2.rectangle(image,(int(x1),int(y1)),(int(x2),int(y2)),(0,255,0),3)
        cv2.putText(image,r[i][0],(int(x1),int(y1)),font,0.8,(0,0,0))
        #This is a method that works well.
        #cv2.imwrite(out_img, im)
    cv2.imshow('yolo_image_detector', image)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

net = dn.load_net(str.encode("cfg/yolov3.cfg"),
                  str.encode("yolov3.weights"), 0)
meta = dn.load_meta(str.encode("cfg/coco.data"))

#print rq
cap = cv2.VideoCapture(0)
while(cap.isOpened()):
    ret,frame = cap.read()
    #cv2.imshow("lalala",frame)
    start = time.time()
    image = dn.nparray_to_image(frame)
    r = dn.detect(net, meta, image)
    showPicResult(frame)
    end = time.time()
    print r
    #fps = cap.get(cv2.CAP_PROP_FPS)
    seconds = end - start
    fps = 1 / seconds
    print "fps: ", fps
    for i in range(len(r)):
        if 'person' in r[i]:
            print "ALERT!"
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

#cv2.release()
cv2.destroyAllWindows()