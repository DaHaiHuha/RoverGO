# Stupid python path shit.
# Instead just add darknet.py to somewhere in your python path
# OK actually that might not be a great idea, idk, work in progress
# Use at your own risk. or don't, i don't care

import sys, os, cv2

sys.path.append(os.path.join(os.getcwd(),'python/'))

import darknet as dn
import pdb

origin_img = "data/dog.jpg"
out_img = "data/dog_test.jpg"

def showPicResult(image):
    img = cv2.imread(image)
    cv2.imwrite(out_img, img)
    for i in range(len(r)):
        x1=r[i][2][0]-r[i][2][2]/2
        y1=r[i][2][1]-r[i][2][3]/2
        x2=r[i][2][0]+r[i][2][2]/2
        y2=r[i][2][1]+r[i][2][3]/2
        im = cv2.imread(out_img)
        cv2.rectangle(im,(int(x1),int(y1)),(int(x2),int(y2)),(0,255,0),3)
        #This is a method that works well.
        cv2.imwrite(out_img, im)
    cv2.imshow('yolo_image_detector', cv2.imread(out_img))
    cv2.waitKey(0)
    cv2.destroyAllWindows()

net = dn.load_net(str.encode("cfg/yolov3.cfg"),
                  str.encode("yolov3.weights"), 0)
meta = dn.load_meta(str.encode("cfg/coco.data"))
image = dn.nparray_to_image(cv2.imread(origin_img))
r = dn.detect(net, meta, image)
#r = dn.detect(net, meta, "data/dog.jpg")
print r
showPicResult(origin_img)



