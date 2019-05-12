## 在YOLOv3原仓库基础上修改了部分文件。由于原仓库太大，这里只添加了有改动的文件

* 修改了yolov3.cfg，减小了网络输入尺寸，使检测速度更快。
* 原仓库给出了检测单张图片的例子，读入的时图片路径，但当处理一个视频时不可能把每帧先保存在处理，因此需要修改。
  修改了image.c image.h darknet.py Makefile并重新编译。
* 添加了修改图片读入方式后的detector_for_one_single_pic.py，可用于检测单张图片
* 添加了detector_for_video.py可用于检测一个视频
* 添加了detector_for_webcame.py可用于使用摄像头的检测

修改了以上文件后可杂python程序中直接调用yolov3，并基本能保持原来的速度(使用TX2帧率在5帧每秒左右)
