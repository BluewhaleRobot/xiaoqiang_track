# xiaoqiang_track

xiaoqiang_track是一个利用摄像头捕捉的图像进行人体追踪的程序。它有比较好的稳定性,能够可靠的追踪目标。同时也具有比较好的扩展性。可以方便的对关键的追踪算法进行调整。在运动的控制上，使用了PID控制，可以通过调整参数适应不同的设备。

## 原理简介

通过人体识别算法获取到人体的上半身位置。获取成功后就把对应的位置信息传递给追踪程序。追踪程序进行目标追踪。运动控制程序根据当前的目标的位置计算运动控制量。
由于在追踪过程中肯能会丢失，丢失的情况下就要再次使用人体识别程序的结果进行追踪程序的初始化。

## 安装

以下安装方法只在ubuntu 16.04 kinetic版本测试过，如果需要在其他版本使用可能需要调整部分指令。

安装相关依赖程序

```bash
sudo apt-get install libjsoncpp libcurl3 libcurl4-openssl-dev openssl
```

安装body_pose人体识别程序

详细安装方法请参照 [body_pose使用说明](https://github.com/bluewhalerobot/body_pose)

可选程序

本程序有语音提示功能，如果想要这个功能可以安装[xiaoqiang_tts](https://github.com/bluewhalerobot/xiaoqiang_tts)软件包

## 运行要求

由于使用的是摄像头追踪方法，所以首先要有一个摄像头设备。同时此设备能够通过ROS的软件包把图像信息通过topic形式发送出来。
对于小强用户，下面的运行指令直接执行就可以。对于其他用户请修改launch文件中的usb_cam节点。替换成自己的摄像头节点。

由于要进行人体识别，摄像头的视野范围也很重要。要能保证在合适的追踪距离下摄像头能够比较完整的看到追踪目标。
对于小强用户，把摄像头固定在二层上，同时有一定的仰角就可以了。
注意图像的分辨率是640x480

## 运行

小强用户请先关闭startup服务

```bash
sudo service startup stop
```

启动roscore

```bash
roscore
```

启动底盘驱动程序

```bash
# 这里是小强的驱动程序，如果不是小强，需要换成自己的
roslaunch xqserial_server xqserial.launch
```

开始运行追踪程序

```bash
# 使用body_pose人体识别程序进行人体识别
roslaunch xiaoqiang_track track_body.launch
# 或者使用百度的人体识别服务
roslaunch xiaoqiang_track track_baidu.launch
```

此时站在摄像头前面，等待初始化完成就可以开始追踪了。注意在转弯的时候适当放慢速度。小车跟不上的话可能会导致追踪丢失。

如果想要实时的看到处理结果可以订阅/xiaoqiang_track/processed_image图像话题。其中的方框即为实时的追踪结果。

![0_1531101664469_ecc18945-34c8-4ceb-90be-1c954f622d49-image.png](https://community.bwbot.org/assets/uploads/files/1531101683318-ecc18945-34c8-4ceb-90be-1c954f622d49-image.png)


## 参数说明

详细参数请参照launch文件中的注释。
