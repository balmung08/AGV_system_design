#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# from https://github.com/amov-lab/Prometheus/wiki/将检测结果(ROS图像)推流到地面站

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from datetime import datetime
import subprocess
import os
# from mqtt_comm.msg import video_img


rospy.init_node('rtmp_stream', anonymous=True)

ip_address = rospy.get_param("~ip_address","rtmp://localhost:1935/live/test")
img_width = rospy.get_param("~img_width",1440)
img_height = rospy.get_param("~img_height",1080)
img_hz = rospy.get_param("~img_hz", 5)



size_str = str(img_width) + 'x' + str(img_height)
pipe_out_to_rtmp = subprocess.Popen(
    ['ffmpeg',
    '-y', '-an',
    '-f', 'rawvideo',
    '-vcodec','rawvideo',
    '-pix_fmt', 'bgr24',
    '-s', size_str,
    '-r', str(img_hz),
    '-i', '-',
    '-c:v', 'libx264',
    '-pix_fmt', 'yuv420p',
    '-preset', 'ultrafast',
    '-f', 'flv',
    ip_address],
    shell=False,
    stdin=subprocess.PIPE
)
create_flag=True
output_filename="/home/bit/record/save.mp4"
pipe_out_to_file = subprocess.Popen(
    ['ffmpeg',
    '-y',
    '-f', 'rawvideo',
    '-vcodec','rawvideo',
    '-pix_fmt', 'bgr24',
    '-s', size_str,
    '-r', str(img_hz),
    '-i', '-',
    '-c:v', 'libx264',
    '-pix_fmt', 'yuv420p',
    '-preset', 'ultrafast',
    '-f', 'flv',
    output_filename],
    shell=False,
    stdin=subprocess.PIPE
)
rec_push_flag=""
# p1=video_img()
# def stream_callback(msg):
#         p1.cmd=msg.cmd
#         print(p1.cmd)  
    # if p1.cmd==1:
    #     p1.image=msg.image
    #     p1.infrared=msg.infrared
    #     p1.sound=msg.sound

def image_callback(imgmsg):
    # global frame
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(imgmsg, "bgr8")
    frame = cv2.putText(frame,str(datetime.now()),(10,50),cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_AA)
    global rec_push_flag
    push_start_flag = False
    rec_push_flag = rospy.get_param("/work_state","work_done")

    # if p1.cmd==1:
    #     rec_push_flag="start_work"
    # else:
    #     rec_push_flag=""
    
    if rec_push_flag=="start_work" or rec_push_flag=="work_processing":
        push_start_flag=True
        rospy.set_param("/rtmp_stream_audio/push_flag",True)
    else:
        push_start_flag=False
        rospy.set_param("/rtmp_stream_audio/push_flag",False) 


    # push_flag = rospy.get_param("~push_flag",True)
    camera_type = rospy.get_param("~camera_type","rgb")
    display = rospy.get_param("~display","1")
    global create_flag
    global pipe_out_to_file
    # global push_flag
    if push_start_flag:
        pipe_out_to_rtmp.stdin.write(frame.tostring())
        if create_flag:
            # 获取当前日期
            current_date = datetime.now()
            # 将日期转换为字符串
            date_string = current_date.strftime("%Y-%m-%d-%H-%M-%S")
            dt_string = current_date.strftime("%Y-%m-%d-%H-%M")
            print(date_string)

            # 指定目录路径
            path = rospy.get_param("/save_file_path","/home/bit")  #"/home/bit/record/"

            # 要检查的文件夹名称
            folder_name =dt_string

            # 检查文件夹是否已存在
            if os.path.exists(path):
                print("文件夹已存在！")
            else:
                # 使用 os.mkdir() 创建文件夹
                os.makedirs(path)
                print("文件夹创建成功！")
           
            output_filename=path + "/" +camera_type+"_video.mp4" #folder_name+"/"+date_string+camera_type+".mp4"
            print(output_filename)
            pipe_out_to_file = subprocess.Popen(
                ['ffmpeg',
                '-y',
                '-f', 'rawvideo',
                '-vcodec','rawvideo',
                '-pix_fmt', 'bgr24',
                '-s', size_str,
                '-r', str(img_hz),
                '-i', '-',
                '-c:v', 'libx264',
                '-pix_fmt', 'yuv420p',
                '-preset', 'ultrafast',
                '-f', 'flv',
                output_filename],
                shell=False,
                stdin=subprocess.PIPE
            )
            create_flag=False
        else:
            pipe_out_to_file.stdin.write(frame.tostring())
    else:
        create_flag=True
        
        pipe_out_to_file.terminate()

    if display==1:   
        cv2.imshow("cap", frame)
        cv2.waitKey(10)

if __name__ == '__main__':
    topic_name = rospy.get_param('~img_topic', '/show_img_node/image')
    rospy.Subscriber(topic_name, Image, image_callback)

    # stream_sub=rospy.Subscriber("/stream_cmd",video_img,stream_callback,queue_size=10)
    rospy.set_param("/save_file_path","/home/bit/save_data/A001/BBB")

    rospy.spin()

    pipe_out_to_rtmp.terminate()

    
    