#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# from https://zulko.github.io/blog/2013/10/04/read-and-write-audio-files-in-python-using-ffmpeg/

import rospy
import subprocess
import threading
from datetime import datetime
import os

rospy.init_node('rtmp_stream', anonymous=True)
freq="44100"
pipe_in = subprocess.Popen(
    ['ffmpeg',
    # '-re',
    # '-i','/home/cloude/test.mp3',
    '-f', 'alsa',
    '-i', 'sysdefault:CARD=Device',
    '-f', 's16le',
    '-acodec','pcm_s16le',
    '-ar',freq,
    '-ac','1',
    '-'],
    shell=False,
    
    stdout=subprocess.PIPE,
    bufsize=2
)


pipe_out_to_rtmp = subprocess.Popen(
    ['ffmpeg',
    '-y', # (optional) means overwrite the output file if it already exists.
    "-f", 's16le', # means 16bit input
    "-acodec", "pcm_s16le", # means raw 16bit input
    '-r', freq, # the input will have 48000 Hz
    '-ac','1', # the input will have 2 channels (stereo)
    '-i', '-', # means that the input will arrive from the pipe
    '-vn', # means "don't expect any video input"
    '-f', 'flv',
    # '-b', "3000k", # output bitrate (=quality). Here, 3000kb/second
    "rtmp://localhost:1935/live/audio"],
    shell=False,
    stdin=subprocess.PIPE
)
create_flag=True
output_filename="/home/bit/record/save_48000.wav"
pipe_out_to_file = subprocess.Popen(
    ['ffmpeg',
    '-y', # (optional) means overwrite the output file if it already exists.
    "-f", 's16le', # means 16bit input
    "-acodec", "pcm_s16le", # means raw 16bit input
    '-r', freq, # the input will have 48000 Hz
    '-ac','1', # the input will have 2 channels (stereo)
    '-i', '-', # means that the input will arrive from the pipe
    '-vn', # means "don't expect any video input"
    '-f', 'flv',
    # '-b', "3000k", # output bitrate (=quality). Here, 3000kb/second
    output_filename],
    shell=False,
    stdin=subprocess.PIPE
)

push_flag = rospy.get_param("~push_flag",True)

def get_push_flag():
    while not rospy.is_shutdown():
        rospy.sleep(1)
        global push_flag
        push_flag = rospy.get_param("~push_flag",True)
        

if __name__ == '__main__':
    thread = threading.Thread(target=get_push_flag)

    # rospy.set_param("/save_file_path","/home/bit/save_data/A001/BBB")


    thread.start()
    while not rospy.is_shutdown():
        raw_audio = pipe_in.stdout.read(2)

        if push_flag: 
                      
            pipe_out_to_rtmp.stdin.write(raw_audio)

            if create_flag:
                print(push_flag) 
                # 获取当前日期
                current_date = datetime.now()
                # 将日期转换为字符串
                date_string = current_date.strftime("%Y-%m-%d-%H-%M-%S")

                dt_string = current_date.strftime("%Y-%m-%d-%H-%M")
                print(date_string)

                # 指定目录路径
                # path = "/home/bit/record/"
                path=rospy.get_param("/save_file_path")

                # 要检查的文件夹名称
                folder_name =dt_string

                # 检查文件夹是否已存在
                if os.path.exists(path):
                    print("文件夹已存在！")
                else:
                    # 使用 os.mkdir() 创建文件夹
                    os.makedirs(path)
                    print("文件夹创建成功！")
            
                # output_filename=path + folder_name+"/"+date_string+"audio.wav"
                output_filename=path+"/"+"audio.wav"
                # print(date_string)
                print(output_filename)
                pipe_out_to_file = subprocess.Popen(
                    ['ffmpeg',
                    '-y', # (optional) means overwrite the output file if it already exists.
                    "-f", 's16le', # means 16bit input
                    "-acodec", "pcm_s16le", # means raw 16bit input
                    '-r', freq, # 48000 the input will have 48000 Hz
                    '-ac','1', # the input will have 2 channels (stereo)
                    '-i', '-', # means that the input will arrive from the pipe
                    '-vn', # means "don't expect any video input"
                    '-f', 'flv',
                    # '-b', "3000k", # output bitrate (=quality). Here, 3000kb/second
                    output_filename],
                    shell=False,
                    stdin=subprocess.PIPE
                )
                create_flag=False
            else:
                pipe_out_to_file.stdin.write(raw_audio)
                # print("write!!!!")            

        else:
            create_flag=True 
            pipe_out_to_file.terminate() 
            # print("close")          
            
    pipe_in.terminate()
    pipe_out_to_rtmp.terminate()
    
    thread.join()