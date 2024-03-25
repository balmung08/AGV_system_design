import os
so_files = os.listdir("src/camera_lbas/lib")
so_files.sort()
so_names = [file.split(".")[0] for file in so_files]

with open("src/camera_lbas/lib_names.txt","w")as f:
    for name in so_names:
        f.write(name[3:]+'\n')