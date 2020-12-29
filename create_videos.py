import skvideo.io
import skimage.io
import numpy as np
import os

fname = 'High_Res_'
base_dir = '/home/gradandpostdoc/carla_autonomous/_out'


# directory = base_dir+'/pedview'
# ped = skvideo.io.FFmpegWriter(base_dir+"/{}_Pedestrian.mp4".format(fname), outputdict={
#   '-vcodec': 'libx264', '-b': '300000000','-r':'30'
# })
#
# for frame_no,entry in enumerate(sorted(os.listdir(directory))):
#     frame = skimage.io.imread(directory+'/'+entry)
#     # skimage.io.imshow(frame)
#     # skimage.io.show()
#     ped.writeFrame(frame)
# ped.close()
# print('{} frames'.format(frame_no))



directory = base_dir+'/overview'
writer = skvideo.io.FFmpegWriter(base_dir+"/{}_Overview.mp4".format(fname), outputdict={
  '-vcodec': 'libx264', '-b': '300000000','-r':'30'
})

for frame_no,entry in enumerate(sorted(os.listdir(directory))):
    frame = skimage.io.imread(directory+'/'+entry)
    # skimage.io.imshow(frame)
    # skimage.io.show()
    writer.writeFrame(frame)

writer.close()
print('{} frames'.format(frame_no))

#
# directory = base_dir+'/driverview'
# driver = skvideo.io.FFmpegWriter(base_dir+"/{}_Driver.mp4".format(fname), outputdict={
#   '-vcodec': 'libx264', '-b': '300000000','-r':'30'
# })
#
# for frame_no,entry in enumerate(sorted(os.listdir(directory))):
#     frame = skimage.io.imread(directory+'/'+entry)
#     # skimage.io.imshow(frame)
#     # skimage.io.show()
#     driver.writeFrame(frame)
# driver.close()
# print('{} frames'.format(frame_no))
# #
#
# directory = base_dir+'/driverviewback'
# driver2 = skvideo.io.FFmpegWriter(base_dir+"/{}_DriverBack.mp4".format(fname), outputdict={
#   '-vcodec': 'libx264', '-b': '300000000','-r':'30'
# })
#
# for frame_no,entry in enumerate(sorted(os.listdir(directory))):
#     frame = skimage.io.imread(directory+'/'+entry)
#     # skimage.io.imshow(frame)
#     # skimage.io.show()
#     driver2.writeFrame(frame)
#
# driver2.close()
# print('{} frames'.format(frame_no))


