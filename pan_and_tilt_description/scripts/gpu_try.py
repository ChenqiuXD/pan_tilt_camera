#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

import pycuda.driver as cuda
import pycuda.autoinit
from pycuda.compiler import SourceModule

def imgCallback(data):
    """ The callback function when subscriber receive the image """
    result = integrateWithGPU()

def integrateWithGPU():
    """ The integration function by GPU, pycuda """
    # Allocate space in the gpu memory
    array_size = 10000000
    h_array = np.random.randint(1, 101, size=array_size).astype(np.float32)
    h_array_copy = h_array.copy()
    print h_array

    # Allocate memory in gpu
    d_array = cuda.mem_alloc(h_array.nbytes)
    cuda.memcpy_htod(d_array, h_array)

    # Construct the kernel which will be run in the gpu thread
    mod = SourceModule("""
        __global__ void calcVal(float *a, const size_t size)
        {
            // A function which convert each data to a specific value.
            // For illustration, here I set: value = 0.5*data+20, where data is the value of red in a pixel 
            unsigned int tid = threadIdx.x;
            unsigned int gid = tid + blockIdx.x * blockDim.x;
            if (gid < size) {
                a[gid] = a[gid]*0.5+20;
            }
        }
        """)

    # Use gpu to calculate the kernel
    func = mod.get_function("calcVal")
    # print (array_size+256-1)//256
    func(d_array, np.uint32(array_size), grid=((array_size+256-1)//256, 1, 1), block=(256, 1, 1))
    
    # Copy data from gpu memory to cpu memory
    cuda.memcpy_dtoh(h_array, d_array)
    print h_array

if __name__ == "__main__":
    rospy.init_node('image_listener', anonymous=True)
    # The image is from robot4's camera. Please control the joints of robot4 if you want to change the received image. 
    rospy.Subscriber("/robot4/pan_and_tilt/camera/image_raw", Image, imgCallback, queue_size=10)
    rospy.spin()

    # _unused = 0
    # imgCallback(_unused)