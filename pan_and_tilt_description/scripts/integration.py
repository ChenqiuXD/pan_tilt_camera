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


class ImgListener:
    def __init__(self, topic, epsilon=30):
        self.topic = topic  # Topic to listen
        self.img = []       # To store the image received from callback function
        self.epsilon = epsilon
        # Construct the kernel which will be run in the gpu thread
        self.mod = SourceModule("""
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

        rospy.init_node('image_listener', anonymous=True)
        # The image is from robot4's camera. Please control the joints of robot4 if you want to change the received image. 
        rospy.Subscriber(self.topic, Image, self.imgCallback, queue_size=1)
    
    def imgCallback(self, data):
        """ Callback function in which we store the image received """
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        r_channel = cv_image[:,:,0]  # Take the Red channel of image for data
        r_channel_resized = np.resize(r_channel, data.height*data.width)    # resize the array to one dimension
        self.img = r_channel_resized.astype(np.float32)

        # If you want to check the image, please comment all the following code. (This is only for debugging)
        # Please do not show image and calculate the integral at the same time.
        # cv2.imshow("image", cv_image)
        # cv2.waitKey(1)  # wait for one microsecond   

    def integrateWithCPU(self):
        start = time.time()
        integral = 0
        for data in self.img:
            if(data>self.epsilon):
                integral += self.func(data)
            else:
                continue
        end = time.time()
        return integral, end-start

    def func(self, data):
        """A function which convert each data to a specific value.
        For illustration, here I set: value = 0.5*data+20, where data is the value of red in a pixel """
        return data * 0.5 + 20

    def integrateWithGPU(self):
        start = time.time()
        array_size = self.img.shape[0]
        print (array_size)

        # Allocate space in gpu memory
        img_gpu = cuda.mem_alloc(self.img.nbytes)
        cuda.memcpy_htod(img_gpu, self.img)

        # Calculate
        func = self.mod.get_function("calcVal")
        func(img_gpu, np.uint32(array_size), grid=((array_size+256-1)//256, 1, 1), block=(256, 1, 1))

        # Copy from gpu memory to cpu memory
        img_func = np.empty_like(self.img)
        cuda.memcpy_dtoh(img_func, img_gpu)
        
        end = time.time()
        print (self.img)
        print (img_func)
        return 0, end-start


if __name__ == "__main__":
    # Get the image
    listener = ImgListener(topic="/robot4/pan_and_tilt/camera/image_raw", epsilon=30)
    d = rospy.Duration(0.1, 0)
    rospy.sleep(d)
    if (listener.img!=[]):
        rospy.loginfo("Listener has got image")
    else:
        rospy.logwarn("No image is received, please check whether main_sim.launch is launched")

    # calculate with CPU
    result, duration = listener.integrateWithCPU()
    print ("Integration calculated with CPU time: ", duration, ". The result is: ", result)

    # calculate with GPU
    result, duration = listener.integrateWithGPU()
    print ("Integration calculated with GPU time: ", duration, ". The result is: ", result)
    
