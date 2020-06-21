import cv2
import numpy as np
import scipy.ndimage.measurements as sc
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from PIL import Image

class Cone_detect():
    def __init__(self):
        self.kernel_1 = np.ones((1,1),np.uint8)
        self.kernel_2 = np.ones((4,4),np.uint8)
        #self.low_red = np.array([150,50,0])
        #self.upper_red = np.array([255,100,255])
        #self.low_blue = np.array([0,0,90])
        #self.upper_blue = np.array([60,60,255])
        self.lower_red1 = np.array([0,43,36])
        self.upper_red1 = np.array([20,255,255])
        self.lower_red2 = np.array([146,43,36])
        self.upper_red2 = np.array([180,255,255])
        self.lower_blue = np.array([110,50,50])
        self.upper_blue = np.array([130,255,255])

    def euc_dist(self,center1,center2):
        return np.sqrt((center1[0] - center2[0])**2 + (center1[1] - center2[1])**2)
    def detect(self,image):
        img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        
        #cv2.imshow('mask_red',mask_red)
        #cv2.imshow('mask_blue',mask_blue)
        #cv2.waitKey(0)
        
        self.get_label(img)
        blue_array = self.filtering(self.blue_labeled_array,self.blue_num_features)
        red_array = self.filtering(self.red_labeled_array,self.red_num_features)
        blue_pos, blue_bound, blue_count = self.locate(blue_array)
        red_pos, red_bound, red_count = self.locate(red_array)
        #for ii in range(len(blue_bound)):
        #    cv2.rectangle(image,(blue_bound[ii][1],blue_bound[ii][0]), (blue_bound[ii][3],blue_bound[ii][2]),(255, 0, 0), 3)
        #for ii in range(len(red_bound)):
        #    cv2.rectangle(image,(red_bound[ii][1],red_bound[ii][0]), (red_bound[ii][3],red_bound[ii][2]),(0, 0, 255), 3)
        #cv2.imshow("cone",image)
        #cv2.waitKey(1)
        return blue_pos, red_pos, blue_bound, red_bound, blue_count, red_count

    def filtering(self,labeled_array,num_features):
        for ii in range(0,num_features):
            num = len(labeled_array[labeled_array == ii])
            #print num
            if num < 20 or num > 10000:
                labeled_array[labeled_array == ii] = 0
            else:
                labeled_array[labeled_array == ii] = 255
        #plt.imshow(labeled_array)
        #plt.gray()
        #plt.show()
        return labeled_array

    def get_label(self,img):
        mask_red1 = cv2.inRange(img,self.lower_red1,self.upper_red1)
        mask_red2 = cv2.inRange(img,self.lower_red2,self.upper_red2)
        mask_red = mask_red1 | mask_red2
        mask_blue = cv2.inRange(img,self.lower_blue,self.upper_blue)
        red = self.open_close(mask_red)
        blue = self.open_close(mask_blue)
        self.blue_labeled_array, self.blue_num_features = sc.label(blue)
        self.red_labeled_array, self.red_num_features = sc.label(red)

    def open_close(self,img):
        dilation = cv2.dilate(img,self.kernel_2, iterations = 1)
        #plt.imshow(dilation)
        #plt.gray()
        #plt.show()
        return dilation

    def locate(self,array):
        pos = []
        bound = []
        labeled_array, num_features = sc.label(array)
        for ii in range(1,num_features):
            if len(labeled_array[labeled_array == ii]) < 10000:
                arr = np.where(labeled_array == ii)
                if np.mean(arr[0]) > 380:
                    center = (np.mean(arr[0]),np.mean(arr[1]))
                    to_add = True
                    for jj in range(len(pos)):
                        if self.euc_dist(pos[jj],center) < 60:
                            to_add = False
                            break
                    if to_add:
                        pos += [center]
                        #bound += [(np.max(arr[0]) - np.min(arr[0]),np.max(arr[1]) - np.min(arr[1]))]
                        bound += [(np.min(arr[0]), np.min(arr[1]), np.max(arr[0]),np.max(arr[1]))]
        count = len(pos)
        if  count < 4:
            for jj in range(4 - count):
                pos += [(np.inf,np.inf)]
        #print(pos)
        #plt.imshow(array)
        #plt.gray()
        #plt.show()
        return pos, bound, count

def main():
    img_name = '0.jpg'
    img = cv2.imread(img_name)
    cone_detect = Cone_detect()
    blue_pos, red_pos, blue_bound, red_bound, blue_count, red_count = cone_detect.detect(img)
    for ii in range(len(blue_bound)):
        cv2.rectangle(img,(blue_bound[ii][1],blue_bound[ii][0]), (blue_bound[ii][3],blue_bound[ii][2]),(255, 0, 0), 3)
    for ii in range(len(red_bound)):
        cv2.rectangle(img,(red_bound[ii][1],red_bound[ii][0]), (red_bound[ii][3],red_bound[ii][2]),(0, 0, 255), 3)
    print("blue count == ")
    print(blue_count)
    print("red count == ")
    print(red_count)
    cv2.imshow('result', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    '''
    im = np.array(Image.open(img_name))
    fig, ax = plt.subplots(1)
    ax.imshow(im)
    for ii in range(len(blue_bound)):
        rect = patches.Rectangle((blue_pos[ii][1] - blue_bound[ii][1] / 2,blue_pos[ii][0] - blue_bound[ii][0] / 2),blue_bound[ii][0],blue_bound[ii][1],linewidth = 2,edgecolor='b', facecolor = 'none')
        ax.add_patch(rect)
    for ii in range(len(red_bound)):
        rect = patches.Rectangle((red_pos[ii][1] - red_bound[ii][1] / 2,red_pos[ii][0] - red_bound[ii][0] / 2),red_bound[ii][0],red_bound[ii][1],linewidth = 2,edgecolor='r', facecolor = 'none')
        ax.add_patch(rect)
    plt.show()
    '''
if __name__ == "__main__":
    main()
