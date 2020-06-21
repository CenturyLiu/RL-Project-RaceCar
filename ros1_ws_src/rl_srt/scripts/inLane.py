import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import math
from shapely.geometry import MultiPoint, Point,Polygon



class mypolygon():
    def __init__(self,path):
        self.red = pd.read_csv(path + "/cone_position/red.csv").to_numpy()
        self.blue = pd.read_csv(path + "/cone_position/blue.csv").to_numpy()
        self.red_x = []
        self.red_y = []
        self.blue_x = []
        self.blue_y = []
        self.red_tuples = []
        self.blue_tuples = []
        self.outer_boundary = []
        self.inner_boundary = []
        self.array2tuple()
        self.clockWiseSort()
        #self.getBoundary()


    def euc_dist2(self,point1, point2):
        return (point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2

    def sortClock(self,point):
        return math.atan2(point[0],point[1])
        
    def array2tuple(self):
        bias = 4.5 #due to the definition of the world
        for ii in range(self.red.shape[0]):
            self.red_tuples += [(self.red[ii][0],self.red[ii][1] + 4.5)]
            self.red_x += [self.red[ii][0]]
            self.red_y += [self.red[ii][1] + 4.5]
        for jj in range(self.blue.shape[0]):
            self.blue_tuples += [(self.blue[jj][0],self.blue[jj][1] + 4.5)]
            self.blue_x += [self.blue[jj][0]]
            self.blue_y += [self.blue[jj][1] + 4.5]
        

    def inLane(self,x,y):
        pt = Point((x,y))
        if self.poly_outer.contains(pt) and not self.poly_inner.contains(pt):
            return True
        return False

    '''
    def getBoundary(self):
        for ii in range(self.red.shape[0]):
            dist = np.sqrt(self.euc_dist2((0,0),(self.red[ii][0], self.red[ii][1])))
            ratio = (dist - 1.0) / dist
            self.outer_boundary += [(self.red[ii][0] * ratio,self.red[ii][1] * ratio)] 
        for jj in range(self.blue.shape[0]):
            dist = np.sqrt(self.euc_dist2((0,0),(self.blue[jj][0], self.blue[jj][1])))
            ratio = (dist + 1.0) / dist
            self.inner_boundary += [(self.blue[jj][0] * ratio,self.blue[jj][1] * ratio)] 
        self.poly_outer_boundary = Polygon(self.outer_boundary)
        self.poly_inner_boundary = Polygon(self.inner_boundary)
    

    def NotinBoundary(self,x,y):
        pt = Point((x,y))
        if self.poly_outer_boundary.contains(pt) and not self.poly_inner_boundary.contains(pt):
            return True
        return False
    '''
    def closeToBoundary(self,x,y):
        pt = (x,y)
        for ii in range(len(self.red_tuples)):
            if np.sqrt(self.euc_dist2(pt,self.red_tuples[ii])) < 2.0:
                return True
        for ii in range(len(self.blue_tuples)):
            if np.sqrt(self.euc_dist2(pt,self.blue_tuples[ii])) < 2.0:
                return True
        return False


    def clockWiseSort(self):
        self.red_tuples.sort(key = self.sortClock)
        self.blue_tuples.sort(key = self.sortClock, reverse = True)
        self.poly_outer = Polygon(self.red_tuples)
        self.poly_inner = Polygon(self.blue_tuples)


    def functionCheck(self):
        self.plot_points()
        x = np.linspace(-20,30,60)
        y = np.linspace(-20,30,60)
        inside_x = []
        inside_y = []
        outside_x = []
        outside_y = []
        boundary_x = []
        boundary_y = []
        for ii in range(len(x)):
            for jj in range(len(x)):
                if self.inLane(x[ii],y[jj]):
                    #if self.NotinBoundary(x[ii],y[jj]):
                    inside_x += [x[ii]]
                    inside_y += [y[jj]]
                    #else:
                    #    boundary_x = [x[ii]]
                    #    boundary_y = [y[jj]]
                else:
                    outside_x += [x[ii]]
                    outside_y += [y[jj]]
        plt.plot(inside_x,inside_y,'g.')
        plt.plot(outside_x,outside_y,'w.')
        #plt.plot(boundary_x,boundary_y,'y.')


    def plot_points(self):
        plt.plot(self.red_x,self.red_y,'r.')
        plt.plot(self.blue_x,self.blue_y,'b.')
        

def main():
    lane_segment = mypolygon("./")
    lane_segment.functionCheck()
    print(lane_segment.inLane(-13.917865,14.346258))
    plt.show()


if __name__ == "__main__":
    main()
