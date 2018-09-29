import cv2
import numpy as np
from math import *
import random


img2 = cv2.imread('map4.pgm')
img = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

world_x = 423
world_y = 305
scan_l = 0.611061990261
scan_r = 0.618436157703
scan_res = 0.00240606302395
image_res = 0.05

class robot:
    def __init__(self):
        self.x = int(random.random() * world_x)
        self.y = int(random.random() * world_y)
        while img[self.y][self.x] <251:
            self.x = int(random.random() * world_x)
            self.y = int(random.random() * world_y)
        self.orientation = random.random() * 2.0 * pi
        self.forward_noise = 30;
        self.turn_noise = 0.1;
        self.sense_noise = 5;

    def set(self, new_x, new_y, new_orientation):
        if new_x < 0 or new_x >= world_x:
            raise ValueError, 'X coordinate out of bound'
        if new_y < 0 or new_y >= world_y:
            raise ValueError, 'Y coordinate out of bound'
        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError, 'Orientation must be in [0..2pi]'
        if img[new_y][new_x] <250:
            raise ValueError, 'area not free'
        self.x = int(new_x)
        self.y = int(new_y)
        self.orientation = float(new_orientation)

    def set_noise(self, new_f_noise, new_t_noise, new_s_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.forward_noise = float(new_f_noise);
        self.turn_noise = float(new_t_noise);
        self.sense_noise = float(new_s_noise);

    def sense(self):
        global img2, img
        final_scan_result = []
        for j in range(0, 4):
            orients = np.arange(self.orientation - scan_l + ((pi/2)*j), self.orientation + scan_r + scan_res + ((pi/2)*j), scan_res)
            scan_result = np.zeros(len(orients))
            i = 0
            offset_x = int(3*cos(self.orientation+(pi/2)*j))
            offset_y = int(3*sin(self.orientation+(pi/2)*j))
            start_x = self.x + offset_x
            start_y = self.y + offset_y
            for theta in orients:
                end_x = start_x + int(150 * cos(theta))
                end_y = start_y + int(150 * sin(theta))
                if end_x < 0:
                    end_x = 0
                if end_y < 0:
                    end_y = 0
                if end_x > world_x:
                    end_x = world_x
                if end_y > world_y:
                    end_y = world_y
                # difference and absolute difference between points
                # used to calculate slope and relative location between points


                dX = end_x - start_x
                dY = end_y - start_y
                dXa = np.abs(dX)
                dYa = np.abs(dY)
                # predefine numpy array for output based on distance between points
                itbuffer = np.empty(shape=(np.maximum(dYa, dXa), 3), dtype=np.float32)
                itbuffer.fill(np.nan)

                # Obtain coordinates along the line using a form of Bresenham's algorithm
                negY = start_y > end_y
                negX = start_x > end_x
                if start_x == end_x:  # vertical line segment
                    itbuffer[:, 0] = start_x
                    if negY:
                        itbuffer[:, 1] = np.arange(start_y - 1, start_y - dYa - 1, -1)
                    else:
                        itbuffer[:, 1] = np.arange(start_y + 1, start_y + dYa + 1)
                elif start_y == end_y:  # horizontal line segment
                    itbuffer[:, 1] = start_y
                    if negX:
                        itbuffer[:, 0] = np.arange(start_x - 1, start_x - dXa - 1, -1)
                    else:
                        itbuffer[:, 0] = np.arange(start_x + 1, start_x + dXa + 1)
                else:  # diagonal line segment
                    steepSlope = dYa > dXa
                    if steepSlope:
                        slope = float(dX) / dY
                        if negY:
                            itbuffer[:, 1] = np.arange(start_y - 1, start_y - dYa - 1, -1)
                        else:
                            itbuffer[:, 1] = np.arange(start_y + 1, start_y + dYa + 1)
                        itbuffer[:, 0] = (slope * (itbuffer[:, 1] - start_y)).astype(np.int) + start_x
                    else:
                        slope = float(dY) / dX
                        if negX:
                            itbuffer[:, 0] = np.arange(start_x - 1, start_x - dXa - 1, -1)
                        else:
                            itbuffer[:, 0] = np.arange(start_x + 1, start_x + dXa + 1)
                        itbuffer[:, 1] = (slope * (itbuffer[:, 0] - start_x)).astype(np.int) + start_y

                # Remove points outside of image
                colX = itbuffer[:, 0]
                colY = itbuffer[:, 1]
                itbuffer = itbuffer[(colX >= 0) & (colY >= 0) & (colX < world_x) & (colY < world_y)]

                # Get intensities from img ndarray
                itbuffer[:, 2] = img[itbuffer[:, 1].astype(np.uint), itbuffer[:, 0].astype(np.uint)]
                indices, = np.where(itbuffer[:, 2] <250 )
                if len(indices) != 0:
                    x = np.amin(indices)
                    scan_result[i] = sqrt((start_x - itbuffer[x, 0])**2 + (start_y-itbuffer[x, 1])**2)
                i+=1
            final_scan_result.append(scan_result)
        return final_scan_result

    def move(self, turn, forward):
        if forward < 0:
            raise ValueError, 'Robot cant move backwards'

            # turn, and add randomness to the turning command
        orientation = self.orientation + float(turn) + random.gauss(0.0, self.turn_noise)
        orientation %= 2 * pi

        # move, and add randomness to the motion command
        dist = float(forward) + random.gauss(0.0, self.forward_noise)
        x = self.x + (cos(orientation) * dist)
        y = self.y + (sin(orientation) * dist)
        x %= world_x  # cyclic truncate
        y %= world_y
        self.orientation = orientation
        self.x = x
        self.y = y
        # # set particle

    def Gaussian(self, mu, sigma, x):
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(-((mu-x)**2)/(sigma**2)/2.0)/sqrt(2.0*pi*(sigma**2))

    def measurement_prob(self, measurements):
        # calculates how likely a measurement should be
        prob = 1.0
        my_vals = self.sense()
        for j in range(len(measurements)):
            m = len(measurements[j])/10
            for i in range(0, len(measurements[j]), m):
                iprob = self.Gaussian(my_vals[j][i], self.sense_noise, measurements[j][i])
                prob *= iprob
        return prob

    def __repr__(self):
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), str(self.orientation))




