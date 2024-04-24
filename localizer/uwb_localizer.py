import numpy as np
import serial
import threading
from queue import Queue
import time
import re
from scipy.optimize import least_squares
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Localizer:
    def __init__(self, tag_port, anchor_names, anchor_locations):
        self.front_tag = serial.Serial(tag_port, 115200, timeout=1)
        self.anchor_names = anchor_names
        self.anchor_locations = anchor_locations
        self.queue = Queue()
        self.plotQueue = Queue()

        plt.ion()
        fig = plt.figure()
        if len(self.anchor_locations[0]) == 2:
            # 2D
            self.ax = fig.add_subplot(111)
        if len(self.anchor_locations[0]) == 3:
            # 3D
            self.ax = fig.add_subplot(111, projection='3d')
            


        threading.Thread(target=self.serialThread, daemon=True).start()
        threading.Thread(target=self.localizationThread, daemon=True).start()


        self.plot()

        while True:
            time.sleep(1)
    
    def plot(self):
        x = []
        y = []
        z = []
        while (True):
            data = self.plotQueue.get(True)
            if len(data.x) == 3:
                # 3D
                x.append(data.x[0])
                y.append(data.x[1])
                z.append(data.x[2])
                self.ax.scatter(data.x[0], data.x[1], data.x[2])
                self.ax.set_xlim3d(-10, 10)
                self.ax.set_ylim3d(-10, 10)
                self.ax.set_zlim3d(-10, 10)
                plt.pause(0.01)
                plt.draw()
                self.ax.cla()
            if len(data.x) == 2:
                # 3D
                x.append(data.x[0])
                y.append(data.x[1])
                if (len(x) > 75):
                    x.pop(0)
                    y.pop(0)
                
                x_avg = np.mean(x)
                y_avg = np.mean(y)
                # print(data.x[0], data.x[1])
                # print(x_avg, y_avg)
                self.ax.scatter(x_avg, y_avg)
                self.ax.set_xlim(-3, 40)
                self.ax.set_ylim(-3, 40)
                plt.pause(0.01)
                plt.draw()
                self.ax.cla()


    def findIntersection(self, distances):
        def cost(guess):
            cost = []
            if len(guess) == 2:
                # 2D
                x_guess, y_guess = guess
                for i in range(0, len(self.anchor_locations)):
                    cost.append((x_guess - self.anchor_locations[i][0])**2 +
                                (y_guess - self.anchor_locations[i][1])**2 -
                                distances[i]**2)

            if len(guess) == 3:
                # 3D
                x_guess, y_guess, z_guess = guess
                for i in range(0, len(self.anchor_locations)):
                    cost.append((x_guess - self.anchor_locations[i][0])**2 +
                            (y_guess - self.anchor_locations[i][1])**2 +
                            (z_guess - self.anchor_locations[i][2])**2 -
                            distances[i]**2)

            return cost

        guess = self.anchor_locations[0]
        
        ans = least_squares(cost, guess)

        return ans

    def localizationThread(self):
        distances = [0, 0, 0]
        while (True):
            data = self.queue.get(True)
            anchor = data["anchor"]
            range = data["range"]
            rxPower = data["rxPower"]
            anchor_idx = self.anchor_names.index(anchor)
            # print(anchor_idx)
            distances[anchor_idx] = range
            result = self.findIntersection(distances)
            # print(round(result.x[0], 2), round(result.x[1], 2), round(result.x[2], 2), result.success)
            # print(distances)
            self.plotQueue.put(result)
            

    # From https://stackoverflow.com/a/3368991
    @staticmethod
    def find_between_r(s, first, last):
        try:
            start = s.rindex(first) + len(first)
            end = s.rindex(last, start)
            return s[start:end]
        except ValueError:
            return ""
    
    def serialThread(self):
        while (True):
            msg = str(self.front_tag.readline().decode("utf-8"))
            print(msg)

            anchor_str = "from: "
            anchor_str_idx = msg.find(anchor_str)
            anchor = msg[anchor_str_idx + len(anchor_str):anchor_str_idx + len(anchor_str) + 4]

            range = Localizer.find_between_r(msg, "Range: ", " m")

            rxPower = Localizer.find_between_r(msg, "RX power: ", " dBm")

            try:
                self.queue.put({"anchor": anchor, "range": float(range), "rxPower": float(rxPower)})
            except:
                print("Serial read failed")





if __name__ == "__main__":
    tag_port = "/dev/cu.usbserial-02619876"
    anchor_names = ["1786", "1785", "1784"]
    # X, Y, Z (fwd, side, height) #5.84
    anchor_locations = [[0, 0], [24.21, 0], [0, 32.51]]
    loc = Localizer(tag_port, anchor_names, anchor_locations)
