import csv
import math
import numpy as np
from matplotlib import pyplot as plt

fig, ax = plt.subplots(figsize=(5, 3))

with open('measurement.txt', 'rb') as csvfile:
    reader = csv.reader(csvfile, delimiter=',', quotechar=',', quoting=csv.QUOTE_NONNUMERIC)
    x = []
    y = []
    for row in reader:
        x.append(row[0])
        y.append(row[1])
    
    plt.title('Path of the robot')
    plt.plot(y, x)
    plt.xlabel('Y')
    plt.ylabel('X')
    plt.legend()
    plt.show()
    
    
        
    
