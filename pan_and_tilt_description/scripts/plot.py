#!/usr/bin/env python3
from matplotlib import pyplot as plt
import pandas as pd
import re
import numpy as np
import csv

# Convert 'test.txt' to 'test.csv' and store it. 
read_file = pd.read_csv(r'test.txt')
read_file.to_csv(r'test.csv', index=None)

# Plot the H values by matplotlib
x = []
y = []
z = []
count = 0
with open("test.csv", 'r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
        x.append(count) # Iteration time
        count += 1
        y.append(float(row[0]))  # H-value
        # z.append(float(row[1]))
plt.plot(x, y, label='H values')
plt.xlabel('iteration counts')
plt.ylabel('H value')
plt.legend()
plt.show()
