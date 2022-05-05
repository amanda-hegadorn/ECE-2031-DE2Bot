#######
# DE2Bot Sorting Algorithm
# Jennifer Wolfe, Amanda Hegadorn
# Spring 2022
# ECE 2031
#######
# import statements 
import random
import pandas as pd
from tabulate import tabulate
from scipy.spatial import distance_matrix, distance

# Ask user for number of destinations
npoints = int(input("Type the npoints: "))

# Start robot from origin
sample = [(0,0)]
xlist = [0]
ylist = [0]
# Initialize variables
closest_idx = 0
path_points = [closest_idx]
path_length = 0

# Ask user for coordinates and add to list
for _ in range(npoints):
    print("Destination # ", _+1)
    x = int(input("Enter x: "))
    y = int(input("Enter y: "))
    sample.append((x, y))
    # formula to convert to robot units
    x = int(x*12*25.4/1.05)    
    y = int(y*12*25.4/1.05) 
    # append to list
    xlist.append(x)
    ylist.append(y)

# Calculate distance with euclidean formula
mat_dist = distance.cdist(sample, sample, 'euclidean')
df_mat_dist = pd.DataFrame(mat_dist)

# Find the closest point to the starting point not visited before and save results
# Greedy algorithm
for _ in range(npoints):
    closest_dist = df_mat_dist.loc[closest_idx, ~df_mat_dist.index.isin(path_points)].min()
    closest_idx = df_mat_dist.loc[closest_idx, ~df_mat_dist.index.isin(path_points)].idxmin()
    path_points.append(closest_idx)
    path_length += closest_dist
# print order of sorted points and total path length
print("Order: ", path_points, "Length (ft): ",path_length)

# print a nice table for the user
d = []
for point in range(npoints+1):
    # print x and y separately since that is how user enters
    x = xlist[path_points[point]]
    # set up 2's complement
    x2 = (0b1111111111111111 &  x)
`   # original index, feet, robot units, binary, 2's complement binary, and hex
    d.append([path_points[point], sample[path_points[point]], x, bin(x), bin(x2), hex(x2)])
    # same steps for y as x
    y = ylist[path_points[point]]
    y2 = (0b1111111111111111 &  y)
    d.append([path_points[point], sample[path_points[point]], y, bin(y), bin(y2), hex(y2)])

print(tabulate(d, headers=["Destination #", "Point", "Robot Units", "Binary", "2's Complement Binary", "Hexadecimal"]))
