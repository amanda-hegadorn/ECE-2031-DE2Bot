# import statements 
import random
import pandas as pd
from scipy.spatial import distance_matrix, distance

# Ask user for number of destinations
npoints = int(input("Type the npoints: "))

# Start robot from origin
sample = [(0,0)]
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
print(*[f"({w:.2f}, {h:.2f})" for w, h in sample], sep=', ')

# Calculate distance with euclidean formula
mat_dist = distance.cdist(sample, sample, 'euclidean')
df_mat_dist = pd.DataFrame(mat_dist)
print(df_mat_dist)


# Find the closest point to the starting point, different from diagonal and save results
# Greedy algorithm
for _ in range(npoints-1):
    closest_dist = df_mat_dist.loc[closest_idx, ~df_mat_dist.index.isin(path_points)].min()
    closest_idx = df_mat_dist.loc[closest_idx, ~df_mat_dist.index.isin(path_points)].idxmin()
    path_points.append(closest_idx)
    path_length += closest_dist
print(path_points, path_length)
