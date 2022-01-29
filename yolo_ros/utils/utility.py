#!/usr/bin/env python3

from collections import deque, Counter
from statistics import mean as st_mean

# create a class for calculating moving average
class mAverage:
    def __init__(self, queue_size):
        self.q = deque(maxlen=queue_size)
        
    def avg (self, data):
        self.q.append(data)
        return st_mean(self.q)
    
    
# create a class for majority voting
class majVote:
    def __init__(self, queue_size):
        self.q = deque(maxlen=queue_size)
        
    def vote(self, newVote):
        self.q.append(newVote)
        cnt = Counter(self.q)
        vote, num = cnt.most_common()[0]
        return vote, num
        

# get distance from depth array via block average
import numpy as np

# Perform block average for the input depth array with 
# the given kernel size. Select the minimum block average
# and scale it with depth_scale.
# It skips 0 (invalid values) in block average calculation 
# It skips blocks that have less than mdp+1 non zero values 
# depth_array: the depth data within the BBOX
# kernel_size: a tuple specifying the kernel width and height
# 
def get_distance(depth_array, kernel_size, mdp):
    
    #depth_scale = 0.001       want an integer number so commented
    rows, cols = depth_array.shape
    k_rows, k_cols = kernel_size
    row_offset = int((rows%k_rows)/2)
    col_offset = int((cols%k_cols)/2)

    rows = rows - rows%k_rows
    cols = cols - cols%k_cols
    d= depth_array[row_offset:rows+row_offset, col_offset:cols+col_offset].copy()
    d = d.reshape(rows//k_rows, k_rows, cols//k_cols, k_cols)
    a = d.sum(axis=(1, -1)).astype(float)
    b = (d!=0).sum(axis=(1,-1)).astype(float)
    #print(b)
    c = np.divide(a, b, out=np.zeros_like(a), where=b>mdp)
    #print(c)
    minval = np.min(c[np.nonzero(c)])
    #print(minval)
    
    return minval
    
def get_dist_debug(depth_array, kernel_size, mdp):
    
    depth_scale = 0.001
    rows, cols = depth_array.shape
    k_rows, k_cols = kernel_size
    row_offset = int((rows%k_rows)/2)
    col_offset = int((cols%k_cols)/2)

    rows = rows - rows%k_rows
    cols = cols - cols%k_cols
    d= depth_array[row_offset:rows+row_offset, col_offset:cols+col_offset].copy()
    d = d.reshape(rows//k_rows, k_rows, cols//k_cols, k_cols)
    a = d.sum(axis=(1, -1)).astype(float)
    b = (d!=0).sum(axis=(1,-1)).astype(float)
    #print(b)
    c = np.divide(a, b, out=np.zeros_like(a), where=b>mdp)
    #print(c)
    minval = np.min(c[np.nonzero(c)])
    row_array, col_array = np.where(c==minval)
    row_val = row_array[0] 
    col_val = col_array[0]
    row_index = row_offset + row_val*k_rows
    col_index = col_offset + col_val*k_cols
    #print(x, y)
    #print(m, n)
    #print(row_offset)
    #print(col_offset)
    return minval*depth_scale, row_index, col_index

# perform non-zero block average
# for blocks with the number of valid data points smaller than mdp, NaN is assigned    
def nzba(depth_array, kernel_size, mdp):
    
    rows, cols = depth_array.shape
    k_rows, k_cols = kernel_size
    row_offset = int((rows%k_rows)/2)
    col_offset = int((cols%k_cols)/2)

    rows = rows - rows%k_rows
    cols = cols - cols%k_cols
    d= depth_array[row_offset:rows+row_offset, col_offset:cols+col_offset].copy()
    d = d.reshape(rows//k_rows, k_rows, cols//k_cols, k_cols)
    a = d.sum(axis=(1, -1)).astype(float)
    b = (d!=0).sum(axis=(1,-1)).astype(float)
    #print(b)
    c = np.divide(a, b, out=np.zeros_like(a), where=b>mdp)
    c[c==0] = np.NaN
    
    return c
 
