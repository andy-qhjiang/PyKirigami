import numpy as np
import matplotlib.pyplot as plt

with open('data/tangram_vertices.txt', 'r') as f:
    lines = f.readlines()
    v = []
    for line in lines:
        face = [float(x) for x in line.split()]
        v.append(face)
    print(v)
