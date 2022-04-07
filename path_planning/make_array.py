import numpy as np
import matplotlib.pyplot as plt

min_val = 0
max_val = 10

map = np.random.randint(1, 20, size=(max_val, max_val))
map[0,0]=0
map[max_val-1,max_val-1]=0

fig, ax = plt.subplots(figsize=(8,8))
ax.matshow(map, cmap=plt.cm.Blues, vmin=0, vmax=20)
for i in range(max_val):
    for j in range(max_val):
        c = map[j,i]
        ax.text(i, j, str(c), va='center', ha='center')

plt.show()
