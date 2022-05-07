from matplotlib import colors
import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np
cmap = colors.ListedColormap(['white','gray','blue','yellow'])
bounds=[0, 2, 4, 6, 8]
norm = colors.BoundaryNorm(bounds, cmap.N)
data = np.array([[1,1,1,1,7,7,7,7], [1,1,1,1,1,1,1,5], [1,1,1,1,1,1,1,5], [1,1,1,3,1,1,1,5], [1,1,1,1,1,1,3,5]])
ax = sns.heatmap(data, cmap=cmap, norm=norm, linewidths=.5, 
linecolor='black', square=True, cbar=False)
sns.set()
plt.annotate('S', (1.4, 3.4))
plt.show()