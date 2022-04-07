import math
import numpy as np
from matplotlib import pyplot

pi = math.pi   #mathモジュールのπを利用

x = np.linspace(0, 2*pi, 100)  #0から2πまでの範囲を100分割したnumpy配列
y = np.sin(x)

pyplot.plot(x, y)
pyplot.show()
