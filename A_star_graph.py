import numpy as np
import matplotlib.pyplot as plt



optimal_path=np.genfromtxt("/Users/henrycappel/Documents/coords_simple.txt",dtype=(float))


x = optimal_path[:,0]
y = optimal_path[:,1]

plt.scatter(x,y)
plt.show()