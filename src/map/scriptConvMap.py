import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

a=Image.open("bite.png")
a=np.asarray(a)

b=np.zeros((200,300))

for k in range(len(a)):
    for j in range(len(a[0])):
            b[k,j] = a[k,j,0]/255
b=np.logical_not(b)
b=np.transpose(b)

np.save("test_map.npy",b)

plt.imshow(b)
plt.show()

