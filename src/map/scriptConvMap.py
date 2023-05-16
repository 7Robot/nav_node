import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import os

liste_images = os.listdir("images")

for im in liste_images:
    if not(".png" in im):
        continue
    
    print("Loading : "+im)

    a=Image.open("images/"+im)
    a=np.asarray(a)

    b=np.zeros((200,300))
    
    for k in range(len(a)):
        for j in range(len(a[0])):
                b[k,j] = a[k,j]==8
    b=np.logical_not(b)
    b=np.transpose(b)
    b=np.flipud(b)
    print('Shape : ',b.shape)
    
    np.save(im[:-4]+".npy",b)
    
    #plt.imshow(b)
    #plt.show()

