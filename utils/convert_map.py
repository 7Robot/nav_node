"""
A simple utility to convert file from .png, .jpg, .jpeg or .ppm to text file understandable by the cpp nav_node.

Usage:
    python convert_map.py <image_file> <output_file>
"""


from PIL import Image
import numpy as np
import sys

MAP_WIDTH = 300
MAP_HEIGHT = 200

if __name__ == '__main__':
    argv = sys.argv
    if len(argv) != 3:
        raise ValueError('Usage: python convert_map.py <image_file> <output_file>')
    image_file = argv[1]
    output_file = argv[2]
    image = Image.open(image_file)
    image_np = np.array(image)
    image_np = image_np[:, :, 0]

    
    shape = image_np.shape
    if shape[0]>shape[1]:
        print('The map is not in the right orientation. Rotating 90° clockwise.')
        image_np = np.rot90(image_np)
    if len(shape) == 3:
        print('The map is not in grayscale. Converting to grayscale.')
        for i in range(shape[0]):
            for j in range(shape[1]):
                image_np[i, j] = image_np[i, j, 0]
    if image_np.shape != (MAP_HEIGHT, MAP_WIDTH):
        print('The map is not of the right size. Expected: {}x{}. Got: {}x{}'.format(MAP_WIDTH, MAP_HEIGHT, image_np.shape[1], image_np.shape[0]))
        res = input('Do you want to resize the map? (y/N)')
        if res == 'y':
            print('Resizing map...')
            image_np = np.array(Image.fromarray(image_np).resize((MAP_WIDTH, MAP_HEIGHT)))
        else:
            print('Aborting.')
            exit()


    print('Converting map...')
    s = ''
    for i in range(MAP_HEIGHT):
        for j in range(MAP_WIDTH):
            if image_np[i, j] < 128:
                s += '1'
            else:
                s += '0'
    print('Done converting, writing to text_file...')
    with open(output_file, 'w') as f:
        f.write(s)
    print('Done writing to text_file.')

        
