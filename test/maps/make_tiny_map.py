import numpy as np
from PIL import Image

values = np.array(
        [[  1.,   0., 0.5],
         [0.25,  0.5,  1.],
         [  0., 0.75,  0.]])

values_8_bit = (values * 255).astype(np.uint8)
img = Image.fromarray(values_8_bit)

img.save('tiny.png')
