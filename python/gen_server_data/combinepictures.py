import matplotlib.pyplot as plt
from PIL import Image

filepath = "/Users/test/Documents/generate_map/script/gen_map/output/"
for i in range(0,51):
    scene = Image.open(filepath + 'scene/'+'scene_' + str(i) + '.png')
    image = Image.open(filepath + 'image/'+'image_' + str(i) + '.png')
    (width1, height1) = scene.size
    (width2, height2) = image.size
    result_width = width1 + width2
    result_height = max(height1, height2)
    result = Image.new('RGB', (result_width, result_height))
    result.paste(im=scene, box=(0, 0))
    result.paste(im=image, box=(width1, 0))
    # plt.imshow(result)
    result.save(filepath + 'merged/'+'merged_' + str(i) + '.png')


