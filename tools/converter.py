import os
import sys
from struct import unpack
import png

def check_integrity():
    pass

def convert_image(path: str):
    with open(path, "rb") as f:
        image_file_bytes = f.read()


    frame_length = unpack("H", image_file_bytes[2: 4])[0]
    payload = image_file_bytes[20:-2]

    frame_width = int(image_file_bytes[14])
    frame_height = int(image_file_bytes[15])
    with open(path + '.png', 'wb') as f: 
        f = open(path + '.png', 'wb')      # binary mode is important
        w = png.Writer(frame_width, frame_height, greyscale=True)
        w.write(f, [ payload[i*frame_width : (i+1)*frame_width]  for i in range(frame_height)])
        f.close()

if __name__ == "__main__":
    path = sys.argv[1]

    if os.path.isdir(path):
        for fn in os.listdir(path):
            if os.path.isfile(fn):
                convert_image(fn)
    elif os.path.isfile(path):
        convert_image(path)