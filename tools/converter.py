import sys
from struct import unpack
import png

def check_integrity():
    pass

if __name__ == "__main__":
    filename = sys.argv[1]

    with open(filename, "rb") as f:
        image_file_bytes = f.read()


    frame_length = unpack("H", image_file_bytes[2: 4])[0]
    payload = image_file_bytes[16+2:frame_length - 16]

    frame_width = int(image_file_bytes[14])
    frame_height = int(image_file_bytes[15])

    f = open(filename + '.png', 'wb')      # binary mode is important
    w = png.Writer(frame_width, frame_height, greyscale=True)
    w.write(f, [ image_file_bytes[i*frame_width : (i+1)*frame_width]  for i in range(frame_height)])
    f.close()
