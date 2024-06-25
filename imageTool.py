import sys
import argparse
from PIL import Image, ImageOps

def apply_bayer_dithering(image):
    bayer_matrix = [
        [0, 8, 2, 10],
        [12, 4, 14, 6],
        [3, 11, 1, 9],
        [15, 7, 13, 5]
    ]
    matrix_size = 4

    def dither_channel(channel):
        width, height = channel.size
        channel_data = channel.load()
        for y in range(height):
            for x in range(width):
                old_pixel = channel_data[x, y]
                new_pixel = int(old_pixel + bayer_matrix[y % matrix_size][x % matrix_size] * 16 / (matrix_size ** 2))
                new_pixel = max(0, min(new_pixel, 255))
                channel_data[x, y] = new_pixel
        return channel

    r, g, b = image.split()
    r = dither_channel(r)
    g = dither_channel(g)
    b = dither_channel(b)
    return Image.merge("RGB", (r, g, b))

def convert_to_rgb565(image):
    def to_rgb565(r, g, b):
        return (r & 0xf8) << 8 | (g & 0xfc) << 3 | (b >> 3)
    
    width, height = image.size
    data = image.getdata()
    rgb565_data = [to_rgb565(r, g, b) for r, g, b in data]
    
    return rgb565_data, width, height

def main():
    parser = argparse.ArgumentParser(description='Convert image to RGB565 format.')
    parser.add_argument('inputfile', type=str, help='Input image file')
    parser.add_argument('-x', type=int, help='Output width')
    parser.add_argument('-y', type=int, help='Output height')
    parser.add_argument('-p', action='store_true', help='Preserve aspect ratio')
    parser.add_argument('-o', type=str, help='Output file')
    parser.add_argument('-b', action='store_true', help='Enable Bayer dithering')
    parser.add_argument('-n', type=str, default='image_data', help='Array name')

    args = parser.parse_args()

    if args.p and args.x and args.y:
        print("Error: When preserving the aspect ratio cannot specify both width and height simultaneously.")
        sys.exit(1)
    
    image = Image.open(args.inputfile).convert('RGB')

    if args.x and args.y:
        new_size = (args.x, args.y)
    elif args.x:
        new_size = (args.x, int(args.x * image.height / image.width)) if args.p else None
    elif args.y:
        new_size = (int(args.y * image.width / image.height), args.y) if args.p else None
    else:
        new_size = image.size

    if not new_size:
        print("Error: Must specify both width and height if not preserving aspect ratio.")
        sys.exit(1)

    image = image.resize(new_size, Image.Resampling.LANCZOS)

    if args.b:
        image = apply_bayer_dithering(image)

    rgb565_data, width, height = convert_to_rgb565(image)

    output = f"const uint16_t {args.n}[{width} * {height}] = {{\n"
    for i, value in enumerate(rgb565_data):
        if i % width == 0:
            output += "\n"
        output += f"0x{value:04x}, "
    output = output.rstrip(", ") + "\n};"

    if args.o:
        with open(args.o, 'w') as f:
            f.write(output)
    else:
        print(output)

if __name__ == "__main__":
    main()