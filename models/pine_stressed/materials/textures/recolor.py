from PIL import Image
import colorsys

filename = "branch_2_diffuse.png"

img = Image.open(filename).convert("RGBA")
pixels = img.load()

for y in range(img.height):
    for x in range(img.width):
        r, g, b, a = pixels[x, y]

        if a == 0:
            continue

        # Only modify pixels where green is dominant
        if g > r * 1.05 and g > b * 1.05:
            h, s, v = colorsys.rgb_to_hsv(
                r / 255.0,
                g / 255.0,
                b / 255.0
            )

            # Yellow / dry vegetation
            h = 45 / 360
            s = min(0.65, s)
            v = min(1.0, v * 1.05)

            nr, ng, nb = colorsys.hsv_to_rgb(h, s, v)

            pixels[x, y] = (
                int(nr * 255),
                int(ng * 255),
                int(nb * 255),
                a
            )

img.save(filename)

print("Stressed tree texture created")
