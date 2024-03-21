from PIL import Image
import os

    
for i in range(1, 101):
    paths = [f"uav{j}/{i:04}.png" for j in range(1, 5)]
    images = []
    w = 0
    h = 0
    for p in paths:
        img = Image.open(p)
        (w, h) = img.size
        w = w
        h = h
        images.append(img)

    print(w,h)
    res = Image.new('RGB', (2*w, 2*h))
    for (idx,img) in enumerate(images):
        res.paste(im=img, box=(idx%2*w, int(idx >= 2)*h))

    res.save(f"res_{i:04}.png")
        


