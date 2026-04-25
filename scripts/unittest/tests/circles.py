def unittest(data_path, temp_path):
    import image

    img = image.Image(data_path + "/shapes.ppm", copy_to_fb=True)
    circles = img.find_circles(threshold=5000, x_margin=30, y_margin=30, r_margin=30)
    if len(circles) != 1:
        return False
    c = circles[0]
    return c.x == 118 and c.y == 56 and c.r == 22 and c.magnitude == 5856
