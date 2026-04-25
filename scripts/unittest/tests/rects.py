def unittest(data_path, temp_path):
    import image

    img = image.Image(data_path + "/shapes.ppm", copy_to_fb=True)
    rects = img.find_rects(threshold=50000)
    if len(rects) != 1:
        return False
    r = rects[0]
    return (
        r.x == 23
        and r.y == 39
        and r.w == 35
        and r.h == 36
        and r.magnitude == 146566
    )
