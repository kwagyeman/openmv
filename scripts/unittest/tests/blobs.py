def unittest(data_path, temp_path):
    import image

    thresholds = [
        (0, 100, 56, 95, 41, 74),   # generic_red_thresholds
        (0, 100, -128, -22, -128, 99),  # generic_green_thresholds
        (0, 100, -128, 98, -128, -16),  # generic_blue_thresholds
    ]

    # Load image
    img = image.Image(data_path + "/blobs.ppm", copy_to_fb=True)

    blobs = img.find_blobs(thresholds, pixels_threshold=200, area_threshold=200)

    b0 = blobs[0]
    b1 = blobs[1]
    b2 = blobs[2]
    return (
        b0.x == 61 and b0.y == 21 and b0.w == 49 and b0.h == 41
        and b0.pixels == 1556 and int(b0.cxf) == 83 and int(b0.cyf) == 41
        and b1.x == 22 and b1.y == 20 and b1.w == 39 and b1.h == 45
        and b1.pixels == 1294 and int(b1.cxf) == 40 and int(b1.cyf) == 42
        and b2.x == 105 and b2.y == 20 and b2.w == 36 and b2.h == 41
        and b2.pixels == 1004 and int(b2.cxf) == 124 and int(b2.cyf) == 38
    )
