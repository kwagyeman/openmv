def unittest(data_path, temp_path):
    import image

    img = image.Image(data_path + "/barcode.pgm", copy_to_fb=True)
    codes = img.find_barcodes()
    c = codes[0]
    return (
        c.x == 11
        and c.y == 12
        and c.w == 514
        and c.h == 39
        and c.payload == "https://openmv.io/"
        and c.type == 15
        and c.rotation == 0.0
        and c.quality == 40
    )
