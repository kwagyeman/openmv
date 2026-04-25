def unittest(data_path, temp_path):
    import image

    img = image.Image(data_path + "/qrcode.pgm", copy_to_fb=True)
    codes = img.find_qrcodes()
    if len(codes) != 1:
        return False
    c = codes[0]
    return (
        c.x == 76
        and c.y == 36
        and c.w == 168
        and c.h == 168
        and c.payload == "https://openmv.io"
        and c.version == 1
        and c.ecc_level == 1
        and c.mask == 3
        and c.data_type == 4
        and c.eci == 0
    )
