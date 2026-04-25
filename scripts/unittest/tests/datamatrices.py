def unittest(data_path, temp_path):
    import image

    img = image.Image(data_path + "/datamatrix.pgm", copy_to_fb=True)
    matrices = img.find_datamatrices()
    if len(matrices) != 1:
        return False
    m = matrices[0]
    return (
        m.x == 34
        and m.y == 15
        and m.w == 90
        and m.h == 89
        and m.payload == "https://openmv.io/"
        and m.rotation == 0.0
        and m.rows == 18
        and m.columns == 18
        and m.capacity == 18
        and m.padding == 0
    )
