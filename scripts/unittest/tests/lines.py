def unittest(data_path, temp_path):
    import image

    img = image.Image(data_path + "/shapes.ppm", copy_to_fb=True)
    lines = img.find_lines(threshold=5000, theta_margin=25, rho_margin=25)
    if len(lines) != 4:
        return False
    l0 = lines[0]
    l1 = lines[1]
    l2 = lines[2]
    l3 = lines[3]
    return (
        l0.x1 == 22 and l0.y1 == 0 and l0.x2 == 22 and l0.y2 == 119
        and l0.length == 119 and l0.magnitude == 8670 and l0.theta == 0 and l0.rho == 22
        and l1.x1 == 0 and l1.y1 == 39 and l1.x2 == 159 and l1.y2 == 39
        and l1.length == 159 and l1.magnitude == 8670 and l1.theta == 90 and l1.rho == 39
        and l2.x1 == 57 and l2.y1 == 0 and l2.x2 == 57 and l2.y2 == 119
        and l2.length == 119 and l2.magnitude == 8670 and l2.theta == 0 and l2.rho == 57
        and l3.x1 == 0 and l3.y1 == 75 and l3.x2 == 159 and l3.y2 == 75
        and l3.length == 159 and l3.magnitude == 10710 and l3.theta == 90 and l3.rho == 75
    )
