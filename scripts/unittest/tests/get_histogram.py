def unittest(data_path, temp_path):
    import image

    e = 0.0000001
    # Load image
    img = image.Image(data_path + "/cat.pgm", copy_to_fb=True)

    # Load histogram
    with open(data_path + "/cat.csv", "r") as f:
        hist1 = [float(x) for x in f.read().split(",")]

    # Get histogram
    hist2 = img.get_histogram()

    # Compare
    for a, b in zip(hist1, hist2.bins):
        if abs(a - b) > e:
            return False

    stats = hist2.get_statistics()
    return (
        hist2.get_percentile(0.5).value == 96
        and stats.l_mean == 81 and stats.l_median == 96 and stats.l_mode == 0 and stats.l_stdev == 59
        and stats.l_min == 0 and stats.l_max == 255 and stats.l_lq == 13 and stats.l_uq == 128
        and stats.a_mean == 0 and stats.a_median == 0 and stats.a_mode == 0 and stats.a_stdev == 0
        and stats.a_min == 0 and stats.a_max == 0 and stats.a_lq == 0 and stats.a_uq == 0
        and stats.b_mean == 0 and stats.b_median == 0 and stats.b_mode == 0 and stats.b_stdev == 0
        and stats.b_min == 0 and stats.b_max == 0 and stats.b_lq == 0 and stats.b_uq == 0
    )
