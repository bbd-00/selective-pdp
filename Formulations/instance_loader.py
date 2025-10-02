def load_instance_from_txt(filename):
    """
    Load a PDPTW instance exported by save_instance_to_txt.
    Returns a dictionary with all necessary data.
    """
    with open(filename, "r") as f:
        lines = [line.strip() for line in f if line.strip()]

    i = 0
    instance = {}

    # PARAMETERS
    assert lines[i] == "PARAMETERS"; i += 1
    H, b, W = map(float, lines[i].split()); i += 1
    c_com_h, c_com_d, c_occ_o, c_occ_d, rev_order = map(float, lines[i].split()); i += 1
    instance.update(dict(H=H, b=b, W=W,
                         c_com_h=c_com_h, c_com_d=c_com_d,
                         c_occ_o=c_occ_o, c_occ_d=c_occ_d,
                         rev_order=rev_order))

    # COMMITTED DRIVERS
    assert lines[i] == "COMMITTED_DRIVERS"; i += 1
    nC = int(lines[i]); i += 1
    committed = []
    for _ in range(nC):
        parts = lines[i].split(); i += 1
        cid, x, y, e, l = int(parts[0]), float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4])
        committed.append(dict(id=cid, location=(x,y), e=e, l=l))
    instance["committed"] = committed

    # OCCASIONAL DRIVERS
    assert lines[i] == "OCCASIONAL_DRIVERS"; i += 1
    nO = int(lines[i]); i += 1
    occasional = []
    for _ in range(nO):
        parts = lines[i].split(); i += 1
        oid, x, y, e, l = int(parts[0]), float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4])
        occasional.append(dict(id=oid, location=(x,y), e=e, l=l))
    instance["occasional"] = occasional

    # REQUESTS
    assert lines[i] == "REQUESTS"; i += 1
    nR = int(lines[i]); i += 1
    requests = []
    for _ in range(nR):
        parts = lines[i].split(); i += 1
        rid = int(parts[0])
        px, py, dx, dy = map(float, parts[1:5])
        dp, dd = int(parts[5]), int(parts[6])
        ep, lp, ed, ld = map(float, parts[7:11])
        requests.append(dict(id=rid,
                             pickup=(px,py), dest=(dx,dy),
                             delay_p=dp, delay_d=dd,
                             e_pickup=ep, l_pickup=lp,
                             e_dest=ed, l_dest=ld))
    instance["requests"] = requests
    return instance