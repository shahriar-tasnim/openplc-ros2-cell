"""
sort_destinations.py -- drop-station geometry for the sorting cell (world frame).

Four separated, colour-coded stations around the UR5e's reach arc:
    A  green   ( 0.58,  0.12)
    B  blue    ( 0.33,  0.45)
    C  orange  ( 0.00,  0.52)
    REJECT red (-0.30,  0.40)

Station tables: top visual centred at z=0.37, 0.03 thick -> surface at 0.385.
A parcel (0.08 cube) resting on that surface has its centre at 0.425.
"""

CENTRES = {
    "A":      ( 0.58,  0.12),
    "B":      ( 0.33,  0.45),
    "C":      ( 0.00,  0.52),
    "REJECT": (-0.30,  0.40),
}

PITCH  = 0.09     # spacing between slots within a layer
Z_TOP  = 0.425    # parcel centre when resting on a station top
Z_STEP = 0.08     # parcel height -> next layer up
Z_HIGH = 0.72     # travel height above everything


def slot(dest, n):
    """World (x, y, z) for the n-th parcel routed to `dest`.

    Stations fill a 2x2 footprint then stack upward. REJECT piles at one
    point, rising each time.
    """
    cx, cy = CENTRES.get(dest, CENTRES["REJECT"])
    if dest == "REJECT":
        return cx, cy, Z_TOP + (n % 6) * Z_STEP
    layer = n // 4
    k = n % 4
    dx = (-PITCH / 2) if k in (0, 2) else (PITCH / 2)
    dy = (-PITCH / 2) if k in (0, 1) else (PITCH / 2)
    return cx + dx, cy + dy, Z_TOP + layer * Z_STEP
