"""
sort_destinations.py -- pallet geometry for the sorting cell (world frame).

Three pallets plus a reject bin. Each pallet holds parcels in a 2x2 grid,
stacking upward once a layer is full. The reject bin is a single drop point.
All positions are reach-verified for the UR5e at the origin.
"""

# pallet centres (world x, y) -- reach-verified
CENTRES = {
    "A":      (0.45, 0.20),
    "B":      (0.30, 0.38),
    "C":      (0.48, 0.05),
    "REJECT": (0.10, 0.47),
}

PITCH   = 0.09    # spacing between slots in a pallet layer
Z_TOP   = 0.44    # parcel resting height on a pallet (world)
Z_STEP  = 0.08    # parcel height -> next layer up
Z_HIGH  = 0.75    # travel height above everything


def slot(dest, n):
    """Where the n-th parcel sent to `dest` should land (world x, y, z).

    Pallets fill a 2x2 footprint then stack upward. REJECT simply piles
    parcels at one point, rising each time.
    """
    cx, cy = CENTRES.get(dest, CENTRES["REJECT"])
    if dest == "REJECT":
        return cx, cy, Z_TOP + (n % 6) * Z_STEP
    layer = n // 4
    k = n % 4
    dx = (-PITCH / 2) if k in (0, 2) else (PITCH / 2)
    dy = (-PITCH / 2) if k in (0, 1) else (PITCH / 2)
    return cx + dx, cy + dy, Z_TOP + layer * Z_STEP
