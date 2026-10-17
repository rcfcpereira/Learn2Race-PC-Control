"""Floor texture of the Dragster track (2026 Dragster rules, section IV, Figures 4 and 5), 2 mm per pixel.

Two parallel lanes of 30 cm, each bordered by a 20 mm black stripe and marked down its middle by another;
10 m from the start line to the 20 mm stripe that ends the lanes (the finish); after it the two stopping
zones of rules II.10f-g, closed by lines at 0.5 m and 1.0 m past the finish (no centre line there, Figure 4).
Around it: 0.5 m of floor behind the start line and up to 2 m past the finish (the unobstructed run-out,
IV-4), 0.1 m at each side. The track runs up the image (the robots start at the bottom), as the TrackBotGP
drawings are placed in make_world.py.

    python gazebo/make_dragster_texture.py      (writes media/track_dragster.png)
"""
# References (full entries: docs/references.md at the repository root):
#   [R2] ISEP, Dragster competition rules (2026 Robotics@ISEP Open): track dimensions (IV-1, IV-2, Figures 4-5)
import os

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MM_PX = 2.0
PAPER, INK = 235, 20                 # grey levels of the TrackBotGP drawings (make_world.py textures)
LINE = 0.020                         # stripe width (m)
LANE = 0.30                          # lane width, stripe centre to stripe centre (m)
RACE = 10.0                          # start line to finish stripe (m)
ZONES = (0.5, 1.0)                   # stopping-zone boundaries past the finish (m)
BEHIND, RUNOUT, SIDE = 0.5, 2.0, 0.1  # floor margins (m)

# Positions in metres: x across the track from its left border, y along it from the start line.
WIDTH = 2 * LANE + 2 * SIDE
LENGTH = BEHIND + RACE + RUNOUT
START_Y_PX = round((RUNOUT + RACE) * 1000 / MM_PX)         # image row of the start line
LEFT_LANE_X_PX = round((SIDE + LANE / 2) * 1000 / MM_PX)    # image column of the left lane's centre


def px(m):
    return round(m * 1000 / MM_PX)


def main():
    h, w = px(LENGTH), px(WIDTH)
    img = np.full((h, w), PAPER, np.uint8)

    def stripe_along(x, y0, y1):        # a stripe along the track, centred on x, from y0 to y1
        r0, r1 = START_Y_PX - px(y1), START_Y_PX - px(y0)
        img[r0:r1, px(SIDE + x - LINE / 2):px(SIDE + x + LINE / 2)] = INK

    def stripe_across(y, x0=-LINE / 2, x1=2 * LANE + LINE / 2):
        img[START_Y_PX - px(y + LINE / 2):START_Y_PX - px(y - LINE / 2), px(SIDE + x0):px(SIDE + x1)] = INK

    end = RACE + ZONES[-1]
    stripe_along(0.0, 0.0, end)                    # outer borders, through the stopping zones
    stripe_along(2 * LANE, 0.0, end)
    stripe_along(LANE, 0.0, RACE)                  # border between the lanes, ends at the finish
    for c in (LANE / 2, 3 * LANE / 2):             # middle of each lane (the line the robots follow)
        stripe_along(c, 0.0, RACE - LINE)
    stripe_across(0.0)                             # start line (rules II.6b)
    stripe_across(RACE)                            # end of the lanes: the finish
    for z in ZONES:
        stripe_across(RACE + z)                    # stopping-zone boundaries
    out = os.path.join(HERE, "media", "track_dragster.png")
    cv2.imwrite(out, img)
    print(f"{out}: {w} x {h} px = {WIDTH:.2f} x {LENGTH:.2f} m; start row {START_Y_PX}, left lane x {LEFT_LANE_X_PX}")


if __name__ == "__main__":
    main()
