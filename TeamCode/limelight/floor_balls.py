# Limelight Python (SnapScript) pipeline: finds Pollen and Nectar on the floor.
#
# Set up on the Limelight's web page as a Python pipeline at index LimelightConstants.floorPipeline
# (1), and paste this script in. TeamCode's LimelightCamera.findBalls() reads its output.
#
# Output (llpython), nearest ball first, at most MAX_BALLS:
#   [count, kind, x, y, r, kind, x, y, r, ...]
#   kind: 1 = Pollen (yellow), 2 = red Nectar, 3 = blue Nectar
#   x, y: the ball's centre in pixels (0, 0 = top left); r: its radius in pixels
#
# The HSV ranges are PLACEHOLDERS: tune them on the real balls under field-like lighting, using the
# Limelight's image view. OpenCV HSV: hue 0-179, saturation and value 0-255. Red wraps round hue 0,
# so it has two ranges.

import cv2
import numpy as np

POLLEN = [((20, 120, 100), (35, 255, 255))]
RED_NECTAR = [((0, 120, 80), (8, 255, 255)), ((170, 120, 80), (179, 255, 255))]
BLUE_NECTAR = [((100, 150, 60), (125, 255, 255))]

# Kind code, HSV ranges, and the colour (BGR) its circles are drawn in on the Limelight's image
KINDS = [
    (1, POLLEN, (0, 255, 255)),
    (2, RED_NECTAR, (0, 0, 255)),
    (3, BLUE_NECTAR, (255, 0, 0)),
]

# Blobs smaller than this many pixels are ignored (noise, or balls too far away to matter)
MIN_AREA = 150
# A blob's area as a fraction of its enclosing circle's. A lone ball is about 0.8 or more; two
# touching balls, or a ball half behind something, fall below this and are ignored
MIN_CIRCULARITY = 0.6
# Keep the output well inside what the Limelight sends back (4 numbers per ball, plus the count)
MAX_BALLS = 7

KERNEL = np.ones((5, 5), np.uint8)


def find_blobs(hsv, ranges):
    mask = None
    for low, high in ranges:
        m = cv2.inRange(hsv, np.array(low, dtype=np.uint8), np.array(high, dtype=np.uint8))
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    # Remove specks so they don't turn into blobs
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours


def runPipeline(image, llrobot):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    balls = []
    for kind, ranges, colour in KINDS:
        for contour in find_blobs(hsv, ranges):
            area = cv2.contourArea(contour)
            if area < MIN_AREA:
                continue
            (x, y), r = cv2.minEnclosingCircle(contour)
            if r <= 0 or area / (np.pi * r * r) < MIN_CIRCULARITY:
                continue
            balls.append((y + r, kind, x, y, r, contour, colour))

    # The camera looks down at the floor, so the lowest ball in the image is the nearest
    balls.sort(key=lambda ball: -ball[0])
    balls = balls[:MAX_BALLS]

    llpython = [float(len(balls))]
    for _, kind, x, y, r, contour, colour in balls:
        llpython += [float(kind), float(x), float(y), float(r)]
        cv2.circle(image, (int(x), int(y)), int(r), colour, 2)

    largest = balls[0][5] if balls else np.array([[]])
    return largest, image, llpython
