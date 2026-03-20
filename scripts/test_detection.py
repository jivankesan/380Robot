#!/usr/bin/env python3
"""Standalone detection visualizer — no ROS required.

Usage:
    python3 scripts/test_detection.py <image_path>
    python3 scripts/test_detection.py <image_path> --save out.png
    python3 scripts/test_detection.py <image_path> --masks   # show raw HSV masks

Shows blue circle, green lego figure, and red line detections overlaid on the
image, using the same HSV/ROI parameters as the ROS nodes / vision.yaml.
Press any key to close, or pass --save to write the annotated image.
"""

import argparse
import math
import sys

import cv2
import numpy as np

# ── HSV parameters (must match object_detector_node.py / vision.yaml) ─────────
BLUE = dict(
    h_min=90, h_max=130,
    s_min=100, s_max=255,
    v_min=50,  v_max=255,
    min_area=800,  max_area=200_000,
    circularity_min=0.4,
    conf_threshold=0.45,
)

GREEN = dict(
    h_min=40, h_max=80,
    s_min=60,  s_max=255,
    v_min=40,  v_max=210,
    min_area=3_000, max_area=200_000,
    conf_threshold=0.45,
)

# ── Red line HSV parameters (must match line_detector_node.py / vision.yaml) ──
RED = dict(
    # Lower red: H=0-10
    h1_min=0,   h1_max=10,
    # Upper red (wrap-around): H=160-180
    h2_min=160, h2_max=180,
    s_min=100, s_max=255,
    v_min=100, v_max=255,
    min_area_px=500,
    # ROI (fraction of frame height/width) — must match vision.yaml
    roi_y_start=0.45, roi_y_end=0.95,
    roi_x_start=0.10, roi_x_end=0.90,
)

# ── Grab-zone threshold (must match fsm.yaml pickup_cy_threshold) ──────────────
PICKUP_CY_THRESHOLD = 0.80


def detect_blue_circle(image, hsv):
    h, w = image.shape[:2]
    mask = cv2.inRange(hsv,
                       np.array([BLUE['h_min'], BLUE['s_min'], BLUE['v_min']]),
                       np.array([BLUE['h_max'], BLUE['s_max'], BLUE['v_max']]))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best, best_area = None, 0
    for c in contours:
        area = cv2.contourArea(c)
        if not (BLUE['min_area'] <= area <= BLUE['max_area']):
            continue
        perim = cv2.arcLength(c, True)
        if perim < 1.0:
            continue
        circularity = 4.0 * math.pi * area / (perim * perim)
        if circularity < BLUE['circularity_min']:
            continue
        x, y, bw, bh = cv2.boundingRect(c)
        score = min(1.0, circularity * 0.6 + min(1.0, area / 30_000.0) * 0.4)
        if score >= BLUE['conf_threshold'] and area > best_area:
            best_area = area
            best = dict(cx=(x + bw / 2) / w, cy=(y + bh / 2) / h,
                        bw=bw / w, bh=bh / h,
                        px_x=x, px_y=y, px_w=bw, px_h=bh,
                        score=score, circularity=circularity)
    return best, mask


def detect_green_figure(image, hsv):
    h, w = image.shape[:2]
    mask = cv2.inRange(hsv,
                       np.array([GREEN['h_min'], GREEN['s_min'], GREEN['v_min']]),
                       np.array([GREEN['h_max'], GREEN['s_max'], GREEN['v_max']]))
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best, best_area = None, 0
    for c in contours:
        area = cv2.contourArea(c)
        if not (GREEN['min_area'] <= area <= GREEN['max_area']):
            continue
        x, y, gw, gh = cv2.boundingRect(c)
        score = min(1.0, area / 20_000.0)
        if score >= GREEN['conf_threshold'] and area > best_area:
            best_area = area
            best = dict(cx=(x + gw / 2) / w, cy=(y + gh / 2) / h,
                        bw=gw / w, bh=gh / h,
                        px_x=x, px_y=y, px_w=gw, px_h=gh,
                        score=score)
    return best, mask


def detect_red_line(image):
    h, w = image.shape[:2]
    y0 = int(h * RED['roi_y_start'])
    y1 = int(h * RED['roi_y_end'])
    x0 = int(w * RED['roi_x_start'])
    x1 = int(w * RED['roi_x_end'])
    roi = image[y0:y1, x0:x1]

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lo1 = np.array([RED['h1_min'], RED['s_min'], RED['v_min']])
    hi1 = np.array([RED['h1_max'], RED['s_max'], RED['v_max']])
    lo2 = np.array([RED['h2_min'], RED['s_min'], RED['v_min']])
    hi2 = np.array([RED['h2_max'], RED['s_max'], RED['v_max']])
    mask = cv2.bitwise_or(cv2.inRange(hsv, lo1, hi1), cv2.inRange(hsv, lo2, hi2))

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    M = cv2.moments(mask)
    area = M['m00']
    if area < RED['min_area_px']:
        return None, mask, (y0, y1, x0, x1)

    cx_roi = M['m10'] / area
    cy_roi = M['m01'] / area
    return dict(
        cx_px=int(cx_roi) + x0,
        cy_px=int(cy_roi) + y0,
        area=area,
        lateral_error=cx_roi - (roi.shape[1] / 2.0),
    ), mask, (y0, y1, x0, x1)


def annotate(image, blue, green, red_line, red_roi):
    out = image.copy()
    h, w = out.shape[:2]

    # Red line ROI box and detection
    y0, y1, x0, x1 = red_roi
    cv2.rectangle(out, (x0, y0), (x1, y1), (0, 0, 180), 1)
    if red_line:
        cv2.circle(out, (red_line['cx_px'], red_line['cy_px']), 8, (0, 0, 255), -1)
        cv2.line(out, (w // 2, y0), (w // 2, y1), (0, 180, 255), 1)  # centre ref
        cv2.putText(out,
                    f'red line  area={int(red_line["area"])}  '
                    f'lat_err={red_line["lateral_error"]:+.1f}px',
                    (x0 + 4, y0 + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 80, 255), 1)
    else:
        cv2.putText(out, 'red line: NOT DETECTED',
                    (8, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)

    # Draw grab-zone line (bottom 20% threshold)
    grab_y = int(PICKUP_CY_THRESHOLD * h)
    cv2.line(out, (0, grab_y), (w, grab_y), (0, 200, 255), 1)
    cv2.putText(out, f'grab zone cy>={PICKUP_CY_THRESHOLD}',
                (4, grab_y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 255), 1)

    # Blue circle
    if blue:
        x, y, bw, bh = blue['px_x'], blue['px_y'], blue['px_w'], blue['px_h']
        cv2.rectangle(out, (x, y), (x + bw, y + bh), (255, 80, 0), 2)
        cx_px = int(blue['cx'] * w)
        cy_px = int(blue['cy'] * h)
        cv2.circle(out, (cx_px, cy_px), 5, (255, 80, 0), -1)
        label = (f"blue_circle  score={blue['score']:.2f}  "
                 f"circ={blue['circularity']:.2f}  "
                 f"w={blue['bw']:.2f}")
        cv2.putText(out, label, (x, y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 80, 0), 1)
    else:
        cv2.putText(out, 'blue_circle: NOT DETECTED',
                    (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 80, 0), 2)

    # Green figure
    if green:
        x, y, gw, gh = green['px_x'], green['px_y'], green['px_w'], green['px_h']
        in_grab_zone = green['cy'] >= PICKUP_CY_THRESHOLD
        color = (0, 255, 60) if in_grab_zone else (0, 180, 60)
        cv2.rectangle(out, (x, y), (x + gw, y + gh), color, 2)
        cx_px = int(green['cx'] * w)
        cy_px = int(green['cy'] * h)
        cv2.circle(out, (cx_px, cy_px), 5, color, -1)
        grab_tag = '  *** IN GRAB ZONE ***' if in_grab_zone else ''
        label = (f"green_figure  score={green['score']:.2f}  "
                 f"cy={green['cy']:.2f}{grab_tag}")
        cv2.putText(out, label, (x, y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
    else:
        cv2.putText(out, 'green_figure: NOT DETECTED',
                    (8, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 60), 2)

    return out


def main():
    parser = argparse.ArgumentParser(description='Test blue/green detection on an image')
    parser.add_argument('image', help='Path to input image')
    parser.add_argument('--save', metavar='OUTPUT', help='Save annotated image instead of displaying')
    parser.add_argument('--masks', action='store_true', help='Also show HSV mask windows')
    args = parser.parse_args()

    image = cv2.imread(args.image)
    if image is None:
        print(f'ERROR: could not load image: {args.image}', file=sys.stderr)
        sys.exit(1)

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    blue, blue_mask = detect_blue_circle(image, hsv)
    green, green_mask = detect_green_figure(image, hsv)
    red_line, red_mask, red_roi = detect_red_line(image)

    # Print results
    if red_line:
        print(f'[red_line]      area={int(red_line["area"])}  '
              f'lat_err={red_line["lateral_error"]:+.1f}px  DETECTED')
    else:
        print('[red_line]      NOT DETECTED')

    if blue:
        print(f'[blue_circle]   cx={blue["cx"]:.3f}  cy={blue["cy"]:.3f}  '
              f'w={blue["bw"]:.3f}  score={blue["score"]:.3f}  '
              f'circularity={blue["circularity"]:.3f}')
    else:
        print('[blue_circle]   NOT DETECTED')

    if green:
        in_zone = green['cy'] >= PICKUP_CY_THRESHOLD
        print(f'[green_figure]  cx={green["cx"]:.3f}  cy={green["cy"]:.3f}  '
              f'w={green["bw"]:.3f}  score={green["score"]:.3f}'
              + ('  *** IN GRAB ZONE ***' if in_zone else ''))
    else:
        print('[green_figure]  NOT DETECTED')

    annotated = annotate(image, blue, green, red_line, red_roi)

    if args.save:
        cv2.imwrite(args.save, annotated)
        print(f'Saved: {args.save}')
    else:
        cv2.imshow('Detection result (any key to close)', annotated)
        if args.masks:
            cv2.imshow('Blue mask', blue_mask)
            cv2.imshow('Green mask', green_mask)
            cv2.imshow('Red mask (ROI)', red_mask)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
