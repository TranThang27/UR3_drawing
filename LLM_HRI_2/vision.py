import math
import time

import cv2
import numpy as np

OBJECTS = ("red_cube", "yellow_cube", "blue_cube", "green_cube", "purple_cube")
ZONES = ("zone_a", "zone_b", "zone_c")
HUES = {
    "red_cube": ((0, 8), (173, 179)),
    "yellow_cube": ((22, 37),),
    "blue_cube": ((108, 127),),
    "green_cube": ((45, 78),),
    "purple_cube": ((128, 148),),
    "zone_a": ((82, 98),),
    "zone_b": ((9, 21),),
    "zone_c": ((150, 172),),
}
CAMERA_POSITION = np.array([0.82, 0.0, 0.80])
PITCH = 0.95
OPTICAL_TO_WORLD = np.array([
    [0.0, math.sin(PITCH), -math.cos(PITCH)],
    [1.0, 0.0, 0.0],
    [0.0, -math.cos(PITCH), math.sin(PITCH) * -1],
])


def point_image(depth, intrinsics):
    fy, fx = intrinsics[4], intrinsics[0]
    cx, cy = intrinsics[2], intrinsics[5]
    v, u = np.indices(depth.shape)
    with np.errstate(invalid="ignore"):
        optical = np.stack(((u - cx) * depth / fx, (v - cy) * depth / fy, depth), axis=-1)
        return optical @ OPTICAL_TO_WORLD.T + CAMERA_POSITION


def occupancy(objects, zones, complete):
    result = {}
    for name, zone in zones.items():
        occupants = [
            obj for obj, data in objects.items()
            if abs(data["position"][0] - zone["position"][0]) < 0.085
            and abs(data["position"][1] - zone["position"][1]) < 0.085
            and data["position"][2] < 0.07
        ]
        result[name] = {
            **zone, "occupants": occupants,
            "status": "OCCUPIED" if occupants else ("FREE" if complete else "UNKNOWN"),
        }
    return result


def analyze(rgb, depth, intrinsics):
    points = point_image(depth, intrinsics)
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    finite = np.isfinite(points).all(axis=-1)
    roi = (finite & (points[..., 0] > 0.12) & (points[..., 0] < 0.52)
           & (np.abs(points[..., 1]) < 0.38))
    objects, zones = {}, {}
    annotated = rgb.copy()
    for name, intervals in HUES.items():
        color = np.zeros(depth.shape, dtype=bool)
        for low, high in intervals:
            color |= (hsv[..., 0] >= low) & (hsv[..., 0] <= high)
        mask = color & (hsv[..., 1] > 105) & (hsv[..., 2] > 45) & roi
        is_zone = name in ZONES
        mask &= (np.abs(points[..., 2] - 0.002) < 0.008) if is_zone else (
            (points[..., 2] > 0.006) & (points[..., 2] < 0.24))
        contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL,
                                      cv2.CHAIN_APPROX_SIMPLE)
        if not is_zone and contours:
            visible = points[mask]
            if len(visible) and np.max(np.ptp(visible[:, :2], axis=0)) < 0.075:
                contours = [cv2.convexHull(np.concatenate(contours))]
        candidates = []
        for contour in contours:
            if cv2.contourArea(contour) < 30:
                continue
            component = np.zeros(depth.shape, np.uint8)
            cv2.drawContours(component, [contour], -1, 1, -1)
            sample = points[(component > 0) & mask]
            if len(sample) < 40:
                continue
            top = float(np.percentile(sample[:, 2], 85))
            full_sample = sample
            elevated = not is_zone and top > 0.09
            if elevated:
                front_x = float(np.percentile(sample[:, 0], 97))
                face = sample[np.abs(sample[:, 0] - front_x) < 0.003]
                if len(face) < 40:
                    continue
                face_rect = cv2.minAreaRect(face[:, 1:3].astype(np.float32))
                if not all(0.037 < side < 0.065 for side in face_rect[1]):
                    continue
                candidates.append({"position": [front_x-0.025, *map(float, face_rect[0])],
                                   "yaw": 0.0, "pixels": len(face)})
                continue
            if not is_zone:
                sample = sample[np.abs(sample[:, 2] - top) < 0.003]
            if len(sample) < 30:
                continue
            rect = cv2.minAreaRect(sample[:, :2].astype(np.float32))
            size = sorted(rect[1])
            if is_zone:
                valid = 0.100 < size[0] < 0.135 and size[1] < 0.135
            else:
                valid = 0.038 < size[0] < 0.060 and size[1] < 0.063
            if not valid:
                if not is_zone and top < 0.075:
                    front_x = float(np.percentile(full_sample[:, 0], 97))
                    face = full_sample[np.abs(full_sample[:, 0] - front_x) < 0.003]
                    if len(face) >= 40:
                        low, high = np.percentile(face[:, 1:3], [1, 99], axis=0)
                        if 0.038 < high[0]-low[0] < 0.065 and 0.032 < high[1]-low[1] < 0.060:
                            candidates.append({"position": [front_x-.025, float((low[0]+high[0])/2), float(high[1]-.025)],
                                               "yaw": 0.0, "pixels": len(face)})
                continue
            x, y = rect[0]
            yaw = math.radians(rect[2])
            candidates.append({"position": [float(x), float(y), 0.001 if is_zone else top - 0.025],
                               "yaw": 0.0 if is_zone else yaw, "pixels": len(sample)})
            u, v, w, h = cv2.boundingRect(contour)
            cv2.rectangle(annotated, (u, v), (u+w, v+h), (255, 255, 255), 1)
            cv2.putText(annotated, name, (u, v-5), cv2.FONT_HERSHEY_SIMPLEX,
                        0.4, (255, 255, 255), 1)
        if len(candidates) == 1:
            (zones if is_zone else objects)[name] = candidates[0]
    complete = set(objects) == set(OBJECTS) and set(zones) == set(ZONES)
    zones = occupancy(objects, zones, complete)
    free = []
    if complete:
        for x in np.arange(0.19, 0.391, 0.02):
            for y in np.arange(-0.30, 0.301, 0.02):
                if math.hypot(x, y) > 0.43:
                    continue
                if any(abs(x-p["position"][0]) < 0.105 and abs(y-p["position"][1]) < 0.105
                       for p in objects.values()):
                    continue
                if any(abs(x-p["position"][0]) < 0.105 and abs(y-p["position"][1]) < 0.105
                       for p in zones.values()):
                    continue
                patch = roi & (np.abs(points[..., 0]-x) < 0.04) & (np.abs(points[..., 1]-y) < 0.04)
                observed = points[patch]
                if len(observed) < 250 or np.mean(np.abs(observed[:, 2]) < 0.006) < 0.99:
                    continue
                coverage, _, _ = np.histogram2d(observed[:, 0], observed[:, 1], bins=8,
                    range=[[x-0.04, x+0.04], [y-0.04, y+0.04]])
                if np.count_nonzero(coverage) < 62:
                    continue
                free.append([float(x), float(y), 0.025])
    return {"observed_at": time.monotonic(), "source": "rgbd_camera", "complete": complete,
            "objects": objects, "zones": zones, "free_positions": free,
            "missing": sorted(set(OBJECTS + ZONES) - set(objects) - set(zones))}, annotated


def require_fresh(state, max_age=2.0):
    if not state or state.get("source") != "rgbd_camera":
        raise RuntimeError("PERCEPTION_UNAVAILABLE: camera state is missing")
    age = time.monotonic() - state["observed_at"]
    if not 0 <= age <= max_age:
        raise RuntimeError("PERCEPTION_STALE: camera frame is too old")
    return state


def choose_temporary(state, exclude=None):
    require_fresh(state)
    if not state["complete"]:
        raise RuntimeError("PERCEPTION_INCOMPLETE: cannot select a free position")
    candidates = state["free_positions"]
    if exclude is not None:
        candidates = [p for p in candidates if math.dist(p[:2], exclude[:2]) > 0.11]
    if not candidates:
        raise RuntimeError("NO_FREE_POSITION: camera found no suitable empty tabletop patch")
    return min(candidates, key=lambda p: math.hypot(p[0]-0.26, p[1]))
