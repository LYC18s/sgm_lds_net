"""Image-relative current-road schematic descriptors for the PL BEV renderer.

No distance, lane count or road width is inferred here. Input is unrectified
left-camera detection coordinates, so no unrelated live SGM frame is sampled.
The hardware register interface also accepts externally calibrated positions.
"""
import math

BEV_CAPABILITY = 0x42455631
BEV_BASE = 0x200


def descriptors(detections):
    """Return exactly sixteen packed descriptors; unused entries are hidden.

    Horizontal bearing and image footpoint determine schematic placement.
    Display-only duplicate suppression does not change detection API results.
    """
    candidates = []
    for detection in detections:
        try:
            category = int(detection['category'])
            kind = 2 if category == 0 else 1 if category in (2, 4, 5) else 0
            values = [float(detection[k]) for k in ('x1', 'y1', 'x2', 'y2', 'score')]
        except (KeyError, TypeError, ValueError, OverflowError):
            continue
        if not kind or not all(math.isfinite(v) for v in values):
            continue
        x1, y1, x2, y2, score = values
        if x2 <= x1 or y2 <= y1 or x2 <= 0 or x1 >= 640 or y2 <= 0 or y1 >= 480:
            continue
        candidates.append((score, kind, max(0., x1), max(0., y1), min(639., x2), min(479., y2)))
    selected, result = [], []
    for candidate in sorted(candidates, key=lambda v: -v[0]):
        _, kind, x1, y1, x2, y2 = candidate
        duplicate = False
        for _, other_kind, a, b, c, d in selected:
            intersection = max(0., min(x2, c)-max(x1, a))*max(0., min(y2, d)-max(y1, b))
            union = (x2-x1)*(y2-y1)+(c-a)*(d-b)-intersection
            if kind == other_kind and union > 0 and intersection/union >= .6:
                duplicate = True
                break
        if duplicate:
            continue
        width, height = (42, 60) if kind == 1 else (16, 32)
        center_x = 320+((x1+x2)/2-320)*(.25+y2/640)*.375
        center_y = 32+y2*.62
        left = max(0, min(640-width, int(round(center_x-width/2))))
        top = max(0, min(378-height, int(round(center_y-height/2))))
        result.append(left | (top << 10) | (kind << 19))
        selected.append(candidate)
        if len(result) == 16:
            break
    return result+[0]*(16-len(result))
