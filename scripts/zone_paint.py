#!/usr/bin/env python3
# encoding: utf-8
"""zone_paint.py -- hand-draw keepout zones on the map with any image editor (Paint / GIMP).

  python3 zone_paint.py export MAP_YAML OUT_PNG [scale] [x,y,label ...]
      map -> big PNG (default 6x) to paint on; optional markers (map metres) drawn as red dots + labels
  python3 zone_paint.py import MAP_YAML EDITED_PNG OUT_PREFIX
      painted PNG -> OUT_PREFIX_keepout.pgm/.yaml : every block that contains a painted BLACK pixel
      becomes a keepout cell (the map's own walls are black too, which is harmless: lethal stays lethal)
Paint rule: pure black (0,0,0) rectangles = forbidden. Do not resize the image.
"""
import sys
from PIL import Image, ImageDraw, ImageFont


def read_yaml(path):
    d = {}
    for line in open(path):
        if ':' in line:
            k, v = line.split(':', 1); d[k.strip()] = v.strip()
    return d['image'], float(d['resolution']), [float(t) for t in d['origin'].strip('[]').split(',')]


def main():
    cmd = sys.argv[1]
    if cmd == 'export':
        yaml_path, out = sys.argv[2], sys.argv[3]
        scale = int(sys.argv[4]) if len(sys.argv) > 4 and ',' not in sys.argv[4] else 6
        markers = [a for a in sys.argv[4:] if ',' in a]
        import os
        image, res, org = read_yaml(yaml_path)
        im = Image.open(os.path.join(os.path.dirname(yaml_path) or '.', image)).convert('RGB')
        w, h = im.size
        big = im.resize((w * scale, h * scale), Image.NEAREST)
        d = ImageDraw.Draw(big)
        try:
            font = ImageFont.truetype('/mnt/c/Windows/Fonts/msyh.ttc', 18)
        except Exception:
            font = None
        for m in markers:
            x, y, label = m.split(',', 2)
            c = (float(x) - org[0]) / res * scale
            r = (h - (float(y) - org[1]) / res) * scale
            d.ellipse([c - 7, r - 7, c + 7, r + 7], outline=(255, 0, 0), width=3)
            d.text((c + 10, r - 10), label, fill=(255, 0, 0), font=font)
        big.save(out)
        print('wrote %s (%dx%d, scale %d). Paint pure-black rectangles over no-go areas, save as PNG (same size).' % (out, w * scale, h * scale, scale))
    elif cmd == 'import':
        yaml_path, edited, prefix = sys.argv[2], sys.argv[3], sys.argv[4]
        image, res, org = read_yaml(yaml_path)
        import os
        base = Image.open(os.path.join(os.path.dirname(yaml_path) or '.', image)).convert('L')
        w, h = base.size
        ed = Image.open(edited).convert('RGB')
        scale = ed.size[0] // w
        if ed.size != (w * scale, h * scale):
            raise SystemExit('edited image size %s is not an integer multiple of the map %s' % (ed.size, (w, h)))
        import numpy as np
        from scipy import ndimage
        a = np.asarray(ed).astype(int)
        black = (a[:, :, 0] < 40) & (a[:, :, 1] < 40) & (a[:, :, 2] < 40)
        # painted BLACK pixels count directly; the map's own walls are black too but only where the
        # map was already occupied, so they add nothing. RED outlines (rectangle tool without fill)
        # are accepted as well: each red blob >= 30 px in both dimensions is filled as a rectangle
        # (the small red marker circles / labels from `export` stay below that size).
        red = (a[:, :, 0] > 150) & (a[:, :, 1] < 90) & (a[:, :, 2] < 90)
        zone = black.copy()
        lab, nlab = ndimage.label(red)
        for i, sl in enumerate(ndimage.find_objects(lab)):
            if sl is None:
                continue
            hh, ww = sl[0].stop - sl[0].start, sl[1].stop - sl[1].start
            if hh >= 30 and ww >= 30:
                zone[sl[0].start:sl[0].stop, sl[1].start:sl[1].stop] = True
                print('red rectangle #%d -> filled %dx%d px' % (i + 1, ww, hh))
        # any painted pixel inside a block -> that map cell is keepout
        cells = zone.reshape(h, scale, w, scale).any(axis=(1, 3))
        mask = base.copy()
        mp = mask.load()
        n = 0
        for r in range(h):
            for c in range(w):
                if cells[r, c] and mp[c, r] != 0:
                    mp[c, r] = 0; n += 1
        mask.save(prefix + '_keepout.pgm')
        with open(prefix + '_keepout.yaml', 'w') as f:
            f.write('image: %s\nmode: trinary\nresolution: %.3f\norigin: [%.3f, %.3f, 0]\nnegate: 0\n'
                    'occupied_thresh: 0.65\nfree_thresh: 0.196\n' % (os.path.basename(prefix) + '_keepout.pgm', res, org[0], org[1]))
        print('wrote %s_keepout.pgm/.yaml: %d new keepout cells (%.2f m2)' % (prefix, n, n * res * res))
    else:
        print(__doc__)



def stations(yaml_path, edited):
    """blue marks on the exported PNG -> station poses. Per station: a BIG blue dot where the robot
    stops and a SMALL blue dot at the board centre; yaw = robot -> board. Prints x y yaw_deg."""
    import os
    import math
    import numpy as np
    from scipy import ndimage
    image, res, org = read_yaml(yaml_path)
    base = Image.open(os.path.join(os.path.dirname(yaml_path) or '.', image))
    w, h = base.size
    a = np.asarray(Image.open(edited).convert('RGB')).astype(int)
    scale = a.shape[1] // w
    blue = (a[:, :, 2] > 150) & (a[:, :, 0] < 90) & (a[:, :, 1] < 120)
    yellow = (a[:, :, 0] > 150) & (a[:, :, 1] > 150) & (a[:, :, 2] < 100)
    results = []
    for cname, cmask in (('blue', blue), ('yellow', yellow)):
        results += _station_blobs(cmask, cname, w, h, scale, res, org)
    return results


def _station_blobs(mask, cname, w, h, scale, res, org):
    import math
    import numpy as np
    from scipy import ndimage
    lab, n = ndimage.label(mask)
    blobs = []
    if n == 0:
        return []
    for i, sl in enumerate(ndimage.find_objects(lab)):
        if sl is None:
            continue
        area = int((lab[sl] == i + 1).sum())
        if area < 12:
            continue
        cy, cx = ndimage.center_of_mass(lab == i + 1)
        mx = org[0] + cx / scale * res
        my = org[1] + (h - cy / scale) * res
        blobs.append((area, mx, my))
    blobs.sort(reverse=True)
    big = blobs[:len(blobs) // 2]
    small = blobs[len(blobs) // 2:]
    out = []
    for area, rx, ry in big:
        _, bx, by = min(small, key=lambda b: (b[1] - rx) ** 2 + (b[2] - ry) ** 2)
        yaw = math.degrees(math.atan2(by - ry, bx - rx))
        out.append((rx, ry, yaw, bx, by))
        print('%s station: robot (%.2f, %.2f) yaw %.0f deg -> board (%.2f, %.2f), gap %.2f m' %
              (cname, rx, ry, yaw, bx, by, math.hypot(bx - rx, by - ry)))
    return out


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'stations':
        stations(sys.argv[2], sys.argv[3])
    else:
        main()
