#!/usr/bin/env python3
# encoding: utf-8
"""make_filter_mask.py -- draw Nav2 costmap-filter masks (keepout / speed zones) onto a map.

Deployment-style "map editing": zones are given as rectangles in MAP coordinates (metres,
the same frame AMCL reports in), painted into mask images that share the map's resolution
and origin. Nav2 loads them through nav2_map_server + costmap_filter_info_server
(src/jr_nav/launch/filters.launch.py) and the costmap filters do the rest.

Encoding (what Nav2's map_io does with each mask):
  keepout mask  mode=trinary  : black (0) -> occupancy 100 -> KeepoutFilter marks LETHAL;
                                white -> 0 (no effect); grey -> -1 (ignored)
  speed mask    mode=scale, free_thresh 0.0, occupied_thresh 1.0 :
                                pixel p -> value v = rint((1 - p/255) * 100);
                                SpeedFilter limit% = base + multiplier * v = 100 - v
                                so a zone limited to PCT% is painted with p = round(255*PCT/100);
                                white (255) -> v 0 -> "no limit"

Usage (pure python3, no numpy/cv2 -- runs on the robot, in WSL, anywhere):
  python3 make_filter_mask.py MAP_YAML OUT_DIR [--name zones] [--canvas map|blank] [--no-png]
        --keepout x1,y1,x2,y2 [--keepout ...]      # forbidden rectangles (metres)
        --speed PCT:x1,y1,x2,y2 [--speed ...]      # speed-limit rectangles, PCT = allowed % of max
Example:
  python3 make_filter_mask.py maps/apt_loop_20260903.yaml maps \
      --keepout 1.8,-0.9,2.6,-0.1 --speed 30:3.3,1.4,4.5,2.2
Writes OUT_DIR/<name>_keepout.{pgm,yaml}, OUT_DIR/<name>_speed.{pgm,yaml}, OUT_DIR/<name>_preview.png
"""
import os
import struct
import sys
import zlib


def read_pgm(path):
    data = open(path, 'rb').read()
    pos = 0
    tokens = []
    while len(tokens) < 4:
        while pos < len(data) and data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            while data[pos:pos + 1] not in (b'\n', b''):
                pos += 1
            continue
        start = pos
        while pos < len(data) and not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    if tokens[0] != b'P5':
        raise SystemExit('%s: only binary P5 PGM supported (got %r)' % (path, tokens[0]))
    w, h, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    if maxval != 255:
        raise SystemExit('%s: maxval %d unsupported' % (path, maxval))
    pos += 1                                   # single whitespace after maxval
    px = bytearray(data[pos:pos + w * h])
    if len(px) != w * h:
        raise SystemExit('%s: short pixel data' % path)
    return w, h, px


def write_pgm(path, w, h, px):
    with open(path, 'wb') as f:
        f.write(b'P5\n# CREATOR: make_filter_mask.py\n%d %d\n255\n' % (w, h))
        f.write(bytes(px))


def write_png_rgb(path, w, h, rgb):
    raw = b''.join(b'\x00' + bytes(rgb[y * w * 3:(y + 1) * w * 3]) for y in range(h))

    def chunk(tag, body):
        return struct.pack('>I', len(body)) + tag + body + struct.pack('>I', zlib.crc32(tag + body) & 0xffffffff)
    png = b'\x89PNG\r\n\x1a\n'
    png += chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
    png += chunk(b'IDAT', zlib.compress(raw, 9))
    png += chunk(b'IEND', b'')
    open(path, 'wb').write(png)


def load_map_yaml(path):
    """minimal reader for the 6 keys map_server writes (no PyYAML dependency)"""
    d = {}
    for line in open(path):
        line = line.split('#', 1)[0].strip()
        if ':' not in line:
            continue
        k, v = line.split(':', 1)
        d[k.strip()] = v.strip()
    res = float(d['resolution'])
    origin = [float(t) for t in d['origin'].strip('[]').split(',')]
    return d['image'], res, origin


def rect_to_pixels(rect, w, h, res, origin):
    """map-frame rectangle (x1,y1,x2,y2 metres) -> inclusive pixel box (c0,r0,c1,r1), row 0 = top"""
    x1, y1, x2, y2 = rect
    xa, xb = sorted((x1, x2))
    ya, yb = sorted((y1, y2))
    c0 = int((xa - origin[0]) / res)
    c1 = int((xb - origin[0]) / res) - 1
    rb = int((ya - origin[1]) / res)                 # grid rows (0 = bottom)
    rt = int((yb - origin[1]) / res) - 1
    r0 = h - 1 - rt                                  # image rows (0 = top)
    r1 = h - 1 - rb
    c0, c1 = max(0, c0), min(w - 1, c1)
    r0, r1 = max(0, r0), min(h - 1, r1)
    if c1 < c0 or r1 < r0:
        raise SystemExit('rectangle %s lies outside the map (map x %.2f..%.2f, y %.2f..%.2f)' %
                         (rect, origin[0], origin[0] + w * res, origin[1], origin[1] + h * res))
    return c0, r0, c1, r1


def paint(px, w, box, value):
    c0, r0, c1, r1 = box
    for r in range(r0, r1 + 1):
        for c in range(c0, c1 + 1):
            px[r * w + c] = value


def main(argv):
    args = list(argv)
    if len(args) < 2 or args[0].startswith('-'):
        print(__doc__)
        return 2
    map_yaml, out_dir = args[0], args[1]
    name, canvas, want_png = 'zones', 'map', True
    keepouts, speeds = [], []
    i = 2
    while i < len(args):
        a = args[i]
        if a == '--name':
            name = args[i + 1]
            i += 2
        elif a == '--canvas':
            canvas = args[i + 1]
            i += 2
        elif a == '--no-png':
            want_png = False
            i += 1
        elif a == '--keepout':
            keepouts.append(tuple(float(t) for t in args[i + 1].split(',')))
            i += 2
        elif a == '--speed':
            pct, rect = args[i + 1].split(':', 1)
            pct = float(pct)
            if not 5.0 <= pct <= 100.0:
                raise SystemExit('--speed PCT must be 5..100 (got %s)' % pct)
            speeds.append((pct, tuple(float(t) for t in rect.split(','))))
            i += 2
        else:
            raise SystemExit('unknown argument %s' % a)
    if not keepouts and not speeds:
        raise SystemExit('give at least one --keepout or --speed zone')

    image, res, origin = load_map_yaml(map_yaml)
    w, h, map_px = read_pgm(os.path.join(os.path.dirname(map_yaml) or '.', image))
    os.makedirs(out_dir, exist_ok=True)
    print('map %s: %dx%d px, %.3f m/px, origin (%.3f, %.3f) -> covers x %.2f..%.2f, y %.2f..%.2f' %
          (image, w, h, res, origin[0], origin[1], origin[0], origin[0] + w * res, origin[1], origin[1] + h * res))

    # ---- keepout mask: trinary, canvas = map copy (walls stay lethal, harmless) or blank white
    kp = bytearray(map_px) if canvas == 'map' else bytearray(b'\xff' * (w * h))
    kboxes = []
    for rect in keepouts:
        box = rect_to_pixels(rect, w, h, res, origin)
        paint(kp, w, box, 0)
        kboxes.append(box)
        print('keepout %s -> pixels cols %d..%d rows %d..%d (%d cells)' %
              (rect, box[0], box[2], box[1], box[3], (box[2] - box[0] + 1) * (box[3] - box[1] + 1)))
    kpgm = '%s_keepout.pgm' % name
    write_pgm(os.path.join(out_dir, kpgm), w, h, kp)
    with open(os.path.join(out_dir, '%s_keepout.yaml' % name), 'w') as f:
        f.write('image: %s\nmode: trinary\nresolution: %.3f\norigin: [%.3f, %.3f, 0]\nnegate: 0\n'
                'occupied_thresh: 0.65\nfree_thresh: 0.196\n' % (kpgm, res, origin[0], origin[1]))

    # ---- speed mask: scale mode, white = no limit, grey level encodes the allowed percentage
    sp = bytearray(b'\xff' * (w * h))
    sboxes = []
    for pct, rect in speeds:
        box = rect_to_pixels(rect, w, h, res, origin)
        p = int(round(255.0 * pct / 100.0))
        v = int(round((1.0 - p / 255.0) * 100.0))
        paint(sp, w, box, p)
        sboxes.append((box, pct))
        print('speed %.0f%% %s -> pixel %d (mask value %d, limit %d%%), cols %d..%d rows %d..%d' %
              (pct, rect, p, v, 100 - v, box[0], box[2], box[1], box[3]))
    spgm = '%s_speed.pgm' % name
    write_pgm(os.path.join(out_dir, spgm), w, h, sp)
    with open(os.path.join(out_dir, '%s_speed.yaml' % name), 'w') as f:
        f.write('image: %s\nmode: scale\nresolution: %.3f\norigin: [%.3f, %.3f, 0]\nnegate: 0\n'
                'occupied_thresh: 1.0\nfree_thresh: 0.0\n' % (spgm, res, origin[0], origin[1]))

    # ---- preview: map in grey, keepout red, speed yellow, x4
    if want_png:
        s = 4
        rgb = bytearray(w * s * h * s * 3)
        for r in range(h):
            for c in range(w):
                g = map_px[r * w + c]
                col = (g, g, g)
                for box in kboxes:
                    if box[0] <= c <= box[2] and box[1] <= r <= box[3]:
                        col = (220, 40, 40)
                for box, pct in sboxes:
                    if box[0] <= c <= box[2] and box[1] <= r <= box[3]:
                        col = (235, 200, 40)
                for dr in range(s):
                    base = ((r * s + dr) * w * s + c * s) * 3
                    for dc in range(s):
                        rgb[base + dc * 3:base + dc * 3 + 3] = bytes(col)
        write_png_rgb(os.path.join(out_dir, '%s_preview.png' % name), w * s, h * s, rgb)
    print('wrote %s/%s_{keepout,speed}.{pgm,yaml}%s' % (out_dir, name, ' + preview.png' if want_png else ''))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
