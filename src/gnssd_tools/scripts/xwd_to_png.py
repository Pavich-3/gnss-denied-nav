#!/usr/bin/env python3
"""
Convert X window dumps (xwd, e.g. the Step 8 RViz screenshots) to PNG.

xwd ships with X11 and can capture one window by its title
(`xwd -name gnssd_nav2_rviz -out shot.xwd`), but nothing on this machine
converts its format (no ImageMagick). The format is simple: a header of
big-endian 32-bit fields, the window name, a colour table, then the pixels
row by row. Only the TrueColor 24/32-bit case of a normal X11 desktop is
handled.

Run with the Python of evo's venv (numpy + matplotlib):
  ~/.venvs/evo/bin/python src/gnssd_tools/scripts/xwd_to_png.py docs/step8/*.xwd
Writes <file>.png next to every <file>.xwd.
"""

import argparse
from pathlib import Path
import struct

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402  (must come after matplotlib.use)
import numpy as np  # noqa: E402

FIELDS = ('header_size', 'file_version', 'pixmap_format', 'pixmap_depth', 'width', 'height',
          'xoffset', 'byte_order', 'bitmap_unit', 'bitmap_bit_order', 'bitmap_pad',
          'bits_per_pixel', 'bytes_per_line', 'visual_class', 'red_mask', 'green_mask',
          'blue_mask', 'bits_per_rgb', 'colormap_entries', 'ncolors', 'window_width',
          'window_height', 'window_x', 'window_y', 'window_border_width')
COLOR_ENTRY = 12   # bytes per XWDColor


def channel(pixels, mask):
    """Extract one 8-bit colour channel given its bit mask."""
    shift = (mask & -mask).bit_length() - 1
    return ((pixels & mask) >> shift).astype(np.uint8)


def convert(path):
    data = Path(path).read_bytes()
    header = dict(zip(FIELDS, struct.unpack('>25I', data[:100])))
    if header['bits_per_pixel'] not in (24, 32):
        raise SystemExit(f'{path}: {header["bits_per_pixel"]} bits per pixel not supported')
    start = header['header_size'] + header['ncolors'] * COLOR_ENTRY
    width, height = header['width'], header['height']
    rows = np.frombuffer(data[start:start + height * header['bytes_per_line']], dtype=np.uint8)
    rows = rows.reshape(height, header['bytes_per_line'])
    step = header['bits_per_pixel'] // 8
    raw = rows[:, :width * step].reshape(height, width, step).astype(np.uint32)
    if header['byte_order'] == 0:         # LSBFirst
        pixels = sum(raw[:, :, k] << (8 * k) for k in range(step))
    else:
        pixels = sum(raw[:, :, k] << (8 * (step - 1 - k)) for k in range(step))
    image = np.dstack([channel(pixels, header[f'{c}_mask']) for c in ('red', 'green', 'blue')])
    out = Path(path).with_suffix('.png')
    plt.imsave(out, image)
    return out, width, height


def main():
    parser = argparse.ArgumentParser(description='xwd window dumps -> PNG')
    parser.add_argument('files', nargs='+')
    for path in parser.parse_args().files:
        out, width, height = convert(path)
        print(f'{path} -> {out} ({width} x {height})')


if __name__ == '__main__':
    main()
