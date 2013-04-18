#!/usr/bin/env python

import subprocess
import sys
import re
import argparse
from os.path import basename, dirname, join, normpath, splitext

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    Generate ROS map description yaml file for given .world file.
    ''')
    parser.add_argument('world', help='world filename')
    args = parser.parse_args()
    world_file = args.world

    try:
        world = open(world_file, 'r').read().replace('\n', '')
        m = re.search('floorplan.*?size \[(\d+(?:\.\d+)?) (\d+(?:\.\d+)?) .*?'
                      'bitmap "(.*?)"', world)
        x, y, bitmap = float(m.group(1)), float(m.group(2)), m.group(3)
        bitmap = normpath(join(dirname(world_file), bitmap))
        print 'World size: %.3f x %.3f' % (x, y)
        print 'World bitmap: %s' % bitmap
    except IOError:
        sys.exit('Failed to open world description file.')
    except (ValueError, AttributeError):
        sys.exit('Failed to parse world description file.')
    try:
        desc = subprocess.check_output(['file', bitmap])
        m = re.match('.*?(\d+) x (\d+).*?', desc)
        width, height = int(m.group(1)), int(m.group(2))
        print 'Bitmap size: %i X %i' % (width, height)
    except subprocess.CalledProcessError:
        sys.exit('Failed to read bitmap file.')
    except AttributeError:
        sys.exit('Failed to determine bitmap size.')

    cell_width = x / width
    cell_height = y / height
    if abs(cell_width - cell_height) > 0.001:
        sys.exit('World and bitmap sizes are not proportional.')
    resolution = max(cell_width, cell_height)

    description_file = splitext(basename(world_file))[0] + '.yaml'
    with open(description_file, 'w') as f:
        f.write('image: %s\n' % bitmap)
        f.write('resolution: %.5f\n' % resolution)
        f.write('origin: [%.2f, %.2f, 0.00]\n' % (-x / 2, -y / 2))
        f.write('occupied_thresh: 0.35\n')
        f.write('free_thresh: 0.196\n')
        f.write('negate: 0')
