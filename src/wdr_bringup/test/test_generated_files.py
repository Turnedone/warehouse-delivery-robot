"""Checks that the generated world, map and masks are up to date and agree with each other."""

import os
import subprocess
import sys

import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GENERATOR = os.path.join(PKG, 'scripts', 'generate_warehouse.py')
sys.path.insert(0, os.path.join(PKG, 'scripts'))

from generate_warehouse import FREE, OCCUPIED, speed_pixel  # noqa: E402


def load(rel):
    with open(os.path.join(PKG, rel)) as f:
        return yaml.safe_load(f)


def read_pgm(rel):
    with open(os.path.join(PKG, rel), 'rb') as f:
        assert f.readline().strip() == b'P5'
        width, height = map(int, f.readline().split())
        assert int(f.readline()) == 255
        return width, height, f.read()


def pixel_at(rel, x, y):
    """Value of the map cell containing world point (x, y)."""
    meta = load(rel.replace('.pgm', '.yaml'))
    width, height, data = read_pgm(rel)
    col = int((x - meta['origin'][0]) / meta['resolution'])
    row = int((y - meta['origin'][1]) / meta['resolution'])
    return data[(height - 1 - row) * width + col]


def test_generated_files_are_up_to_date():
    result = subprocess.run([sys.executable, GENERATOR, '--check'], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr


def test_stations_are_in_free_space_and_outside_keepout():
    layout = load('config/warehouse_layout.yaml')
    for st in layout['stations'] + layout['docks']:
        x, y, _ = st['pose']
        assert pixel_at('maps/warehouse.pgm', x, y) == FREE, st['name']
        assert pixel_at('maps/keepout_mask.pgm', x, y) == FREE, st['name']


def test_racks_are_occupied_and_aisles_free():
    assert pixel_at('maps/warehouse.pgm', 6.0, 12.4) == OCCUPIED     # rack_a1
    assert pixel_at('maps/warehouse.pgm', 6.0, 10.4) == FREE         # aisle between A and B
    assert pixel_at('maps/warehouse.pgm', -0.1, 8.0) == OCCUPIED     # west wall


def test_keepout_mask_grows_zones_by_the_margin():
    layout = load('config/warehouse_layout.yaml')
    margin = layout['keepout_margin']
    min_x, min_y, max_x, max_y = layout['keepout_zones'][0]['rect']
    assert pixel_at('maps/keepout_mask.pgm', min_x - margin + 0.03, min_y + 0.5) == OCCUPIED
    assert pixel_at('maps/keepout_mask.pgm', min_x - margin - 0.1, min_y + 0.5) == FREE


def test_speed_mask_values_load_as_the_configured_percent():
    layout = load('config/warehouse_layout.yaml')
    meta = load('maps/speed_mask.yaml')
    assert meta['mode'] == 'scale' and meta['free_thresh'] == 0.0
    for zone in layout['speed_zones']:
        min_x, min_y, max_x, max_y = zone['rect']
        pixel = pixel_at('maps/speed_mask.pgm', (min_x + max_x) / 2, (min_y + max_y) / 2)
        assert pixel == speed_pixel(zone['percent'])
        # map_server, mode scale: occupancy = rint((255 - pixel) / 255 * 100)
        assert round((255 - pixel) / 255 * 100) == zone['percent']
    assert pixel_at('maps/speed_mask.pgm', 2.0, 8.0) == 255  # no limit elsewhere


def test_docks_and_stations_agree():
    stations = load('config/stations.yaml')['stations']
    docks = load('config/docks.yaml')['docks']
    for name, dock in docks.items():
        assert stations[name]['type'] == 'dock'
        assert [stations[name]['x'], stations[name]['y']] == dock['pose'][:2]
