#!/usr/bin/env python3

r"""
Satellite imagery under the track, from the Mission Control Station's cache.

MCS keeps its slippy-map tiles as a FLAT directory of PNGs - no z/x/y nesting,
no index, no expiry - at `~/.config/blueboat_mcs/tile_cache/`, one file per
tile named `{z}_{x}_{y}.png`, fetched from Esri World Imagery. Since every
mission is flown from MCS, the area a log covers is almost always already
cached, so this app reads that cache directly and draws offline.

TWO THINGS TO GET RIGHT, BOTH OF WHICH FAIL SILENTLY IF YOU GET THEM WRONG.

1. THE URL IS `{z}/{y}/{x}` AND THE FILENAME IS `{z}_{x}_{y}`. Esri's World
   Imagery endpoint puts row before column; MCS's cache filename puts column
   before row. Swap them and you do not get an error - you get a perfectly
   valid tile of somewhere else on Earth, drawn confidently under your track.
   Both are written with keyword formatting below so the ordering is explicit.

2. THE FILES ARE NOT ACTUALLY PNGs. Esri's World Imagery endpoint serves
   JPEG, and MCS writes the reply body to a name ending `.png` without looking
   at it. So the cache is full of JPEG bytes under a `.png` name, and anything
   that decodes by EXTENSION fails on it - `matplotlib.image.imread` special-
   cases `.png` and raises `SyntaxError: not a PNG file` on every single tile.
   Decoding is therefore done by CONTENT, through PIL, which sniffs the magic
   bytes. (PIL is not a new dependency: matplotlib requires Pillow.)

3. WRITES ARE ATOMIC. MCS writes its own tiles with a plain `write_bytes`, so
   a fetch here that landed mid-read would hand MCS a truncated PNG. Every
   fetch writes a `.part` file next to the target and `os.replace`s it, which
   is atomic on the same filesystem - the two processes can share the cache.

CM-3: no module modifies or imports a neighbour's package. The three slippy-map
functions below are COPIED from `BlueBoat-MCS/mcs/core/geo.py` (they are ~25
lines of pure arithmetic) rather than imported, so BlueBoat-Control carries no
path dependency on BlueBoat-MCS. If MCS ever changes its tile URL or cache
directory, this reads the change out of its config.json; if it changes the
projection maths, the copies below are wrong and must be re-copied.
"""

import json
import math
import os
import threading

try:                                     # Qt is optional at import time, so
    from PySide6.QtCore import QObject, Signal      # the module stays testable
except ImportError:                                 # headlessly without it.
    QObject, Signal = object, None

try:
    import requests
except ImportError:                                 # fetching simply goes off
    requests = None

import numpy as np
from PIL import Image

# MCS's own defaults, mirrored - see BlueBoat-MCS/mcs/config/settings.py.
DEFAULT_CONFIG_DIR = os.path.join(os.path.expanduser("~"), ".config",
                                  "blueboat_mcs")
DEFAULT_TILE_URL = ("https://server.arcgisonline.com/ArcGIS/rest/services/"
                    "World_Imagery/MapServer/tile/{z}/{y}/{x}")
DEFAULT_CACHE_DIR = os.path.join(DEFAULT_CONFIG_DIR, "tile_cache")
DEFAULT_MAX_ZOOM = 19

TILE_PX = 256
MIN_ZOOM = 3
MAX_TILES = 64          # MCS's own cap; past it the view is too wide to tile
USER_AGENT = "BlueBoatMissionControl/1.0"
FETCH_TIMEOUT_S = 6.0
MEMORY_TILES = 256      # decoded tiles held in RAM


# ---------------------------------------------------------------------------
# Slippy-map arithmetic - COPIED verbatim from BlueBoat-MCS/mcs/core/geo.py
# ---------------------------------------------------------------------------

def latlon_to_tile_xy(lat, lon, zoom):
    """Fractional slippy-map tile coordinates for a lat/lon at a zoom level."""
    lat = max(min(lat, 85.05112878), -85.05112878)
    n = 2.0 ** zoom
    x = (lon + 180.0) / 360.0 * n
    lat_r = math.radians(lat)
    y = (1.0 - math.log(math.tan(lat_r) + 1.0 / math.cos(lat_r)) / math.pi) / 2.0 * n
    return x, y


def tile_xy_to_latlon(x, y, zoom):
    n = 2.0 ** zoom
    lon = x / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * y / n))))
    return lat, lon


def metres_per_pixel(lat, zoom, tile_px=TILE_PX):
    return (2 * math.pi * 6378137.0 * math.cos(math.radians(lat))) / (tile_px * 2 ** zoom)


# ---------------------------------------------------------------------------
# The provider
# ---------------------------------------------------------------------------

class TileProvider(QObject if Signal is not None else object):
    """Cache-first tile reader with an optional background filler.

    `tiles_for` never blocks and never raises: it returns the tiles that are on
    disk right now, and queues the rest. When a queued tile lands, `tileReady`
    fires and the canvas redraws with one more tile in place. A tile that will
    not download (no network, no `requests`, a 404 at max zoom over open water)
    is simply never drawn - a hole in the imagery, never an error box over a
    perfectly good track.
    """

    if Signal is not None:
        tileReady = Signal()

    def __init__(self, fetch=True, parent=None):
        if Signal is not None:
            QObject.__init__(self, parent)
        cfg = _read_mcs_config()
        self.tile_url = cfg["tile_url"]
        self.cache_dir = cfg["cache_dir"]
        self.max_zoom = cfg["max_zoom"]
        self.fetch_enabled = bool(fetch) and requests is not None

        self._memory = {}          # (z, x, y) -> decoded RGB(A) array
        self._order = []           # crude LRU
        self._missing = set()      # tried and failed, do not re-queue forever
        self._queued = set()
        self._lock = threading.Lock()
        self._worker = None

    # -- reading ----------------------------------------------------------

    def cache_path(self, z, x, y):
        """MCS's flat name. Column then row - the opposite of the URL."""
        return os.path.join(self.cache_dir, "%d_%d_%d.png" % (z, x, y))

    def choose_zoom(self, lat_c, px_per_m):
        """MCS's own rule: the coarsest zoom whose pixels are no larger than
        the screen's, with 1.2 of slack."""
        if not math.isfinite(px_per_m) or px_per_m <= 0:
            return MIN_ZOOM
        for z in range(MIN_ZOOM, self.max_zoom + 1):
            if metres_per_pixel(lat_c, z) * px_per_m <= 1.2:
                return z
        return self.max_zoom

    def tiles_for(self, lon_lo, lon_hi, lat_lo, lat_hi, canvas_px):
        """Every cached tile covering the view, as (extent, image) pairs.

        `extent` is `[lon_west, lon_east, lat_south, lat_north]` of that tile,
        ready for `imshow(..., extent=...)`: each tile is placed by its OWN
        corners in degrees, so plotting Web-Mercator imagery on lon/lat axes
        distorts only WITHIN one tile - well under a pixel at survey scale -
        and the track panel keeps the familiar degree axes the PNG report has.
        """
        if lon_hi <= lon_lo or lat_hi <= lat_lo or canvas_px <= 0:
            return []
        lat_c = 0.5 * (lat_lo + lat_hi)
        span_m = math.radians(lon_hi - lon_lo) * 6378137.0 * math.cos(math.radians(lat_c))
        if span_m <= 0:
            return []
        zoom = self.choose_zoom(lat_c, canvas_px / span_m)

        x0, y0 = latlon_to_tile_xy(lat_hi, lon_lo, zoom)      # north-west
        x1, y1 = latlon_to_tile_xy(lat_lo, lon_hi, zoom)      # south-east
        ix0, ix1 = int(math.floor(x0)), int(math.floor(x1))
        iy0, iy1 = int(math.floor(y0)), int(math.floor(y1))
        n = 2 ** zoom
        if (ix1 - ix0 + 1) * (iy1 - iy0 + 1) > MAX_TILES:
            return []

        out = []
        for ix in range(ix0, ix1 + 1):
            for iy in range(iy0, iy1 + 1):
                if not (0 <= ix < n and 0 <= iy < n):
                    continue
                image = self._tile(zoom, ix, iy)
                if image is None:
                    continue
                lat_n, lon_w = tile_xy_to_latlon(ix, iy, zoom)
                lat_s, lon_e = tile_xy_to_latlon(ix + 1, iy + 1, zoom)
                out.append(([lon_w, lon_e, lat_s, lat_n], image))
        return out

    def _tile(self, z, x, y):
        key = (z, x, y)
        if key in self._memory:
            return self._memory[key]
        path = self.cache_path(z, x, y)
        if os.path.isfile(path):
            try:
                # By content, not by extension - the cache holds JPEG bytes
                # under a .png name (see the module docstring).
                with Image.open(path) as handle:
                    image = np.asarray(handle.convert("RGB"))
            except Exception:                              # noqa: BLE001
                return None                                # corrupt/partial file
            self._remember(key, image)
            return image
        self._queue(key)
        return None

    def _remember(self, key, image):
        self._memory[key] = image
        self._order.append(key)
        while len(self._order) > MEMORY_TILES:
            self._memory.pop(self._order.pop(0), None)

    # -- fetching ---------------------------------------------------------

    def _queue(self, key):
        if not self.fetch_enabled:
            return
        with self._lock:
            if key in self._queued or key in self._missing:
                return
            self._queued.add(key)
            if self._worker is None or not self._worker.is_alive():
                self._worker = threading.Thread(target=self._drain, daemon=True)
                self._worker.start()

    def _drain(self):
        while True:
            with self._lock:
                if not self._queued:
                    self._worker = None
                    return
                key = self._queued.pop()
            if self._download(key):
                if Signal is not None:
                    try:
                        self.tileReady.emit()
                    except RuntimeError:          # the window went away
                        return

    def _download(self, key):
        z, x, y = key
        url = self.tile_url.format(z=z, x=x, y=y)      # NOTE: {z}/{y}/{x}
        target = self.cache_path(z, x, y)              # NOTE: {z}_{x}_{y}
        try:
            os.makedirs(self.cache_dir, exist_ok=True)
            reply = requests.get(url, timeout=FETCH_TIMEOUT_S,
                                 headers={"User-Agent": USER_AGENT})
            if reply.status_code != 200 or not reply.content:
                with self._lock:
                    self._missing.add(key)
                return False
            # Atomic, so a Mission Control Station reading the same cache at
            # the same moment can never see a half-written tile.
            partial = target + ".part"
            with open(partial, "wb") as fh:
                fh.write(reply.content)
            os.replace(partial, target)
            return True
        except Exception:                                  # noqa: BLE001
            with self._lock:
                self._missing.add(key)
            return False


def _read_mcs_config():
    """MCS's map settings, so a relocated cache is followed rather than missed."""
    cfg = {"tile_url": DEFAULT_TILE_URL,
           "cache_dir": DEFAULT_CACHE_DIR,
           "max_zoom": DEFAULT_MAX_ZOOM}
    path = os.path.join(DEFAULT_CONFIG_DIR, "config.json")
    try:
        with open(path) as fh:
            raw = json.load(fh).get("map", {})
        cfg["tile_url"] = raw.get("tile_url", cfg["tile_url"])
        cfg["cache_dir"] = os.path.expanduser(
            raw.get("tile_cache_dir", cfg["cache_dir"]))
        cfg["max_zoom"] = int(raw.get("tile_max_zoom", cfg["max_zoom"]))
    except (OSError, ValueError, AttributeError):
        pass
    return cfg
