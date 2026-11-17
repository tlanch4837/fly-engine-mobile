# Bakes one airport for the basic build (src_basic, fly_basic.exe): its runways from OpenStreetMap (ODbL: credit "OpenStreetMap
# contributors"), fetched once at development time from the Overpass API, into data/airports/<icao>.json. The engine flattens the
# ground round the airport and draws each runway as a flat strip with its markings; the satellite imagery shows everything else
# (taxiways, aprons, terminals), so no 3D is needed.
# Usage: python tools/bake_airport.py KDTW
# Format: {icao, name, lat, lon, flat_radius_m, runways: [{ref, width_m, a: [lat, lon], b: [lat, lon], ends: ["04L", "22R"]}]}
#   a is the end named ends[0] (its threshold), b the other end.
import json, math, os, sys, time
import requests

icao = sys.argv[1].upper()
Q = f"""[out:json][timeout:120];
(way["aeroway"="aerodrome"]["icao"="{icao}"];relation["aeroway"="aerodrome"]["icao"="{icao}"];)->.ad;
.ad out center tags;
.ad map_to_area->.a;
way["aeroway"="runway"](area.a);
out geom tags;"""
MIRRORS = ["https://overpass-api.de/api/interpreter", "https://lz4.overpass-api.de/api/interpreter", "https://overpass.kumi.systems/api/interpreter"]
for attempt in range(6):
    url = MIRRORS[attempt % len(MIRRORS)]
    try:
        r = requests.post(url, data={"data": Q}, headers={"User-Agent": "fly-engine-data/1.0 (offline extract)"}, timeout=180)
        r.raise_for_status()
        d = r.json()
        break
    except Exception as e:
        print("overpass retry", attempt, url, str(e)[:100])
        time.sleep(5 + attempt * 5)
else:
    sys.exit("overpass unavailable")

ad = next((e for e in d["elements"] if e.get("tags", {}).get("aeroway") == "aerodrome"), None)
if not ad: sys.exit(f"{icao}: no aerodrome with that icao tag in OpenStreetMap")
c = ad.get("center") or {}
lat0, lon0 = c.get("lat"), c.get("lon")
name = ad["tags"].get("name", icao)

def local(lat, lon):  # metres east, north of the centre (fine over an airport)
    return (math.radians(lon - lon0) * 6371000.0 * math.cos(math.radians(lat0)), math.radians(lat - lat0) * 6371000.0)

def heading(a, b):  # degrees true from a to b
    (x0, y0), (x1, y1) = local(*a), local(*b)
    return math.degrees(math.atan2(x1 - x0, y1 - y0)) % 360.0

runways = []
for e in d["elements"]:
    t = e.get("tags", {})
    if t.get("aeroway") != "runway" or e["type"] != "way": continue
    g = [(q["lat"], q["lon"]) for q in e.get("geometry", []) if q]
    if len(g) < 2: continue
    if t.get("area") == "yes" or (g[0] == g[-1] and len(g) > 3):  # drawn as an outline: its long axis
        pts = [local(*p) for p in g[:-1]]
        best = max(((i, j) for i in range(len(pts)) for j in range(i + 1, len(pts))), key=lambda ij: math.dist(pts[ij[0]], pts[ij[1]]))
        a, b = g[best[0]], g[best[1]]
    else:
        a, b = g[0], g[-1]
    try: width = float(str(t.get("width", "45")).split()[0])
    except ValueError: width = 45.0
    ref = t.get("ref", "")
    ends = [s.strip() for s in ref.split("/")] if "/" in ref else []
    # the end whose number matches the heading from it: a runway "04" is landed on heading ~040
    h = heading(a, b)
    if ends:
        n0 = int("".join(ch for ch in ends[0] if ch.isdigit()) or 0) * 10
        if abs((h - n0 + 180) % 360 - 180) > 90: a, b = b, a
    else:
        h1 = round(heading(a, b) / 10) % 36 or 36
        ends = [f"{h1:02d}", f"{(h1 + 18 - 1) % 36 + 1:02d}"]
    length = math.dist(local(*a), local(*b))
    if length < 300: continue
    runways.append({"ref": ref or "/".join(ends), "width_m": width, "length_m": round(length), "a": [a[0], a[1]], "b": [b[0], b[1]], "ends": ends})

# one runway drawn as several ways (split at a crossing or a resurfaced part): join them end to end along their axis
merged = {}
for rw in runways:
    k = rw["ref"]
    if k not in merged: merged[k] = rw; continue
    m = merged[k]
    (ax, ay), (bx, by) = local(*m["a"]), local(*m["b"])
    L = math.hypot(bx - ax, by - ay); ux, uy = (bx - ax) / L, (by - ay) / L
    along = lambda p: (local(*p)[0] - ax) * ux + (local(*p)[1] - ay) * uy
    pts = [m["a"], m["b"], rw["a"], rw["b"]]
    m["a"], m["b"] = min(pts, key=along), max(pts, key=along)
    m["width_m"] = max(m["width_m"], rw["width_m"])
    m["length_m"] = round(math.dist(local(*m["a"]), local(*m["b"])))
runways = list(merged.values())
if not runways: sys.exit(f"{icao}: no runways found")
reach = max(math.hypot(*local(*p)) for rw in runways for p in (rw["a"], rw["b"]))
out = {"icao": icao, "name": name, "lat": lat0, "lon": lon0, "flat_radius_m": round(reach + 400.0),
       "runways": runways, "source": "OpenStreetMap contributors (ODbL)"}
here = os.path.dirname(os.path.abspath(__file__))
dst = os.path.join(here, "..", "data", "airports")
os.makedirs(dst, exist_ok=True)
with open(os.path.join(dst, icao.lower() + ".json"), "w") as f: json.dump(out, f, indent=1)
print(f"{icao} {name}: {len(runways)} runways, flat {out['flat_radius_m']} m:", ", ".join(f"{r['ref']} {r['length_m']}x{r['width_m']:.0f}" for r in runways))
