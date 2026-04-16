#!/usr/bin/env python3
"""
hotel_map_generator.py  (v2)
============================
Gazebo world geometrisinden doğrudan PGM harita üretir.
AMCL asimetrisi: Kat 1 ve Kat 2 FARKLI haritalar alır:
  floor1.pgm → Resepsiyon bankosu (asansör çıkışı yanı)
  floor2.pgm → Beton kolonlar + saksılar (asansör çıkışı yanı)
Bu sayede AMCL her kat geçişinden sonra doğru katı tanır.

Kullanım:
  python3 hotel_map_generator.py
  → floor1.pgm, floor1.yaml, floor2.pgm, floor2.yaml
"""

import os

# ─── World parametreleri (hotel_world_generator.py ile SENKRON) ──────────────
ELEV_X       =  8.0
ELEV_Y       =  0.0
ELEV_W       =  2.5
ELEV_D       =  2.5
ELEV_DOOR_X  =  ELEV_X - ELEV_D / 2    # 6.75

CORR_W       = 10.0
CORR_HW      = CORR_W / 2
CORR_START_X = -12.0
WALL_T       =  0.2
DOOR_POSITIONS_X = [-9.5, -6.5, -3.5, -0.5]
DOOR_W       =  1.5
POST_T       =  0.15

# Asimetri öğe koordinatları (world_generator ile SENKRON)
RECEP_CX = ELEV_DOOR_X - 1.8   # = 4.95  (Kat 1 resepsiyon bankosu X merkezi)
RECEP_CY = 4.4
RECEP_SX = 2.2                  # Banka uzunluğu
RECEP_SY = 0.5                  # Banka derinliği
RECEP_WING_OX = RECEP_CX + 0.85
RECEP_WING_OY = RECEP_CY - 0.45
RECEP_WING_SX = 0.5
RECEP_WING_SY = 0.7

COL_X    = ELEV_DOOR_X - 1.2   # = 5.55  (Kat 2 kolon X)
COL_R    = 0.22                 # Kolon yarıçapı
COL_Y_L  = -4.4
COL_Y_R  =  4.4
PLANT_R  = 0.25                 # Saksı yarıçapı
PLANT_OX = COL_X - 0.5         # Saksı X (kolon önünde)

# ─── Harita parametreleri ────────────────────────────────────────────────────
RESOLUTION = 0.05
MAP_MIN_X  = -13.5
MAP_MAX_X  =  11.5
MAP_MIN_Y  =  -6.5
MAP_MAX_Y  =   6.5

FREE     = 254
OCCUPIED =   0
UNKNOWN  = 205

MAP_W = int((MAP_MAX_X - MAP_MIN_X) / RESOLUTION)
MAP_H = int((MAP_MAX_Y - MAP_MIN_Y) / RESOLUTION)

FLOOR1_OBSTACLES = []

FLOOR2_OBSTACLES = []


# ─── Koordinat dönüşümleri ────────────────────────────────────────────────────
def world_to_pixel(wx, wy):
    col = int((wx - MAP_MIN_X) / RESOLUTION)
    row = MAP_H - 1 - int((wy - MAP_MIN_Y) / RESOLUTION)
    return row, col


def fill_rect(grid, wx0, wy0, wx1, wy1, value):
    """World koordinatlarıyla dikdörtgen doldur."""
    r0, c0 = world_to_pixel(min(wx0, wx1), max(wy0, wy1))
    r1, c1 = world_to_pixel(max(wx0, wx1), min(wy0, wy1))
    r0, r1 = max(0, min(r0, r1)), min(MAP_H - 1, max(r0, r1))
    c0, c1 = max(0, min(c0, c1)), min(MAP_W - 1, max(c0, c1))
    for r in range(r0, r1 + 1):
        for c in range(c0, c1 + 1):
            grid[r][c] = value


def fill_circle(grid, cx, cy, radius, value):
    """Daire doldur (kolon için)."""
    px_r = int(radius / RESOLUTION) + 1
    rc, cc = world_to_pixel(cx, cy)
    for dr in range(-px_r, px_r + 1):
        for dc in range(-px_r, px_r + 1):
            # Dünya koordinatına geri çevir ve mesafeyi kontrol et
            # (yaklaşık — kare maskeyle yeterli)
            if dr * dr + dc * dc <= (px_r) ** 2:
                r, c = rc + dr, cc + dc
                if 0 <= r < MAP_H and 0 <= c < MAP_W:
                    grid[r][c] = value


# ─── Temel koridor geometrisi (her iki katta ortak) ──────────────────────────
def generate_base_grid():
    grid = [[UNKNOWN] * MAP_W for _ in range(MAP_H)]

    # 1. Koridor serbest alanı
    fill_rect(grid, CORR_START_X, -CORR_HW, ELEV_DOOR_X, CORR_HW, FREE)

    # 2. Asansör nişi serbest alanı
    fill_rect(grid, ELEV_DOOR_X, -ELEV_W/2, ELEV_X + ELEV_D/2, ELEV_W/2, FREE)

    # 3. Batı duvarı
    fill_rect(grid,
              CORR_START_X - WALL_T, -CORR_HW,
              CORR_START_X,           CORR_HW,
              OCCUPIED)

    # 4. Kuzey ve güney koridor duvarları (kapı açıklıklı)
    prev_x = CORR_START_X
    for dx in DOOR_POSITIONS_X:
        dl = dx - DOOR_W / 2 - POST_T
        dr = dx + DOOR_W / 2 + POST_T
        if dl > prev_x + 0.01:
            fill_rect(grid, prev_x, CORR_HW,       dl, CORR_HW + WALL_T,       OCCUPIED)
            fill_rect(grid, prev_x, -CORR_HW - WALL_T, dl, -CORR_HW,           OCCUPIED)
        # Pervazlar
        for px in [dx - DOOR_W/2 - POST_T, dx + DOOR_W/2]:
            fill_rect(grid, px, CORR_HW,          px + POST_T, CORR_HW + WALL_T,       OCCUPIED)
            fill_rect(grid, px, -CORR_HW - WALL_T, px + POST_T, -CORR_HW,              OCCUPIED)
        prev_x = dr

    # Son segmentten asansöre
    if ELEV_DOOR_X > prev_x + 0.01:
        fill_rect(grid, prev_x, CORR_HW,          ELEV_DOOR_X, CORR_HW + WALL_T,       OCCUPIED)
        fill_rect(grid, prev_x, -CORR_HW - WALL_T, ELEV_DOOR_X, -CORR_HW,              OCCUPIED)

    # 5. Asansör arka ve yan duvarları
    fill_rect(grid,
              ELEV_X + ELEV_D/2, -ELEV_W/2 - WALL_T,
              ELEV_X + ELEV_D/2 + WALL_T, ELEV_W/2 + WALL_T,
              OCCUPIED)
    fill_rect(grid,
              ELEV_DOOR_X, -ELEV_W/2 - WALL_T,
              ELEV_X + ELEV_D/2, -ELEV_W/2,
              OCCUPIED)
    fill_rect(grid,
              ELEV_DOOR_X, ELEV_W/2,
              ELEV_X + ELEV_D/2, ELEV_W/2 + WALL_T,
              OCCUPIED)

    # 6. Asansör kapısı (başlangıçta kapalı — haritada engel)
    fill_rect(grid,
              ELEV_DOOR_X - WALL_T/2, -ELEV_W/2,
              ELEV_DOOR_X + WALL_T/2,  ELEV_W/2,
              OCCUPIED)

    return grid


def add_rect_obstacles(grid, obstacles):
    for cx, cy, sx, sy in obstacles:
        fill_rect(
            grid,
            cx - sx / 2, cy - sy / 2,
            cx + sx / 2, cy + sy / 2,
            OCCUPIED,
        )
    return grid


# ─── Kat 1 asimetri: Resepsiyon bankosu ──────────────────────────────────────
def add_floor1_asymmetry(grid):
    """
    AMCL Kat 1 tanıma öğeleri — statik haritaya işlenir.
    Resepsiyon bankosu: asansör çıkışı kuzeyinde L-şekilli blok.
    """
    # Ana gövde
    fill_rect(grid,
              RECEP_CX - RECEP_SX/2, RECEP_CY - RECEP_SY/2,
              RECEP_CX + RECEP_SX/2, RECEP_CY + RECEP_SY/2,
              OCCUPIED)
    # Köşe kanadı
    fill_rect(grid,
              RECEP_WING_OX - RECEP_WING_SX/2, RECEP_WING_OY - RECEP_WING_SY/2,
              RECEP_WING_OX + RECEP_WING_SX/2, RECEP_WING_OY + RECEP_WING_SY/2,
              OCCUPIED)
    return grid


# ─── Kat 2 asimetri: Kolonlar ─────────────────────────────────────────────────
def add_floor2_asymmetry(grid):
    """
    AMCL Kat 2 tanıma öğeleri — statik haritaya işlenir.
    İki beton kolon + saksı kombinasyonu.
    """
    # Sol kolon (silindir → haritada daire)
    fill_circle(grid, COL_X, COL_Y_L, COL_R + 0.05, OCCUPIED)
    # Sağ kolon
    fill_circle(grid, COL_X, COL_Y_R, COL_R + 0.05, OCCUPIED)
    # Sol saksı
    fill_circle(grid, PLANT_OX, COL_Y_L, PLANT_R + 0.05, OCCUPIED)
    # Sağ saksı
    fill_circle(grid, PLANT_OX, COL_Y_R, PLANT_R + 0.05, OCCUPIED)
    return grid


def generate_grid(floor_num: int):
    """
    Kat numarasına göre farklı PGM haritası üret.
    floor_num=1 → resepsiyon bankosu ekli
    floor_num=2 → beton kolonlar + saksılar ekli
    """
    grid = generate_base_grid()

    if floor_num == 1:
        add_floor1_asymmetry(grid)
        add_rect_obstacles(grid, FLOOR1_OBSTACLES)
    else:
        add_floor2_asymmetry(grid)
        add_rect_obstacles(grid, FLOOR2_OBSTACLES)

    return grid


def write_pgm(grid, path):
    h = len(grid); w = len(grid[0])
    with open(path, "wb") as f:
        f.write(f"P5\n# CREATOR: hotel_map_generator.py v2  {RESOLUTION:.3f}m/px\n"
                f"{w} {h}\n255\n".encode())
        for row in grid:
            f.write(bytes(row))


def write_yaml(path, pgm_name):
    with open(path, "w") as f:
        f.write(
            f"image: {pgm_name}\n"
            f"resolution: {RESOLUTION:.6f}\n"
            f"origin: [{MAP_MIN_X:.6f}, {MAP_MIN_Y:.6f}, 0.000000]\n"
            f"negate: 0\n"
            f"occupied_thresh: 0.65\n"
            f"free_thresh: 0.196\n"
        )


def main():
    maps_dir = os.path.join(os.path.dirname(__file__), "..", "maps")
    os.makedirs(maps_dir, exist_ok=True)

    print(f"hotel_map_generator v2")
    print(f"  Harita: {MAP_W}×{MAP_H} px  @ {RESOLUTION}m/px")
    print(f"  Kapsam: X[{MAP_MIN_X},{MAP_MAX_X}]  Y[{MAP_MIN_Y},{MAP_MAX_Y}]")
    print()

    for floor in [1, 2]:
        grid = generate_grid(floor)   # Her kat farklı ızgara
        pgm  = f"floor{floor}.pgm"
        pgm_path  = os.path.join(maps_dir, pgm)
        yaml_path = os.path.join(maps_dir, f"floor{floor}.yaml")
        alias_pgm_path = os.path.join(maps_dir, f"floor_{floor}.pgm")
        alias_yaml_path = os.path.join(maps_dir, f"floor_{floor}.yaml")

        write_pgm(grid, pgm_path)
        write_yaml(yaml_path, pgm)
        write_pgm(grid, alias_pgm_path)
        write_yaml(alias_yaml_path, f"floor_{floor}.pgm")

        free = sum(1 for row in grid for v in row if v == FREE)
        occ  = sum(1 for row in grid for v in row if v == OCCUPIED)
        asym = "Resepsiyon bankosu" if floor == 1 else "Kolonlar + saksılar"
        print(f"  floor{floor}.pgm  {os.path.getsize(pgm_path)//1024}KB"
              f"  free:{free} occ:{occ}")
        print(f"    Asimetri: {asym} ✓")

    print()
    print("  AMCL artık iki katı birbirinden ayırt edebilir:")
    print("    floor1 ≠ floor2  (farklı statik engel düzeni)")
    print("  floor1.yaml ve floor2.yaml güncellendi ✓")


if __name__ == "__main__":
    main()
