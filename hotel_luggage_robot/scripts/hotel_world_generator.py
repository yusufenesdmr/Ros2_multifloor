#!/usr/bin/env python3
"""
hotel_world_generator.py  –  ROTA-V Otel Dünyası (v5)
=======================================================
Yenilikler (v5 vs v4):
  • AMCL asimetrisi: Kat 1 asansör çıkışında Resepsiyon Bankosu
                     Kat 2 asansör çıkışında Kolonlar + Saksılar
  • SDF2MAP uyumlu: hareketli aktörler (insanlar) AYRI dosyaya yazılır
  • Çöp kovası ve bavul arabaları koridorda farklı konumda her katta
  • Oda duvarları: sadece kapı çerçevesi (pervaz), tavan YOK
  • Asansör kapısı: kinematik SDF modeli (f1_elev_door / f2_elev_door)
  • Kat 1 odalar: 101-108  |  Kat 2 odalar: 201-208

Çıktı dosyaları:
  worlds/hotel_two_floor.world   — bina geometrisi (static)
  worlds/hotel_actors.world      — yürüyen insanlar (ayrı include için)
"""

import os

DIGIT_SEGMENTS = {
    "0": "abcfed",
    "1": "bc",
    "2": "abged",
    "3": "abgcd",
    "4": "fgbc",
    "5": "afgcd",
    "6": "afgcde",
    "7": "abc",
    "8": "abcdefg",
    "9": "abcfgd",
}

# ─── KRİTİK KOORDİNATLAR — floor_switcher / elevator_sequencer ile SENKRON ──
ELEV_X = 8.0
ELEV_Y = 0.0
ELEV_W = 2.5
ELEV_D = 2.5
ELEV_DOOR_X = ELEV_X - ELEV_D / 2   # = 6.75

FLOOR1_Z = 0.0
FLOOR2_Z = 3.5
WALL_H   = 2.8

CORR_W       = 10.0
CORR_HW      = CORR_W / 2   # 2.5
CORR_START_X = -12.0

WALL_T = 0.2
DOOR_POSITIONS_X = [-9.5, -6.5, -3.5, -0.5]
DOOR_W  = 1.5
POST_T  = 0.15

# Asansör bekleme noktası (robot buraya navigate eder, sonra içeri girer)
ELEV_WAIT_X = ELEV_DOOR_X - 1.0   # = 5.75
ELEV_WAIT_Y = ELEV_Y               # = 0.0


# ─── Materyaller ──────────────────────────────────────────────────────────────
def mat(r, g, b, s=0.1):
    return (f"<ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient>"
            f"<diffuse>{r:.2f} {g:.2f} {b:.2f} 1</diffuse>"
            f"<specular>{s:.2f} {s:.2f} {s:.2f} 1</specular>")

MAT_WALL   = mat(0.93, 0.90, 0.82)
MAT_FLOOR  = mat(0.84, 0.80, 0.70)
MAT_ELEV   = mat(0.65, 0.70, 0.78, 0.6)
MAT_DOOR   = mat(0.55, 0.38, 0.22, 0.2)
MAT_SIGN_S = mat(0.90, 0.36, 0.10)    # turuncu güney
MAT_SIGN_N = mat(0.11, 0.43, 0.82)    # mavi kuzey
MAT_OBS    = mat(0.28, 0.28, 0.28)
MAT_RECEP  = mat(0.60, 0.45, 0.25, 0.3)   # ahşap resepsiyon
MAT_PILLAR = mat(0.80, 0.80, 0.82, 0.5)   # beton kolon
MAT_PLANT  = mat(0.15, 0.55, 0.18)        # saksı yeşili
MAT_RUNNER_1 = mat(0.73, 0.16, 0.16, 0.2)
MAT_RUNNER_2 = mat(0.06, 0.44, 0.53, 0.2)
MAT_ROOM_N = mat(0.80, 0.90, 0.98, 0.1)
MAT_ROOM_S = mat(0.99, 0.89, 0.78, 0.1)
MAT_GOLD = mat(0.94, 0.78, 0.20, 0.4)
MAT_LABEL = mat(0.98, 0.98, 0.98, 0.2)
MAT_BIN = mat(0.12, 0.22, 0.28, 0.3)
MAT_BIN_LID = mat(0.85, 0.20, 0.10, 0.3)


def box(name, px, py, pz, sx, sy, sz, material=MAT_WALL, col=True):
    col_xml = (f"<collision name='col'>"
               f"<geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>"
               f"</collision>" if col else "")
    return (f"\n    <model name='{name}'><static>true</static>"
            f"<pose>{px:.4f} {py:.4f} {pz:.4f} 0 0 0</pose>"
            f"<link name='link'>{col_xml}"
            f"<visual name='vis'><geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size>"
            f"</box></geometry><material>{material}</material></visual></link></model>")


def cylinder(name, px, py, pz, radius, length, material=MAT_PILLAR, col=True):
    col_xml = (f"<collision name='col'>"
               f"<geometry><cylinder><radius>{radius}</radius>"
               f"<length>{length}</length></cylinder></geometry>"
               f"</collision>" if col else "")
    return (f"\n    <model name='{name}'><static>true</static>"
            f"<pose>{px:.4f} {py:.4f} {pz:.4f} 0 0 0</pose>"
            f"<link name='link'>{col_xml}"
            f"<visual name='vis'><geometry><cylinder>"
            f"<radius>{radius}</radius><length>{length}</length>"
            f"</cylinder></geometry><material>{material}</material>"
            f"</visual></link></model>")


def trash_bin(name, px, py, z):
    body_h = 0.72
    lid_h = 0.06
    return "".join([
        cylinder(f"{name}_body", px, py, z + body_h / 2, 0.20, body_h, MAT_BIN, col=False),
        cylinder(f"{name}_lid", px, py, z + body_h + lid_h / 2, 0.22, lid_h, MAT_BIN_LID, col=False),
    ])


def room_number_label(name, room_no, px, py, pz, side, material=MAT_LABEL):
    parts = []
    digit_w = 0.26
    digit_h = 0.42
    thickness = 0.05
    depth = 0.02
    spacing = 0.08
    y = py + (-0.10 if side == "s" else 0.10)
    centers = [
        px - (digit_w + spacing),
        px,
        px + (digit_w + spacing),
    ]
    seg_map = {
        "a": (0.0,  digit_h / 2, digit_w, thickness),
        "b": (digit_w / 2,  digit_h / 4, thickness, digit_h / 2),
        "c": (digit_w / 2, -digit_h / 4, thickness, digit_h / 2),
        "d": (0.0, -digit_h / 2, digit_w, thickness),
        "e": (-digit_w / 2, -digit_h / 4, thickness, digit_h / 2),
        "f": (-digit_w / 2,  digit_h / 4, thickness, digit_h / 2),
        "g": (0.0, 0.0, digit_w, thickness),
    }
    for idx, digit in enumerate(str(room_no)):
        for seg in DIGIT_SEGMENTS[digit]:
            ox, oz, sx, sz = seg_map[seg]
            parts.append(box(
                f"{name}_{idx}_{seg}",
                centers[idx] + ox,
                y,
                pz + oz,
                sx,
                depth,
                sz,
                material,
                col=False,
            ))
    return "".join(parts)


def door_model(name, px, py, pz, sx, sy, sz, material=MAT_DOOR):
    """Kinematik asansör kapısı (SetEntityState ile açılır/kapanır)."""
    return (
        f"\n    <model name='{name}'><static>false</static>"
        f"<pose>{px:.4f} {py:.4f} {pz:.4f} 0 0 0</pose>"
        f"<link name='door'>"
        f"<gravity>false</gravity>"
        f"<kinematic>true</kinematic>"
        f"<inertial><mass>50.0</mass>"
        f"<inertia><ixx>10</ixx><ixy>0</ixy><ixz>0</ixz>"
        f"<iyy>10</iyy><iyz>0</iyz><izz>10</izz></inertia>"
        f"</inertial>"
        f"<collision name='col'>"
        f"<geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>"
        f"</collision>"
        f"<visual name='vis'>"
        f"<geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>"
        f"<material>{material}</material></visual></link></model>"
    )


# ─── Kat 1 asimetri: Resepsiyon Bankosu ──────────────────────────────────────
def floor1_asymmetry(z: float) -> str:
    """
    AMCL asimetrisi için Kat 1'e özgü öğeler.
    Asansör çıkışı (ELEV_DOOR_X) yanında L-şekilli resepsiyon bankosu.
    Bu sayede Kat 1 ve Kat 2 haritaları birbirinden ayırt edilebilir.
    """
    parts = []
    rx = ELEV_DOOR_X - 1.8   # banka X merkezi
    ry = 4.4                  # kuzey duvara yakın

    # Ana banka gövdesi
    parts.append(box("f1_recep_main",
        rx, ry, z + 0.55,
        2.2, 0.5, 1.1, MAT_RECEP))
    # Banka üst tezgah
    parts.append(box("f1_recep_top",
        rx, ry, z + 1.15,
        2.4, 0.65, 0.08, MAT_RECEP))
    # Köşe kısmı
    parts.append(box("f1_recep_wing",
        rx + 0.85, ry - 0.45, z + 0.55,
        0.5, 0.7, 1.1, MAT_RECEP))

    return "".join(parts)


# ─── Kat 2 asimetri: Kolonlar + Saksılar ─────────────────────────────────────
def floor2_asymmetry(z: float) -> str:
    """
    AMCL asimetrisi için Kat 2'ye özgü öğeler.
    Asansör çıkışı yanında iki kolon ve saksılar.
    """
    parts = []
    col_x = ELEV_DOOR_X - 1.2

    # Sol kolon
    parts.append(cylinder("f2_col_L",
        col_x, -4.4, z + WALL_H / 2,
        0.22, WALL_H, MAT_PILLAR))
    # Sağ kolon
    parts.append(cylinder("f2_col_R",
        col_x,  4.4, z + WALL_H / 2,
        0.22, WALL_H, MAT_PILLAR))
    # Saksı sol
    parts.append(cylinder("f2_plant_L",
        col_x - 0.5, -4.4, z + 0.35,
        0.25, 0.70, MAT_PLANT))
    # Saksı sağ
    parts.append(cylinder("f2_plant_R",
        col_x - 0.5,  4.4, z + 0.35,
        0.25, 0.70, MAT_PLANT))

    return "".join(parts)


# ─── Bir katın tüm öğeleri ────────────────────────────────────────────────────
def floor_elements(label: str, z: float, floor_num: int) -> str:
    parts = []
    h2 = WALL_H / 2

    # ── Zemin ────────────────────────────────────────────────────────────────
    total_x = ELEV_X + ELEV_D / 2 + WALL_T - CORR_START_X
    cx_bldg = (CORR_START_X + ELEV_X + ELEV_D / 2 + WALL_T) / 2
    parts.append(box(f"{label}_floor",
        cx_bldg, ELEV_Y, z - 0.05,
        total_x, CORR_W + 12.0, 0.1, MAT_FLOOR))
    parts.append(box(
        f"{label}_runner",
        (CORR_START_X + ELEV_DOOR_X) / 2, 0.0, z - 0.01,
        ELEV_DOOR_X - CORR_START_X, 1.8, 0.02,
        MAT_RUNNER_1 if floor_num == 1 else MAT_RUNNER_2,
        col=False,
    ))

    # ── Batı duvar ───────────────────────────────────────────────────────────
    parts.append(box(f"{label}_wall_west",
        CORR_START_X - WALL_T / 2, ELEV_Y, z + h2,
        WALL_T, CORR_W, WALL_H))

    # ── Koridor duvar segmentleri (kapı açıklıklı) ────────────────────────────
    prev_x = CORR_START_X
    segs = []
    for dx in DOOR_POSITIONS_X:
        dl = dx - DOOR_W / 2 - POST_T
        dr = dx + DOOR_W / 2 + POST_T
        if dl > prev_x + 0.01:
            segs.append((prev_x, dl))
        prev_x = dr
    if ELEV_DOOR_X > prev_x + 0.01:
        segs.append((prev_x, ELEV_DOOR_X))

    for i, (x0, x1) in enumerate(segs):
        cx = (x0 + x1) / 2; w = x1 - x0
        for side, yoff in [("n", CORR_HW + WALL_T / 2), ("s", -CORR_HW - WALL_T / 2)]:
            parts.append(box(f"{label}_seg_{side}_{i}",
                cx, yoff, z + h2, w, WALL_T, WALL_H))

    # ── Kapı çerçeveleri + plaklar (tavan/lento yok) ─────────────────────────
    for i, dx in enumerate(DOOR_POSITIONS_X):
        for side, yf, sm, rno in [
            ("s", -CORR_HW, f"<material>{MAT_SIGN_S}</material>", floor_num*100+i+1),
            ("n",  CORR_HW, f"<material>{MAT_SIGN_N}</material>", floor_num*100+i+5),
        ]:
            sd = -1 if side == "s" else 1
            parts.append(box(f"{label}_dp_{side}{i}L",
                dx - DOOR_W/2 - POST_T/2, yf, z + WALL_H/2,
                POST_T, WALL_T, WALL_H, MAT_DOOR))
            parts.append(box(f"{label}_dp_{side}{i}R",
                dx + DOOR_W/2 + POST_T/2, yf, z + WALL_H/2,
                POST_T, WALL_T, WALL_H, MAT_DOOR))
            parts.append(box(f"{label}_sign_{side}{i}",
                dx + DOOR_W/2 + POST_T + 0.25, yf + sd*0.025, z + 1.5,
                0.38, 0.05, 0.18, sm))
            parts.append(box(f"{label}_lbl_{side}{i}",
                dx + DOOR_W/2 + POST_T + 0.25, yf + sd*0.055, z + 1.5,
                0.28, 0.01, 0.12,
                f"<material>{mat(0.97,0.97,0.97)}</material>", col=False))

    # ── Asansör nişi ──────────────────────────────────────────────────────────
    ex, ey = ELEV_X, ELEV_Y
    parts.append(box(f"{label}_elev_back",
        ex + ELEV_D/2, ey, z + h2,
        WALL_T, ELEV_W + 2*WALL_T, WALL_H, MAT_ELEV))
    parts.append(box(f"{label}_elev_L",
        ex, ey - ELEV_W/2, z + h2,
        ELEV_D, WALL_T, WALL_H, MAT_ELEV))
    parts.append(box(f"{label}_elev_R",
        ex, ey + ELEV_W/2, z + h2,
        ELEV_D, WALL_T, WALL_H, MAT_ELEV))
    parts.append(box(f"{label}_elev_floor",
        ex, ey, z - 0.02, ELEV_D, ELEV_W, 0.04,
        mat(0.20,0.50,0.90,0.4)))
    parts.append(box(f"{label}_kat_sign",
        ex + ELEV_D/2 + 0.4, ey, z + 2.0, 0.08, 0.9, 0.45,
        mat(1.0,0.80,0.0) if floor_num==1 else mat(0.18,0.78,0.28)))

    # ── Asansör kapısı (kinematik) ────────────────────────────────────────────
    door_z = z + WALL_H / 2
    parts.append(door_model(
        f"{label}_elev_door",
        ELEV_DOOR_X, 0.0, door_z,
        WALL_T, ELEV_W, WALL_H,
        mat(0.40,0.55,0.75,0.8)
    ))

    # ── Asimetri öğeleri (AMCL kat ayrımı için) ───────────────────────────────
    if floor_num == 1:
        parts.append(floor1_asymmetry(z))
    else:
        parts.append(floor2_asymmetry(z))

    # ── Kat'a özgü engeller ───────────────────────────────────────────────────
    if floor_num == 1:
        obst = []
    else:
        obst = []

    for nm, ox, oy, sx, sy, sh in obst:
        parts.append(box(f"{label}_obs_{nm}",
            ox, oy, z + sh/2, sx, sy, sh,
            MAT_OBS))

    for i, dx in enumerate(DOOR_POSITIONS_X):
        north_room = floor_num * 100 + i + 5
        south_room = floor_num * 100 + i + 1
        parts.append(box(
            f"{label}_pad_n_{i}",
            dx, CORR_HW + 0.8, z - 0.01,
            1.8, 1.4, 0.02, MAT_ROOM_N, col=False
        ))
        parts.append(box(
            f"{label}_pad_s_{i}",
            dx, -CORR_HW - 0.8, z - 0.01,
            1.8, 1.4, 0.02, MAT_ROOM_S, col=False
        ))
        label_x = dx + DOOR_W / 2 + POST_T + 0.30
        parts.append(room_number_label(
            f"{label}_room_n_{north_room}",
            north_room, label_x, CORR_HW, z + 1.65, "n", MAT_LABEL
        ))
        parts.append(room_number_label(
            f"{label}_room_s_{south_room}",
            south_room, label_x, -CORR_HW, z + 1.65, "s", MAT_LABEL
        ))

    # ── Dekoratif çöp kutuları (yalnızca görsel, navigasyonu bozmaz) ────────
    bin_xs = [-10.8, -4.8, 4.8]
    north_y = CORR_HW - 0.45
    south_y = -CORR_HW + 0.45
    for i, bx in enumerate(bin_xs):
        by = north_y if i % 2 == 0 else south_y
        parts.append(trash_bin(f"{label}_trash_{i}", bx, by, z))

    return "".join(parts)


# ─── Hareketli aktörler (SDF2MAP için ayrı dosya) ────────────────────────────
def actors_sdf() -> str:
    """
    Koridorda yürüyen 3 karakter.
    SDF2MAP ile harita çıkarırken INCLUDE ETME — insanlar duvara dönmez.
    Kullanım: hotel_two_floor.world içine
      <include><uri>model://hotel_actors</uri></include>
    veya ayrı bir launch'ta include et.
    """
    def actor(name, x, y, dx, dy, speed=0.8):
        return f"""
    <actor name="{name}">
      <pose>{x:.1f} {y:.1f} 1.0 0 0 0</pose>
      <skin>
        <filename>walk.dae</filename>
        <scale>1.0</scale>
      </skin>
      <animation name="walking">
        <filename>walk.dae</filename>
        <scale>1.0</scale>
        <interpolate_x>true</interpolate_x>
      </animation>
      <script>
        <loop>true</loop>
        <delay_start>0.0</delay_start>
        <auto_start>true</auto_start>
        <trajectory id="0" type="walking">
          <waypoint><time>0</time>
            <pose>{x:.1f} {y:.1f} 0 0 0 0</pose></waypoint>
          <waypoint><time>8</time>
            <pose>{x+dx*speed*8:.1f} {y+dy*speed*8:.1f} 0 0 0 0</pose></waypoint>
          <waypoint><time>16</time>
            <pose>{x:.1f} {y:.1f} 0 0 0 0</pose></waypoint>
        </trajectory>
      </script>
    </actor>"""

    return f"""\
<?xml version="1.0"?>
<!--
  hotel_actors.world  —  Hareketli karakterler (AYRI dosya)
  SDF2MAP ile harita çıkarırken BU DOSYAYI INCLUDE ETME.
  Sadece simülasyon çalışırken include edilmeli.

  Kullanım: gzserver komut satırına ek olarak include et:
    ros2 launch ... extra_gz_args:="--verbose"
  veya launch dosyasına ayrı ExecuteProcess ekle.
-->
<sdf version="1.6">
  <world name="hotel_actors">

    <!-- Koridor boyunca ileri geri yürüyen 3 karakter -->
    {actor("walker_1",  -8.0,  0.3,  1, 0, 0.7)}
    {actor("walker_2",  -5.0, -0.4, -1, 0, 0.6)}
    {actor("walker_3",  -2.0,  0.5,  1, 0, 0.8)}

  </world>
</sdf>
"""


def actor_block(name, x, y, z, yaw, dx, dy, speed=0.5, delay=0.0):
    end_x = x + dx
    end_y = y + dy
    duration = max(3.0, (abs(dx) + abs(dy)) / max(speed, 0.1))
    return f"""
    <actor name="{name}">
      <pose>{x:.2f} {y:.2f} {z:.2f} 0 0 {yaw:.4f}</pose>
      <skin>
        <filename>walk.dae</filename>
        <scale>1.0</scale>
      </skin>
      <animation name="walking">
        <filename>walk.dae</filename>
        <scale>1.0</scale>
        <interpolate_x>true</interpolate_x>
      </animation>
      <script>
        <loop>true</loop>
        <delay_start>{delay:.1f}</delay_start>
        <auto_start>true</auto_start>
        <trajectory id="0" type="walking">
          <waypoint><time>0.0</time><pose>{x:.2f} {y:.2f} {z:.2f} 0 0 {yaw:.4f}</pose></waypoint>
          <waypoint><time>{duration:.1f}</time><pose>{end_x:.2f} {end_y:.2f} {z:.2f} 0 0 {yaw:.4f}</pose></waypoint>
          <waypoint><time>{2 * duration:.1f}</time><pose>{x:.2f} {y:.2f} {z:.2f} 0 0 {yaw:.4f}</pose></waypoint>
        </trajectory>
      </script>
    </actor>"""


# ─── World XML üretimi ────────────────────────────────────────────────────────
def generate() -> str:
    return f"""\
<?xml version="1.0"?>
<!--
  hotel_two_floor.world  —  ROTA-V v5
  ─────────────────────────────────────────────────────────────────────
  Kat-1: Z={FLOOR1_Z}  (101-108)  |  Kat-2: Z={FLOOR2_Z}  (201-208)
  Asansör: ({ELEV_X}, {ELEV_Y})  —  her katta aynı
  Bekleme noktası: ({ELEV_WAIT_X}, {ELEV_WAIT_Y})

  AMCL asimetrisi:
    Kat 1 → Resepsiyon bankosu (asansör çıkışı kuzey)
    Kat 2 → Betonkolon çifti + saksılar (asansör çıkışı)

  Tavan YOK → üstten görünür
  Asansör kapısı kinematik → floor_switcher.py ile kontrol

  Aktörler (yürüyen insanlar): hotel_actors.world'de (SDF2MAP'te INCLUDE ETME)
-->
<sdf version="1.6">
  <world name="hotel_two_floor">

    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>

    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>0.9 0.9 0.85 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="col">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="vis">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.13 0.16 0.20 1</ambient>
                    <diffuse>0.13 0.16 0.20 1</diffuse></material>
        </visual>
      </link>
    </model>

    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/</namespace></ros>
      <update_rate>10</update_rate>
    </plugin>

    <!-- ════ KAT 1  (Z={FLOOR1_Z}, odalar 101-108) ════ -->
    {floor_elements("f1", FLOOR1_Z, 1)}

    <!-- ════ KAT 2  (Z={FLOOR2_Z}, odalar 201-208, üst üste) ════ -->
    {floor_elements("f2", FLOOR2_Z, 2)}

    <!-- ════ Hareketli insanlar (haritaya işlenmez, yalnızca simülasyon) ════ -->
    {actor_block("guest_f1_a", -8.8, 1.6, FLOOR1_Z + 0.02, 0.0, 2.6, 0.0, speed=0.35, delay=1.0)}
    {actor_block("guest_f1_b", -2.8, -1.7, FLOOR1_Z + 0.02, 3.1415926535, -2.4, 0.0, speed=0.30, delay=3.0)}
    {actor_block("guest_f1_c", -6.0, 0.8, FLOOR1_Z + 0.02, 0.0, 1.8, 0.0, speed=0.26, delay=5.0)}
    {actor_block("guest_f2_a", -7.2, 1.5, FLOOR2_Z + 0.02, 0.0, 2.2, 0.0, speed=0.32, delay=2.0)}
    {actor_block("guest_f2_b", -1.8, -1.6, FLOOR2_Z + 0.02, 3.1415926535, -2.0, 0.0, speed=0.28, delay=4.0)}
    {actor_block("guest_f2_c", -4.5, 0.9, FLOOR2_Z + 0.02, 0.0, 1.6, 0.0, speed=0.24, delay=6.0)}

  </world>
</sdf>
"""


def main():
    base = os.path.join(os.path.dirname(__file__), "..", "worlds")
    os.makedirs(base, exist_ok=True)

    # Ana bina world
    world_path = os.path.join(base, "hotel_two_floor.world")
    with open(world_path, "w") as f:
        f.write(generate())
    print(f"✓ {world_path}  ({os.path.getsize(world_path)/1024:.1f} KB)")

    # Aktörler (ayrı dosya — SDF2MAP'e dahil edilmeyecek)
    actors_path = os.path.join(base, "hotel_actors.world")
    with open(actors_path, "w") as f:
        f.write(actors_sdf())
    print(f"✓ {actors_path}")

    print()
    print(f"  AMCL asimetrisi:")
    print(f"    Kat 1 → Resepsiyon bankosu  (x≈{ELEV_DOOR_X-1.8:.1f}, y≈4.4)")
    print(f"    Kat 2 → Kolonlar + saksılar (x≈{ELEV_DOOR_X-1.2:.1f}, y=±4.4)")
    print(f"  Asansör bekleme: ({ELEV_WAIT_X}, {ELEV_WAIT_Y})")
    print(f"  Oda numaraları: Kat1=101-108, Kat2=201-208")
    print(f"  Aktörler: hotel_actors.world (SDF2MAP'te INCLUDE ETME)")


if __name__ == "__main__":
    main()
