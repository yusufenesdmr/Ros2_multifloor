#!/usr/bin/env python3
"""
Create a two-floor wormhole database for the hotel simulation.

Schema matches `multi_floor_navigator.cpp`.
"""

import argparse
import os
import sqlite3


SCHEMA = """
CREATE TABLE IF NOT EXISTS wormholes (
    id            INTEGER PRIMARY KEY AUTOINCREMENT,
    from_floor    TEXT NOT NULL,
    to_floor      TEXT NOT NULL,
    approach_x    REAL NOT NULL,
    approach_y    REAL NOT NULL,
    approach_yaw  REAL NOT NULL DEFAULT 0.0,
    elevator_id   TEXT NOT NULL,
    map_yaml_path TEXT NOT NULL
);
"""

INSERT = """
INSERT INTO wormholes
  (from_floor, to_floor, approach_x, approach_y, approach_yaw, elevator_id, map_yaml_path)
VALUES (?, ?, ?, ?, ?, ?, ?)
"""


def build_entries(maps_dir: str):
    maps_dir = maps_dir.rstrip("/")
    # Synced with hotel_world_generator / hotel_map_generator.
    approach_x = 5.75
    approach_y = 0.0
    approach_yaw = 0.0
    return [
        (
            "floor_1",
            "floor_2",
            approach_x,
            approach_y,
            approach_yaw,
            "rotav_elevator",
            os.path.join(maps_dir, "floor_2.yaml"),
        ),
        (
            "floor_2",
            "floor_1",
            approach_x,
            approach_y,
            approach_yaw,
            "rotav_elevator",
            os.path.join(maps_dir, "floor_1.yaml"),
        ),
    ]


def create_db(db_path: str, maps_dir: str):
    if os.path.exists(db_path):
        os.remove(db_path)

    conn = sqlite3.connect(db_path)
    conn.execute(SCHEMA)
    conn.executemany(INSERT, build_entries(maps_dir))
    conn.commit()
    conn.close()

    print(f"Created {db_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--maps-dir", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    create_db(args.out, args.maps_dir)


if __name__ == "__main__":
    main()
