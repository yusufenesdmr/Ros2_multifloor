# Hotel Floor Maps

Place SDF2MAP-generated maps here, one per hotel floor.

## Expected files

```
maps/
├── floor_1.pgm
├── floor_1.yaml
├── floor_2.pgm
├── floor_2.yaml
├── floor_3.pgm
├── floor_3.yaml
├── floor_4.pgm
├── floor_4.yaml
└── floor_5.pgm
└── floor_5.yaml
```

## SDF2MAP generation settings

Use these settings in SDF2MAP (https://github.com/sherif1152/SDF2MAP)
to ensure compatibility with the Nav2 config in `config/hotel_nav2.yaml`:

| Parameter         | Value  | Reason                                    |
|-------------------|--------|-------------------------------------------|
| Resolution        | 0.05   | Matches Nav2 costmap resolution           |
| LIDAR Height      | 0.20 m | Typical sensor mounting height            |
| occupied_thresh   | 0.65   | ROS standard                              |
| free_thresh       | 0.25   | ROS standard                              |
| World Margin      | 1.0 m  | Border clearance                          |
| Super Sampling    | 4x     | Anti-aliasing for cleaner walls           |

## Map YAML format (output of SDF2MAP)

```yaml
image: floor_1.pgm
mode: trinary
resolution: 0.05
origin: [-20.0, -20.0, 0.0]   # bottom-left in world coords – adjust per floor
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

## Important: elevator exit coordinates

The wormhole database stores the elevator approach/exit coordinates
**in the coordinate frame of each floor's map**.

If different floors have different map origins (which is typical when
each floor's SDF/World file has different dimensions), you must
update the wormhole database with the correct coordinates for each floor.

Run the database creation script and edit the coordinates:

```bash
cd database/
python3 create_demo_db.py --maps-dir /path/to/maps/ --out hotel_wormholes.db
```

Then verify the database:
```bash
sqlite3 hotel_wormholes.db "SELECT from_map, to_map, from_x, from_y FROM wormholes;"
```
