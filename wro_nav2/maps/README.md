# wro_nav2 maps

`wro_field.pgm` / `wro_field.yaml` is a placeholder: a 200x200 pixel image at
0.02 m/px (so 4x4 m) filled with the "free" value (0xFE = 254). Origin is
`(-2.0, -2.0, 0.0)` so the map is centered on `(0, 0)`.

## Regenerating a blank placeholder

If you ever need to recreate it (e.g. resize):

```bash
python3 - <<'PY'
w, h = 200, 200
with open('wro_field.pgm', 'wb') as f:
    f.write(f'P5\n{w} {h}\n255\n'.encode())
    f.write(bytes([254]) * (w * h))
PY
```

Then edit `wro_field.yaml` if you change resolution, size, or origin.

## Making a real map

Either:

1. Run the sim (`ros2 launch wro_sim sim_nav2.launch.py slam:=true`), drive
   the robot around, then save with:

   ```bash
   ros2 run nav2_map_server map_saver_cli -f wro_field
   ```

   and drop the resulting `wro_field.pgm` + `wro_field.yaml` here.

2. Or draw a map by hand from the WRO field spec, keeping the same
   resolution/origin scheme.
