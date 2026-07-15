# RoboDK dot following

## Purpose

This RoboDK station represents the initial point-following proof of concept.

Robot 1 executes a predefined motion program and moves the reference object.
Robot 2 continuously follows the target `Touch_1`, whose absolute pose changes
with the motion of Robot 1.

## Included files

### Station

- `station/robodk_dot_following.rdk`

### Scripts

- `scripts/Dot_Following.py`
- `scripts/Dot_Following_Record.py`
- `scripts/Dot_Following_Replay.py`

## Required software

- RoboDK
- Python 3
- RoboDK Python API

## Required station objects

The scripts expect the station to contain:

- Robot 1
- Robot 2
- `Program_R1`
- `Touch_1`

The station object names must match the names used in the Python scripts.

## Point-following procedure

1. Open `station/robodk_dot_following.rdk`.
2. Verify that both robots are available.
3. Verify that `Program_R1` and `Touch_1` exist.
4. Run `scripts/Dot_Following.py`.
5. Robot 2 moves to the initial target pose.
6. Robot 1 starts its predefined motion.
7. Robot 2 repeatedly updates its target toward `Touch_1`.

## Recording procedure

1. Reset the station.
2. Run `scripts/Dot_Following_Record.py`.
3. Allow the complete motion to finish.
4. The script generates a timestamped CSV file containing both robot joint positions.

Generated CSV recordings are not stored in the repository.

## Replay procedure

1. Generate a recording with `Dot_Following_Record.py`.
2. Reset the station.
3. Run `scripts/Dot_Following_Replay.py`.
4. Both robots replay the recorded joint-position sequence.

## Expected behaviour

Robot 2 follows the moving point while Robot 1 executes its own motion.

## Detailed setup and portability notes

Install the RoboDK Python API in the interpreter used by RoboDK:

```bash
python3 -m pip install --user robodk
```

Before starting, confirm in the station tree that the names are exactly
`Robot1`, `Robot2`, `Program_R1` and `Touch_1`. `Touch_1` must be attached to
the reference moved by Robot 1, so its absolute pose changes while
`Program_R1` runs. Use simulation mode and stop any previous program.

The recording and replay scripts preserve their original development-machine
Windows paths. Before recording, set `CSV_FILE` in
`Dot_Following_Record.py` to an existing writable directory. Before replay,
set `pattern` in `Dot_Following_Replay.py` to the directory containing the
generated `R1R2_Record_*.csv` file. Generated recordings are intentionally
excluded from Git.

Stop `Program_R1` and both robot motions before resetting or reloading the
station. This station is the introductory proof of concept; continue with
[RoboDK path following and configuration analysis](../path_following_configuration_analysis/README.md).
