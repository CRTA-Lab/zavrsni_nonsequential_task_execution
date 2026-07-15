# RoboDK path following and configuration analysis

## Purpose

This station represents the RoboDK line-following and robot-configuration
analysis phase.

Robot 1 moves the workpiece and executes a predefined motion. Robot 2 follows
an ordered sequence of Cartesian targets attached to the moving reference.

Several initial configurations of Robot 2 are tested. The generated joint data
are analysed to select the configuration with the most continuous motion.

## Included files

### Station

- `station/robodk_line_following_configuration_analysis.rdk`

### Scripts

- `scripts/Target_Generator.py`
- `scripts/Line_Following.py`
- `scripts/Line_Following_konfg.py`
- `scripts/Line_Following_Analiza.py`
- `scripts/Line_Following_ReplayAnalize.py`

## Required software

- RoboDK
- Python 3
- RoboDK Python API
- NumPy
- pandas

## Required station objects

The scripts expect:

- Robot 1
- Robot 2
- `Program_R1`
- `Start_Robot1`
- `Start_Robot2`
- targets `Line_001` to `Line_400`
- configuration targets such as `TMP_R1`, `TMP_R2` and `TMP_R3`

The exact names must match the names used in the Python scripts.

## Generate line targets

1. Open `station/robodk_line_following_configuration_analysis.rdk`.
2. Verify that the start and end targets expected by `Target_Generator.py` exist.
3. Run `scripts/Target_Generator.py`.
4. Verify that the required `Line_XXX` targets were created or updated.

## Execute basic line following

1. Reset the station.
2. Place both robots in the required starting configurations.
3. Run `scripts/Line_Following.py`.
4. Robot 1 starts `Program_R1`.
5. Robot 2 follows the moving ordered target sequence.

## Test initial configurations

1. Verify that all required `TMP_Ri` targets exist.
2. Run `scripts/Line_Following_konfg.py`.
3. The script repeats the task for every available initial configuration.
4. A detailed CSV recording is generated locally.

Generated CSV recordings are not stored in the repository.

## Analyse configurations

1. Run `scripts/Line_Following_Analiza.py`.
2. The script calculates joint jumps, reset events and other motion-quality values.
3. The script selects the most suitable tested configuration.
4. Generated analysis files remain local and are not committed.

## Replay the selected configuration

1. Generate the selected-configuration file with the analysis script.
2. Run `scripts/Line_Following_ReplayAnalize.py`.
3. Both simulated robots replay the selected motion.

## Expected behaviour

The procedure identifies a configuration with continuous joint motion and
without large configuration-reset events.

## Detailed setup, outputs and final selection

Install the Python dependencies in the interpreter used by RoboDK:

```bash
python3 -m pip install --user robodk numpy pandas
```

`Line_Following.py` and `Line_Following_konfg.py` require the full ordered
sequence `Line_001` through `Line_400`. The included `Target_Generator.py`
currently generates or updates the 100-point segment `Line_301` through
`Line_400` between `Kut_4` and `Pocetak`; the other segments must already be
present in the station. The generator preserves the orientation of `Kut_4`
and interpolates only the dominant translation axis.

Before testing, verify that `Robot1`, `Robot2`, `Program_R1`, `Start_Robot1`,
`Start_Robot2`, all `Line_XXX` targets and the sequential `TMP_R1`, `TMP_R2`,
... targets exist. The line targets must move with the Robot 1 reference.

`Line_Following_konfg.py` writes
`configs_line_following_detailed_notime.csv` next to the open station. It
records the configuration, sample, target, IK state and all twelve joint
values. Set `INPUT_CSV` in `Line_Following_Analiza.py` to that file. Analysis
creates `analysis_summary.csv` and `best_config_only.csv` and evaluates IK
failures, reset events and joint-jump statistics. The final project analysis
selected `TMP_R1` as the best configuration.

Set `INPUT_BEST_CSV` in `Line_Following_ReplayAnalize.py` to the generated
`best_config_only.csv` before replay. `INTERP_SUBSTEPS` controls inserted
interpolation steps and `PLAYBACK_SPEED` changes replay speed.

The complete order is: verify/generate targets, run basic line following,
record every `TMP_Ri`, analyse the detailed CSV, inspect the summary, replay
`best_config_only.csv`, then stop both robots and `Program_R1` before reset.
