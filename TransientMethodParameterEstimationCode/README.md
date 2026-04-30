# `oneDkhEstimator.py`

Modernized 1-D transient convection parameter estimator for rod cooling/heating data.

This script is a cleaned-up and parallelized version of the original `oneDkhEstimator.py` workflow. It preserves the same three-stage estimation idea:

1. fit the boundary heat-flux/temperature-gradient surrogate from the first three thermocouples,
2. fit a steady-state fin model to obtain an initial estimate of `k`,
3. fit the transient Fourier-series model to estimate
   - convection coefficient `h`,
   - thermal conductivity `k`,
   - steady input power `Pss`.

The script writes one PDF plot per data file plus a results table in CSV format and, when `openpyxl` is installed, XLSX format.

---

## 1. Directory layout

The easiest layout is:

```text
project_directory/
├── oneDkhEstimator.py
├── data/
│   ├── 50al1.csv
│   ├── 75al2.csv
│   ├── 100al3.csv
│   ├── 50cu1.csv
│   └── ...
└── FIGS/                 # created automatically
```

By default, the script looks for a `data/` directory next to the script. If no `data/` directory exists, it uses the script directory itself as the data directory.

You can override the data and output locations:

```bash
python oneDkhEstimator.py \
  --data-dir /path/to/data \
  --output-dir /path/to/results
```

---

## 2. Required Python packages

Required:

```bash
pip install numpy scipy matplotlib
```

Optional, for XLSX output:

```bash
pip install openpyxl
```

If `openpyxl` is not installed, the script still writes the CSV results file.

---

## 3. Expected CSV data format

The estimator currently expects at least **eight numeric columns** in each data row:

```text
time, TC1, TC2, TC3, TC4, TC5, TC6, duty_cycle
```

Extra trailing columns are accepted and ignored. For example, future acquisition files with up to 21 columns are fine:

```text
time, TC1, TC2, TC3, TC4, TC5, TC6, duty_cycle, extra_1, extra_2, ..., extra_13
```

The script records the maximum numeric column count detected in the output column:

```text
n_columns_detected
```

It also records duty-cycle summary values:

```text
duty_cycle_initial
duty_cycle_final
duty_cycle_mean
```

At present, the duty-cycle column is not used in the thermal model fit. It is parsed and recorded so the acquisition metadata is not lost.

---

## 4. Header and metadata preamble support

Future files may include acquisition metadata before the numeric data, such as:

```text
Program: Transient Rod Acquisition
Version: 1.4.2
Date: 2026-04-28
Operator: DS
Columns: time, TC1, TC2, TC3, TC4, TC5, TC6, duty_cycle
0.000, 0.01, 0.03, 0.02, 0.01, 0.00, -0.01, 512
0.100, 0.05, 0.04, 0.03, 0.02, 0.01, 0.00, 512
...
```

The parser skips all rows until it finds rows whose first eight columns are numeric. The number of skipped rows is recorded in:

```text
skipped_preamble_rows
```

---

## 5. Default file selection behavior

Version 2.3.0 uses **data-directory discovery by default**.

That means the script scans the data directory for `.csv` files, infers the material from the filename, processes the files it finds, and reports only the number selected.

Example:

```bash
python oneDkhEstimator.py --material all --test temp
```

Typical startup output:

```text
Selected files:   9 CSV file(s) (data-directory discovery)
Worker processes: 9
```

The script no longer prints a long list of missing historical filenames by default.

### Material inference from filenames

The discovery mode recognizes filenames containing:

| Filename contains | Material used |
|---|---|
| `al` | aluminum |
| `cu` | copper |
| `ss` | stainless steel |

Examples:

```text
50al1.csv   -> aluminum
75cu1.csv   -> copper
100ss1.csv  -> stainless steel
```

CSV files whose material cannot be inferred are skipped silently in discovery mode. To force one of those files to run, specify it explicitly with `--files` and choose the material with `--material`.

Example:

```bash
python oneDkhEstimator.py \
  --material al \
  --files my_special_aluminum_test.csv
```

---

## 6. Running the script

### Run all discovered aluminum files

```bash
python oneDkhEstimator.py
```

This is equivalent to:

```bash
python oneDkhEstimator.py --material al --test temp
```

### Run all discovered aluminum, copper, and stainless files

```bash
python oneDkhEstimator.py --material all --test temp
```

### Run selected files only

```bash
python oneDkhEstimator.py \
  --material al \
  --files 50al1 75al2 100al3
```

The names can be supplied with or without `.csv`.

### Run the historical built-in file list

The original code had hard-coded expected file lists such as:

```text
50al1, 50al2, 50al3, 75al1, ..., 100al3
```

To use those expected lists instead of directory discovery:

```bash
python oneDkhEstimator.py \
  --material al \
  --test temp \
  --expected-list
```

By default, missing files in the expected list are skipped quietly. To print the missing expected-list paths:

```bash
python oneDkhEstimator.py \
  --material al \
  --test temp \
  --expected-list \
  --report-missing
```

To make missing expected files an error:

```bash
python oneDkhEstimator.py \
  --material al \
  --test temp \
  --expected-list \
  --strict
```

---

## 7. Parallel processing

The data files are independent, so the script parallelizes at the file level. One worker process handles one data file at a time.

Automatic mode is the default:

```bash
python oneDkhEstimator.py --jobs 0
```

`--jobs 0` means:

```text
use up to all available CPU cores, but never more workers than selected files
```

So if the machine has 16 cores but only 9 files are selected, the script uses at most 9 worker processes.

Manual examples:

```bash
python oneDkhEstimator.py --jobs 1   # serial
python oneDkhEstimator.py --jobs 4   # four worker processes
python oneDkhEstimator.py --jobs 8   # eight worker processes
```

### Practical speed recommendation

For 9 files, 16 workers cannot help because there are only 9 independent jobs. In practice, useful settings are usually:

```bash
--jobs 1
--jobs 3
--jobs 6
--jobs 9
```

The best value depends on:

- number of files,
- number of Fourier terms `N`,
- data length,
- optimizer settings,
- whether NumPy/SciPy are already using threaded BLAS internally,
- memory bandwidth.

For short datasets or small `N`, multiprocessing overhead can dominate. For larger `N`, longer datasets, or many repeated tests, parallel processing should help more.

---

## 8. Controlling the Fourier-series truncation

The original script used:

```text
N = 100
```

The modernized script keeps that default:

```bash
python oneDkhEstimator.py --N 100
```

For quick smoke testing:

```bash
python oneDkhEstimator.py --N 10 --max-nfev 20
```

For production runs, use the default or a convergence-tested larger value.

---

## 9. Output files

Default outputs are written to the output directory:

```text
oneDkh_results.csv
oneDkh_results.xlsx     # if openpyxl is installed
FIGS/<filename>_fit.pdf
```

Change the base results filename:

```bash
python oneDkhEstimator.py --results-name April_2026_rod_tests
```

Change the plot directory:

```bash
python oneDkhEstimator.py --plots-dir /path/to/plot_directory
```

Add fitted values to the plot title:

```bash
python oneDkhEstimator.py --title
```

Show plots interactively:

```bash
python oneDkhEstimator.py --show
```

When `--show` is used, the script automatically forces serial execution because interactive plotting and multiprocessing do not mix reliably.

---

## 10. How to modify the code for a different material

All material-specific information lives in the `MATERIALS` dictionary near the top of the script.

Each material is defined by a `MaterialCase` entry:

```python
"al": MaterialCase(
    key="al",
    label="Aluminum",
    x_tc=(0.000, 0.010, 0.020, 0.030, 0.040, 0.050),
    rho=2767.99,
    diameter=0.003175,
    length=0.150,
    c_p=896.0,
    plot_time=300.0,
    temp_files=("50al1", "50al2", "50al3", "75al1", "75al2", "75al3", "100al1", "100al2", "100al3"),
    temp_v_guess=(5.0,) * 9,
    temp_ymax=(60.0, 60.0, 60.0, 90.0, 90.0, 90.0, 120.0, 120.0, 120.0),
    temp_tinf=20.5,
    wind_files=("al1", "al2", "al3", "al4", "al5", "al6", "al7", "al8", "al9", "al10"),
    wind_v_guess=(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0),
    wind_ymax=(90.0,) * 10,
    wind_tinf=20.6,
),
```

### Meaning of each field

| Field | Meaning | Units |
|---|---|---|
| `key` | short material key used on the command line | dimensionless |
| `label` | human-readable material name | dimensionless |
| `x_tc` | six thermocouple locations from the heated boundary | m |
| `rho` | density | kg/m^3 |
| `diameter` | rod diameter | m |
| `length` | rod length | m |
| `c_p` | specific heat | J/(kg K) |
| `plot_time` | maximum time shown on plots | s |
| `temp_files` | historical expected filenames for temperature tests | dimensionless |
| `temp_v_guess` | air-speed guesses for those files | m/s |
| `temp_ymax` | plot y-axis maxima for those files | deg C |
| `temp_tinf` | ambient/reference temperature added for plotting | deg C |
| `wind_files` | historical expected filenames for wind-speed tests | dimensionless |
| `wind_v_guess` | air-speed guesses for wind-speed tests | m/s |
| `wind_ymax` | plot y-axis maxima for wind-speed tests | deg C |
| `wind_tinf` | ambient/reference temperature for wind-speed tests | deg C |

### Add a new material

Suppose you want to add brass data with filenames like:

```text
50br1.csv
75br1.csv
100br1.csv
```

Add a new entry to `MATERIALS`:

```python
"br": MaterialCase(
    key="br",
    label="Brass",
    x_tc=(0.000, 0.010, 0.020, 0.030, 0.040, 0.050),
    rho=8500.0,
    diameter=0.003175,
    length=0.150,
    c_p=380.0,
    plot_time=300.0,
    temp_files=("50br1", "75br1", "100br1"),
    temp_v_guess=(5.0, 5.0, 5.0),
    temp_ymax=(60.0, 90.0, 120.0),
    temp_tinf=20.5,
    wind_files=("br1", "br2", "br3", "br4", "br5", "br6", "br7", "br8", "br9", "br10"),
    wind_v_guess=(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0),
    wind_ymax=(90.0,) * 10,
    wind_tinf=20.6,
),
```

Then update the command-line choices in `parse_args()`:

```python
parser.add_argument("--material", choices=("al", "cu", "ss", "br", "all"), ...)
```

Finally, update `infer_material_key_from_filename()` so discovery mode recognizes brass filenames:

```python
def infer_material_key_from_filename(stem: str) -> str | None:
    lower = stem.lower()
    if "ss" in lower:
        return "ss"
    if "cu" in lower:
        return "cu"
    if "al" in lower:
        return "al"
    if "br" in lower:
        return "br"
    return None
```

Now this will work:

```bash
python oneDkhEstimator.py --material br
```

and this will include brass too:

```bash
python oneDkhEstimator.py --material all
```

provided `br` has been added to the `choices` tuple and to `MATERIALS`.

---

## 11. How to modify the code for different thermocouple locations

Change the `x_tc` tuple for the material.

Example:

```python
x_tc=(0.000, 0.012, 0.024, 0.036, 0.048, 0.060)
```

Important constraints:

1. The current estimator assumes exactly six thermocouple channels.
2. The current CSV parser reads columns `TC1` through `TC6`.
3. The finite-difference boundary model uses `TC1`, `TC2`, and `TC3`.
4. The second-order boundary-gradient estimate assumes the first three thermocouple locations are uniformly spaced starting at the boundary location.

The boundary-gradient estimate is:

```python
boundary_gradient = (3.0 * tc[0] - 4.0 * tc[1] + tc[2]) / (2.0 * x_tc[1])
```

That formula is appropriate when:

```text
x_TC1 = 0
x_TC2 = dx
x_TC3 = 2 dx
```

If the first three thermocouples are not uniformly spaced, this boundary-gradient approximation should be generalized before trusting the fitted boundary model.

---

## 12. How to modify the code for more than six thermocouples

The current script is intentionally conservative: it accepts extra columns but only models the first six thermocouples.

To use more than six thermocouples, the following parts need to be updated together:

1. `LoadedData.tc` construction in `load_six_tc_csv()`.
2. The minimum required column count.
3. The `x_tc` tuple length in each `MaterialCase`.
4. The data stacking logic in `run_one_file()`.
5. The plotting loop in `plot_fit()`.
6. The boundary-gradient model if the near-boundary sensor layout changes.

The main reason this is not automatic is that extra acquisition channels may not all be thermocouples. They may include heater voltage, current, PWM duty, ambient temperature, fan speed, or diagnostic channels. Those should be mapped explicitly rather than guessed.

---

## 13. Changing optimization behavior

The default optimizer method is now:

```bash
--method trf
```

`trf` supports bounds and is safer for production use because it prevents negative `h`, `k`, and `Pss` during the transient fit.

The older Levenberg-Marquardt style can be requested with:

```bash
--method lm
```

The `lm` method does not enforce bounds in SciPy's `least_squares`, so it may explore nonphysical parameter values. The residual functions try to guard against that, but `trf` is usually preferable.

Limit the number of function evaluations for quick tests:

```bash
python oneDkhEstimator.py --max-nfev 50
```

---

## 14. Suggested validation workflow after editing materials

After adding or changing a material, run this sequence:

### 1. Syntax check

```bash
python -m py_compile oneDkhEstimator.py
```

### 2. Fast smoke test on one file

```bash
python oneDkhEstimator.py \
  --material br \
  --files 50br1 \
  --N 10 \
  --max-nfev 30 \
  --jobs 1
```

### 3. Full single-file run

```bash
python oneDkhEstimator.py \
  --material br \
  --files 50br1 \
  --N 100 \
  --jobs 1
```

### 4. Full batch run

```bash
python oneDkhEstimator.py \
  --material br \
  --N 100 \
  --jobs 0
```

Then inspect:

- fitted `h`, `k`, and `Pss`,
- transient RMS error,
- the PDF overlays in `FIGS/`,
- whether the fitted `k` is physically plausible,
- whether the model tracks all thermocouple histories, not just `TC1`.

---

## 15. Troubleshooting

### No files were processed

Check that:

- the data directory is correct,
- files end in `.csv`,
- filenames contain a recognized material string such as `al`, `cu`, or `ss`,
- or run explicit files with `--files`.

Example:

```bash
python oneDkhEstimator.py \
  --material al \
  --files my_file_without_al_in_the_name.csv
```

### The parser says there are no numeric data rows

The parser needs at least eight numeric columns in each data row:

```text
time, TC1, TC2, TC3, TC4, TC5, TC6, duty_cycle
```

If your file has a text header, that is fine. But once data begins, the first eight columns must be numeric.

### Fit succeeds but parameters are unrealistic

Check:

- thermocouple locations `x_tc`,
- material density `rho`,
- specific heat `c_p`,
- diameter and length,
- whether the temperature data are temperature rises or absolute temperatures,
- whether `Tinf` is only being used for plotting,
- whether the early boundary response is well captured by the boundary model,
- whether the first three thermocouples are actually suitable for the finite-difference boundary estimate.

### Parallel run is not faster

Try:

```bash
--jobs 1
--jobs 3
--jobs 6
--jobs 9
```

On some systems, NumPy/SciPy may use threaded numerical libraries internally. In that case, too many Python worker processes can oversubscribe the CPU and memory bandwidth.

If oversubscription seems likely, try launching with threaded BLAS limited to one thread per worker:

```bash
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
python oneDkhEstimator.py --material all --jobs 9
```

---

## 16. Version notes

### v2.3.0

- Directory discovery is now the default.
- Startup reports the number of selected files instead of expected-but-missing files.
- Added `--expected-list` for legacy hard-coded file-list behavior.
- Added `--report-missing` for optional missing-file reports.
- Automatic worker count is capped by the number of selected files.
- README expanded with material-editing instructions.

### v2.2.1

- Added multiprocessing.
- Added metadata/header preamble support.
- Added support for files with extra trailing columns.
- Added duty-cycle summary columns.

---

## 17. Important modeling caveats

The code modernization does not, by itself, rederive or revalidate the physical model. It preserves the original modeling structure and makes the workflow more robust. Any change in geometry, sensor layout, boundary conditions, heater drive behavior, or acquisition format should be checked against the assumptions in the model before relying on the fitted parameters.
