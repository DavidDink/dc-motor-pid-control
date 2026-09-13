"""Serial test harness for DC motor PID step-response characterization."""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import serial

# --- Serial connection ---
SERIAL_PORT = "/dev/cu.usbmodem1101"
BAUD_RATE = 9600
SERIAL_READ_TIMEOUT_SEC = 0.1
BOOT_DELAY_SEC = 2.0  # opening the port resets the Arduino; let it finish booting before use

# --- Test matrix ---
# Commented-out block below is the original full-matrix defaults, kept here to restore easily
# after trimming the active values down for a quicker/smaller test run.
## STEP_SIZES_DEG = [10, 45, 90, 180, 360]
## DIRECTIONS = [1, -1]
## STARTING_CONDITIONS = ["rest", "in_motion_same", "in_motion_reversal"]
## REPEATS = 3
STEP_SIZES_DEG = [10,45,90,180,360]
DIRECTIONS = [1,-1]
STARTING_CONDITIONS = ["rest","in_motion_same","in_motion_reversal"]
REPEATS = 5

INTERRUPT_DELAY_SEC = 0.3  # time to let the motor get moving before interrupting it mid-move

# --- Settling / capture tuning ---
TOLERANCE_DEG = 3.0
OVERSHOOT_THRESHOLD_DEG = 2.0  # how far past target counts as overshoot, vs. just settling noise
MIN_SETTLE_SAMPLES = 5
CAPTURE_TIMEOUT_SEC = 10.0  # safety ceiling only; capture normally ends on genuine settle
REST_SETTLE_TIMEOUT_SEC = 10.0
STEADY_STATE_WINDOW_SEC = 1.0

SCRIPT_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = SCRIPT_DIR.parent / "pid_test_logs"

# --- Motor spec ---
MOTOR_MAX_RPM = 130
MAX_ANGULAR_SPEED_DEG_PER_SEC = MOTOR_MAX_RPM * 360 / 60

@dataclass
class StepTest:
    """One entry in the test matrix: a single step-response test to run and log."""

    step_size: float
    direction: int
    start_condition: str
    rep: int

    @property
    def target(self) -> float:
        """Signed target angle in degrees (step_size scaled by direction)."""
        return self.step_size * self.direction


def build_test_matrix() -> list[StepTest]:
    """Build the full list of tests to run: every combination of step size, direction,
    starting condition, and repeat count from the globals above (a full cross-product,
    so changing any one list changes the total test count multiplicatively)."""
    return [
        StepTest(step_size=size, direction=direction, start_condition=start, rep=rep)
        for size in STEP_SIZES_DEG
        for direction in DIRECTIONS
        for start in STARTING_CONDITIONS
        for rep in range(1, REPEATS + 1)
    ]


def connect(port: str = SERIAL_PORT, baud_rate: int = BAUD_RATE) -> serial.Serial:
    """Open the serial connection to the Arduino and wait for it to finish rebooting.

    Opening a USB-serial port resets the Arduino, so we sleep for BOOT_DELAY_SEC before
    doing anything else, then clear out any boot-time garbage bytes it printed while resetting.
    """
    ser = serial.Serial(port, baud_rate, timeout=SERIAL_READ_TIMEOUT_SEC)
    time.sleep(BOOT_DELAY_SEC)
    ser.reset_input_buffer()
    return ser


def send_target(ser: serial.Serial, command_id: int, target_angle: float) -> int:
    """Send a new target angle to the Arduino and return the next command id.

    command_id is threaded through explicitly (passed in, returned incremented) rather than
    kept as a global, so callers can always tell exactly which command a given response
    line belongs to.
    """
    command_id += 1
    ser.write(f"{target_angle}\n".encode())
    return command_id


def query_firmware_params(ser: serial.Serial, timeout_sec: float = 2.0) -> dict:
    """Ask the Arduino for its live Kp/Ki/Kd/friction/tolerance via the 'P' query command.

    Telemetry lines keep streaming during the query, so we read until we see the
    "PARAMS," line rather than assuming the next line is the response.
    """
    ser.reset_input_buffer()
    ser.write(b"P\n")
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        line = ser.readline().decode(errors="ignore").strip()
        if not line.startswith("PARAMS,"):
            continue
        _, kp, ki, kd, friction, tolerance = line.split(",")
        return {
            "kp": float(kp),
            "ki": float(ki),
            "kd": float(kd),
            "friction": float(friction),
            "tolerance": float(tolerance),
        }
    raise TimeoutError("Arduino did not respond to params query")


def capture_response(
    ser: serial.Serial,
    expected_command_id: int,
    tolerance_deg: float = TOLERANCE_DEG,
    min_settle_samples: int = MIN_SETTLE_SAMPLES,
    timeout_sec: float = CAPTURE_TIMEOUT_SEC,
    post_settle_window_sec: float = STEADY_STATE_WINDOW_SEC,
) -> tuple[pd.DataFrame, bool]:
    """Read telemetry lines for one command and capture its step response.

    Blocks, reading lines from the Arduino, until either:
      - the response has genuinely settled (min_settle_samples consecutive samples within
        tolerance_deg of target) and an extra post_settle_window_sec of data past that point
        has been captured (so later steady-state calculations have real settled data to
        average instead of reaching back into the transient), or
      - timeout_sec elapses with no settle (the test is considered failed/still moving).

    Returns the captured samples as a DataFrame plus whether it actually settled.
    """
    rows = []
    consecutive_in_band = 0
    settled = False
    settled_t_ms: float | None = None
    deadline = time.time() + timeout_sec

    while time.time() < deadline:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue  # readline() timed out (SERIAL_READ_TIMEOUT_SEC) with nothing received

        parts = line.split(",")
        if len(parts) != 4:
            continue  # not a well-formed commandID,millis,currentAngle,targetAngle telemetry line
        try:
            line_command_id = int(float(parts[0]))
            t_ms, current_angle, target_angle = (float(p) for p in parts[1:])
        except ValueError:
            continue  # a line torn mid-write by the serial buffer; drop it rather than crash

        if line_command_id != expected_command_id:
            continue  # stale line from a previous command; association is by ID only, never by timing

        rows.append((t_ms, current_angle, target_angle))
        error = abs(current_angle - target_angle)
        consecutive_in_band = consecutive_in_band + 1 if error <= tolerance_deg else 0
        if not settled and consecutive_in_band >= min_settle_samples:
            settled = True
            settled_t_ms = t_ms

        # Keep recording past the settle point so compute_metrics' steady-state window has
        # actual settled data to average, rather than reaching back into the transient.
        if settled and (t_ms - settled_t_ms) >= post_settle_window_sec * 1000:
            break

    df = pd.DataFrame(rows, columns=["t_ms", "current_angle", "target_angle"])
    if not df.empty:
        df["t_sec"] = (df["t_ms"] - df["t_ms"].iloc[0]) / 1000.0  # rebase to seconds since first sample
    return df, settled


def compute_metrics(df: pd.DataFrame, start_angle: float, tolerance_deg: float = TOLERANCE_DEG) -> dict:
    """Compute pass/fail and performance metrics for one captured step response.

    Returns a dict with:
      - dist_deg: distance travelled from start_angle to the final commanded target.
      - settling_time_sec: time of the first sample after which error stays within
        tolerance_deg for the rest of the capture (a later dip back out resets the search,
        so this is "enters the band and never leaves again," not "first touches the band").
        NaN if that never happens within the capture window.
      - asymptotic_settling_time_sec: settling_time_sec with the time the motor "should"
        spend just travelling dist_deg at its rated max speed subtracted off, isolating the
        extra time spent converging into tolerance from unavoidable travel time. NaN if the
        test didn't settle.
      - steady_state_error_deg: mean absolute error over the last STEADY_STATE_WINDOW_SEC of
        the capture (only meaningful as "steady state" if the test actually passed).
      - overshoot: True if the response passed target by more than OVERSHOOT_THRESHOLD_DEG in
        the direction of travel (small crossings within that threshold don't count).
      - passed: True iff settling_time_sec is not NaN, i.e. the response genuinely settled.
    """
    if df.empty:
        return {
            "dist_deg": np.nan,
            "settling_time_sec": np.nan,
            "asymptotic_settling_time_sec": np.nan,
            "steady_state_error_deg": np.nan,
            "overshoot": False,
            "passed": False,
        }

    target = df["target_angle"].iloc[-1]
    error = (df["current_angle"] - target).to_numpy()
    t = df["t_sec"].to_numpy()
    within_band = np.abs(error) <= tolerance_deg

    settling_time_sec = np.nan
    for idx in range(len(within_band)):
        # within_band[idx:].all() requires every remaining sample to also be in tolerance,
        # so a brief in-band dip that's later left again is correctly rejected as a real settle
        if within_band[idx] and within_band[idx:].all():
            settling_time_sec = t[idx]
            break

    final_window = t >= (t[-1] - STEADY_STATE_WINDOW_SEC)
    steady_state_error_deg = np.mean(np.abs(error[final_window])) if final_window.any() else np.nan

    # dist_deg is measured from start_angle -- the rest baseline (0) for from-rest tests, or the
    # intermediate in-motion setpoint for in-motion tests -- not the raw logged starting sample.
    # Subtracting the time to cover that distance at the motor's 130 rpm max speed isolates the
    # "extra" time spent settling into tolerance from the time just spent travelling at full speed.
    dist_deg = abs(target - start_angle)
    expected_full_speed_time_sec = dist_deg / MAX_ANGULAR_SPEED_DEG_PER_SEC
    asymptotic_settling_time_sec = (
        settling_time_sec - expected_full_speed_time_sec if not np.isnan(settling_time_sec) else np.nan
    )

    # only counts as overshoot past OVERSHOOT_THRESHOLD_DEG beyond target, so settling noise near
    # the target isn't mistaken for a real overshoot
    movement_direction = np.sign(target - start_angle)
    if movement_direction > 0:
        overshoot = bool(np.any(df["current_angle"].to_numpy() > target + OVERSHOOT_THRESHOLD_DEG))
    elif movement_direction < 0:
        overshoot = bool(np.any(df["current_angle"].to_numpy() < target - OVERSHOOT_THRESHOLD_DEG))
    else:
        overshoot = False

    return {
        "dist_deg": dist_deg,
        "settling_time_sec": settling_time_sec,
        "asymptotic_settling_time_sec": asymptotic_settling_time_sec,
        "steady_state_error_deg": steady_state_error_deg,
        "overshoot": overshoot,
        "passed": not np.isnan(settling_time_sec),
    }


def run_single_test(ser: serial.Serial, command_id: int, test: StepTest) -> tuple[pd.DataFrame, int, float]:
    """Drive the motor into the test's starting condition, then command the real target
    and capture the resulting step response.

    "rest" first sends the motor back to 0 and waits for it to settle there, so every rest
    test starts from a known, stationary baseline. "in_motion_same"/"in_motion_reversal"
    instead send an intermediate setpoint and only wait INTERRUPT_DELAY_SEC (not a full
    settle) before issuing the real target, so the motor is deliberately still moving when
    the test command lands -- that's the point of those two start conditions.

    Returns the captured response, the updated command_id, and the start_angle the response
    actually began from (0 for "rest", the intermediate setpoint otherwise) for use in
    compute_metrics' distance/direction calculations.
    """
    if test.start_condition == "rest":
        command_id = send_target(ser, command_id, 0)
        _, settled = capture_response(ser, command_id, timeout_sec=REST_SETTLE_TIMEOUT_SEC)
        if not settled:
            print(f"  WARNING: rest baseline never settled for {test}")
        start_angle = 0.0
    else:
        # "in_motion_same": approach the target from halfway along the same direction, so the
        # real command continues the motor's existing direction of travel.
        # "in_motion_reversal": approach from halfway in the opposite direction, so the real
        # command forces the motor to reverse mid-move instead of continuing straight through.
        sign = 1 if test.start_condition == "in_motion_same" else -1
        intermediate = sign * test.target / 2
        command_id = send_target(ser, command_id, intermediate)
        time.sleep(INTERRUPT_DELAY_SEC)
        start_angle = intermediate

    command_id = send_target(ser, command_id, test.target)
    df, settled = capture_response(ser, command_id)
    if df.empty:
        print(f"  WARNING: no data captured for {test}")
    elif not settled:
        print(f"  WARNING: {test} hit the {CAPTURE_TIMEOUT_SEC}s capture timeout without settling")

    return df, command_id, start_angle


def print_test_result(metrics: dict) -> None:
    """Print a one-line human-readable summary of a single test's compute_metrics() result."""
    pass_fail = "PASS" if metrics["passed"] else "FAIL"
    overshoot = "y" if metrics["overshoot"] else "n"
    print(
        f"  dist={metrics['dist_deg']:.2f}deg  {pass_fail}"
        f"  settling_time={metrics['settling_time_sec']:.3f}s"
        f"  steady_state_error={metrics['steady_state_error_deg']:.3f}deg"
        f"  overshoot={overshoot}"
    )


def make_run_dir() -> Path:
    """Create (and return) a fresh timestamped directory under OUTPUT_DIR for this run's
    output, so results from different runs land in separate folders instead of overwriting
    each other's CSVs/plots."""
    run_dir = OUTPUT_DIR / time.strftime("run_%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir


def write_params_file(run_dir: Path, firmware_params: dict) -> None:
    """Write the firmware-reported tolerance/friction/Kp/Ki/Kd for this run to params.txt,
    so each run folder records exactly which constants were flashed when it was collected."""
    lines = [
        f"tolerance_deg: {firmware_params['tolerance']}",
        f"friction: {firmware_params['friction']}",
        f"kp: {firmware_params['kp']}",
        f"ki: {firmware_params['ki']}",
        f"kd: {firmware_params['kd']}",
    ]
    (run_dir / "params.txt").write_text("\n".join(lines) + "\n")


def run_all_tests(ser: serial.Serial) -> tuple[pd.DataFrame, pd.DataFrame]:
    """Top-level test driver: query the firmware's live constants, run every test in the
    matrix in sequence, and save all of this run's output (per-test CSVs, the combined raw
    and summary CSVs, params.txt, and the three summary plots) into one fresh run folder.

    Returns the concatenated raw per-sample data and the per-test summary, in case the
    caller wants to inspect them further (e.g. interactively).
    """
    run_dir = make_run_dir()
    firmware_params = query_firmware_params(ser)
    write_params_file(run_dir, firmware_params)

    test_matrix = build_test_matrix()
    print(f"Total tests to run: {len(test_matrix)}")

    command_id = 0
    all_runs = []
    summary_rows = []

    for i, test in enumerate(test_matrix):
        print(f"Running test {i + 1}/{len(test_matrix)}: {test}")
        df, command_id, start_angle = run_single_test(ser, command_id, test)

        df = df.assign(
            test_id=i,
            step_size=test.step_size,
            direction=test.direction,
            target=test.target,
            start_condition=test.start_condition,
            rep=test.rep,
        )
        filename = (
            f"test_{i:03d}_size{test.step_size}_dir{test.direction}"
            f"_{test.start_condition}_rep{test.rep}.csv"
        )
        df.to_csv(run_dir / filename, index=False)
        all_runs.append(df)

        metrics = compute_metrics(df, start_angle)
        summary_rows.append(
            {
                "test_id": i,
                "step_size": test.step_size,
                "direction": test.direction,
                "target": test.target,
                "start_condition": test.start_condition,
                "rep": test.rep,
                **metrics,
            }
        )
        print_test_result(metrics)

    raw_df = pd.concat(all_runs, ignore_index=True)
    raw_df.to_csv(run_dir / "all_runs_raw.csv", index=False)

    summary_df = pd.DataFrame(summary_rows)
    summary_df.to_csv(run_dir / "summary.csv", index=False)

    print("All tests complete.")
    print_summary_statistics(summary_df)
    print_step_size_breakdown(summary_df, step_size=45)
    plot_steady_state_error_by_step_size(summary_df, run_dir)
    plot_settling_time_by_step_size(summary_df, run_dir)
    plot_overshoot_pct_by_step_size(summary_df, run_dir)
    return raw_df, summary_df


def print_summary_statistics(summary_df: pd.DataFrame) -> None:
    """Print aggregate pass rate, mean asymptotic settling time, mean steady-state error,
    and overshoot rate across every test in this run's summary_df."""
    n_total = len(summary_df)
    n_passed = int(summary_df["passed"].sum())
    pct_passed = 100 * n_passed / n_total if n_total else np.nan
    mean_asymptotic_settling_time = summary_df.loc[summary_df["passed"], "asymptotic_settling_time_sec"].mean()
    mean_steady_state_error = summary_df["steady_state_error_deg"].mean()
    pct_overshoot = 100 * summary_df["overshoot"].mean() if n_total else np.nan

    print("=== Test Summary Statistics ===")
    print(f"Tests run:                           {n_total}")
    print(f"Tests passed:                        {n_passed}")
    print(f"Success rate:                        {pct_passed:.1f}%")
    print(f"Mean asymptotic settling time (s):   {mean_asymptotic_settling_time:.3f}")
    print(f"Mean steady-state error (deg):       {mean_steady_state_error:.3f}")
    print(f"Overshoot rate:                      {pct_overshoot:.1f}%")


def _bar_chart_by_step_size(
    values_by_step: pd.Series, ylabel: str, title: str, filename: str, color: str, output_dir: Path
) -> None:
    """Shared helper: draw and save one bar chart of a metric already aggregated by step
    size (index = step size, values = the metric), used by each plot_* function below so
    they only need to supply the aggregation and labels."""
    x = values_by_step.index.astype(str)

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.bar(x, values_by_step.to_numpy(), color=color)
    ax.set_xlabel("Step Size (deg)")
    ax.set_ylabel(ylabel)
    ax.set_title(title)

    fig.tight_layout()
    fig.savefig(output_dir / filename, dpi=150)
    plt.show()


def plot_steady_state_error_by_step_size(summary_df: pd.DataFrame, output_dir: Path) -> None:
    """Plot and save mean steady-state error (deg) grouped by step size."""
    mean_error_by_step = summary_df.groupby("step_size")["steady_state_error_deg"].mean().sort_index()
    _bar_chart_by_step_size(
        mean_error_by_step,
        ylabel="Mean Steady-State Error (deg)",
        title="Mean Steady-State Error by Step Size",
        filename="steady_state_error_by_step_size.png",
        color="tab:blue",
        output_dir=output_dir,
    )


def plot_settling_time_by_step_size(summary_df: pd.DataFrame, output_dir: Path) -> None:
    """Plot and save mean (raw, not asymptotic) settling time (s) grouped by step size."""
    mean_settling_time_by_step = summary_df.groupby("step_size")["settling_time_sec"].mean().sort_index()
    _bar_chart_by_step_size(
        mean_settling_time_by_step,
        ylabel="Mean Settling Time (s)",
        title="Mean Settling Time by Step Size",
        filename="settling_time_by_step_size.png",
        color="tab:orange",
        output_dir=output_dir,
    )


def plot_overshoot_pct_by_step_size(summary_df: pd.DataFrame, output_dir: Path) -> None:
    """Plot and save the percentage of tests that overshot by more than
    OVERSHOOT_THRESHOLD_DEG, grouped by step size."""
    overshoot_pct_by_step = 100 * summary_df.groupby("step_size")["overshoot"].mean().sort_index()
    _bar_chart_by_step_size(
        overshoot_pct_by_step,
        ylabel=f"Overshoot (%, >{OVERSHOOT_THRESHOLD_DEG:g}° past target)",
        title=f"Percent Overshoot by Step Size (>{OVERSHOOT_THRESHOLD_DEG:g}° past target)",
        filename="overshoot_pct_by_step_size.png",
        color="tab:green",
        output_dir=output_dir,
    )


def print_step_size_breakdown(summary_df: pd.DataFrame, step_size: float) -> None:
    """Print descriptive statistics (count/mean/std/min/max/etc.) of settling_time_sec for
    just the tests at one specific step_size."""
    subset = summary_df[summary_df["step_size"] == step_size]["settling_time_sec"]
    print(f"=== Settling Time Breakdown (step_size={step_size}) ===")
    print(subset.describe())


def main() -> None:
    """Entry point: connect to the Arduino, run the full test matrix, and always close the
    serial port afterward (even if a test run raises) so the port isn't left locked."""
    ser = connect()
    print(f"Connected to {SERIAL_PORT}")
    try:
        run_all_tests(ser)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
