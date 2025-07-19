import sys

sys.dont_write_bytecode = True
import numpy as np
from pathlib import Path
from comparison import load_data_from_file, filter_data
import matplotlib.pyplot as plt
import os


def load_data(files: list[str]) -> dict:
    data = {}
    for file in files:
        run = Path(file).stem
        data[run] = filter_data(load_data_from_file(file), [0])
    return data


def get_times(data: dict) -> tuple[list[str], list[list[float]]]:
    run_names = []
    times = []

    for run in data:
        run_names.append(run)
        run_times = []
        for inst in data[run]:
            run_times.append(data[run][inst]["time"])
        times.append(run_times)

    return run_names, times


def plot(
    runs: list[str], times: list[list[float]], out_file: str = "plots/times.svg"
) -> None:
    plt.figure(figsize=(10, 8))
    styles = [
        {
            "linestyle": "-",
            "marker": "*",
            "color": "black",
            "markevery": 25,
        },
        {
            "linestyle": "--",
            "marker": "+",
            "color": "red",
            "markevery": 25,
        },
        {
            "linestyle": ":",
            "marker": "o",
            "color": "blue",
            "markevery": 25,
        },
        {"linestyle": "-.", "marker": None, "color": "green"},
        {
            "linestyle": "-",
            "marker": "x",
            "color": "magenta",
            "markevery": 25,
        },
    ]

    x_points = np.logspace(-2, 3, 900)
    low_ylimit = 3085
    high_ylimit = 0

    for i, (run, run_times) in enumerate(zip(runs, times)):
        style_idx = i % len(styles)
        y_points = []

        for threshold in x_points:
            count = sum(1 for t in run_times if t <= threshold)
            y_points.append(count)
            if threshold > 0.5 and count < low_ylimit:
                low_ylimit = count
            if high_ylimit < count:
                high_ylimit = count

        plt.semilogx(x_points, y_points, **styles[style_idx], linewidth=1.5, label=run)

    plt.grid(True, which="both", ls="--", lw=0.5)
    plt.tick_params(axis="both", which="major", length=6, direction="in")
    plt.tick_params(axis="both", which="minor", length=3, direction="in")
    plt.tick_params(top=True, right=True, which="major")
    plt.tick_params(top=True, right=True, which="minor")
    plt.legend(loc="lower right", bbox_to_anchor=(0.98, 0.04), frameon=True)
    plt.xlim(0.5, 2000)
    plt.ylim(low_ylimit - 50, high_ylimit + 50)
    plt.xticks([1, 10, 100, 1000], ["$10^0$", "$10^1$", "$10^2$", "$10^3$"])
    plt.xlabel("Time (s)")
    plt.ylabel("Problems Solved")

    for spine in plt.gca().spines.values():
        spine.set_visible(True)
        spine.set_color("black")

    plt.tight_layout()
    plt.savefig(out_file)
    print(f"Saved plot to {out_file}")


def main():
    if len(sys.argv) < 3:
        print(f"Usage: > python3 {Path(__file__).name} <json1> ...")
        exit(1)

    data = load_data(sys.argv[1:])
    runs, times = get_times(data)
    os.makedirs("plots", exist_ok=True)
    plot(runs, times)


if __name__ == "__main__":
    main()
