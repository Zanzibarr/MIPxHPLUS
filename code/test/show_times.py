import sys

sys.dont_write_bytecode = True
import numpy as np
from pathlib import Path
from comparison import load_data_from_file, filter_data
import matplotlib.pyplot as plt
import matplotlib as mpl
import os

# Configure matplotlib for LaTeX compatibility
plt.rcParams.update(
    {
        "font.family": "serif",
        "font.serif": ["Computer Modern Roman"],
        "text.usetex": True,
        "font.size": 11,  # Match your document's font size
        "axes.labelsize": 11,
        "axes.titlesize": 11,
        "legend.fontsize": 10,
        "xtick.labelsize": 10,
        "ytick.labelsize": 10,
        "figure.figsize": (6, 4.5),  # Better aspect ratio for thesis
        "axes.linewidth": 0.8,
        "grid.linewidth": 0.5,
        "lines.linewidth": 1.2,
    }
)

labels = {
    "002_2.2.0:4_tl_e__250611": r"$\mathrm{TL}^e$",
    "002_2.2.0:4_ve_e__250612": r"$\mathrm{VE}^e$",
    "002_2.2.0:2_ve_e_h__250608": r"$\mathrm{VE}^e_h$",
    "002_2.2.0:2_tl_e_h__250609": r"$\mathrm{TL}^e_h$",
    "002_2.2.0:4_sec_e_h__250614": r"$\mathrm{SEC}^e_h$",
    "002_2.2.0:4_lm_e_h__250613": r"$\mathrm{LM}^e_h$",
    "002_2.2.0:4_lms_e_h_candsecfacts__250610": r"$\mathrm{LMS}^e_h$",
}


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
    runs: list[str], times: list[list[float]], out_file: str = "plots/times.pdf"
) -> None:
    fig, ax = plt.subplots(figsize=(6, 4.5))

    # Professional color scheme
    colors = [
        "#1f77b4",
        "#ff7f0e",
        "#2ca02c",
        "#d62728",
        "#9467bd",
        "#8c564b",
        "#e377c2",
    ]

    styles = [
        {"linestyle": "-", "marker": "*", "markevery": 25, "markersize": 6},
        {"linestyle": "--", "marker": "+", "markevery": 25, "markersize": 8},
        {"linestyle": ":", "marker": "o", "markevery": 25, "markersize": 5},
        {"linestyle": "-.", "marker": "s", "markevery": 25, "markersize": 5},
        {"linestyle": "-", "marker": "x", "markevery": 25, "markersize": 6},
        {"linestyle": "--", "marker": "^", "markevery": 25, "markersize": 5},
        {"linestyle": ":", "marker": "d", "markevery": 25, "markersize": 5},
    ]

    x_points = np.logspace(-2, 3, 900)
    low_ylimit = 3085
    high_ylimit = 0

    for i, (run, run_times) in enumerate(zip(runs, times)):
        color = colors[i % len(colors)]
        style = styles[i % len(styles)]
        y_points = []

        for threshold in x_points:
            count = sum(1 for t in run_times if t <= threshold)
            y_points.append(count)
            if threshold > 0.5 and count < low_ylimit:
                low_ylimit = count
            if high_ylimit < count:
                high_ylimit = count

        ax.semilogx(
            x_points,
            y_points,
            color=color,
            label=labels[run] if run in labels else run,
            **style,
        )

    # Grid styling to match academic papers
    ax.grid(True, which="major", linestyle="-", alpha=0.3)
    ax.grid(True, which="minor", linestyle=":", alpha=0.2)

    # Tick styling
    ax.tick_params(axis="both", which="major", length=4, direction="in")
    ax.tick_params(axis="both", which="minor", length=2, direction="in")
    ax.tick_params(top=True, right=True, which="major")
    ax.tick_params(top=True, right=True, which="minor")

    # Professional legend
    legend = ax.legend(
        loc="lower right",
        frameon=True,
        fancybox=False,
        shadow=False,
        framealpha=1.0,
        edgecolor="black",
    )
    legend.get_frame().set_linewidth(0.5)

    ax.set_xlim(0.5, 2000)
    ax.set_ylim(low_ylimit - 50, high_ylimit + 50)

    # Better tick labels
    ax.set_xticks([1, 10, 100, 1000])
    ax.set_xticklabels([r"$10^0$", r"$10^1$", r"$10^2$", r"$10^3$"])

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Problems Solved")

    # Clean spines
    for spine in ax.spines.values():
        spine.set_visible(True)
        spine.set_color("black")
        spine.set_linewidth(0.8)

    plt.tight_layout()

    # Save as PDF for better quality in LaTeX
    plt.savefig(
        out_file,
        format="pdf",
        dpi=300,
        bbox_inches="tight",
        facecolor="white",
        edgecolor="none",
    )
    print(f"Saved plot to {out_file}")
    plt.close()  # Free memory


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
