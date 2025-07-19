import sys

sys.dont_write_bytecode = True
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
from comparison import prepare_data, sgm
from scipy import stats
import os


def compute_ratios(data: dict, metric: str):
    categories = ["[0,1)", "[1,10)", "[10,100)", "[100,900)", "[900+)"]
    values_1 = {cat: [] for cat in categories}
    values_2 = {cat: [] for cat in categories}
    ratios = []

    for key in data:
        cat = data[key]["diff"]
        values_1[cat].append(data[key][f"{metric}_1"])
        values_2[cat].append(data[key][f"{metric}_2"])

    for cat in categories:
        sgm1 = sgm(values_1[cat]) if values_1[cat] else 1e-6
        sgm2 = sgm(values_2[cat]) if values_2[cat] else 1e-6
        ratios.append(sgm2 / sgm1)

    return categories, ratios


def plot_comparison(
    data: dict, names: list[str], out_file: str = "plots/comparison.svg"
):
    plt.style.use("default")
    plt.figure(figsize=(12, 8), facecolor="white")

    # --- 1. Scatter plot: time_1 vs time_2 ---
    ax1 = plt.subplot(1, 2, 1)
    x = [data[k]["time_1"] for k in data]
    y = [data[k]["time_2"] for k in data]
    ax1.scatter(x, y, alpha=0.8, s=10, label="Instances")
    ax1.plot([0.9, 1000], [0.9, 1000], "k--", linewidth=1)
    ax1.set_xlabel(names[0])
    ax1.set_ylabel(names[1])
    ax1.set_title("Scatter Plot of Time")
    ax1.grid(True, which="both", ls="--", lw=0.5)

    # Calculate best fit line in log space
    log_x = np.log10(x)
    log_y = np.log10(y)
    slope, intercept, r_value, p_value, std_err = stats.linregress(log_x, log_y)

    # Create fitted line
    x_fit = np.logspace(np.log10(min(x)), np.log10(max(x)), 100)
    y_fit = 10 ** (slope * np.log10(x_fit) + intercept)

    # Plot fitted line (red) and diagonal reference (black dashed)
    ax1.plot(
        x_fit,
        y_fit,
        "r-",
        linewidth=2,
        label=f"Best Fit: y ≈ {10**intercept:.2f} · x^{slope:.2f}\nR² = {r_value**2:.2f}",
    )
    min_time = min(min(x), min(y))
    max_time = max(max(x), max(y))
    ax1.plot([min_time, max_time], [min_time, max_time], "k--", linewidth=2, alpha=0.8)

    ax1.set_xscale("log")
    ax1.set_yscale("log")

    # Enhanced grid styling
    ax1.grid(True, which="major", alpha=0.4, linewidth=0.8)
    ax1.grid(True, which="minor", alpha=0.2, linewidth=0.4)
    ax1.set_axisbelow(True)

    # Set limits and ticks
    ax1.set_xlim(min_time * 0.8, max_time * 1.2)
    ax1.set_ylim(min_time * 0.8, max_time * 1.2)

    # Add legend
    ax1.legend(loc="upper left", frameon=True, fancybox=True, shadow=True, fontsize=9)

    # Styling improvements
    ax1.spines["top"].set_visible(True)
    ax1.spines["right"].set_visible(True)
    ax1.spines["top"].set_linewidth(0.5)
    ax1.spines["right"].set_linewidth(0.5)
    ax1.spines["bottom"].set_linewidth(0.8)
    ax1.spines["left"].set_linewidth(0.8)

    # --- 2. Bar plot: ratio of SGMs for time ---
    ax2 = plt.subplot(2, 2, 2)
    categories, time_ratios = compute_ratios(data, "time")

    # Filter out categories with no data (ratio of 1e-6/1e-6 = 1.0)
    display_categories = []
    display_ratios = []
    for cat, ratio in zip(categories, time_ratios):
        # Check if this category has actual data
        has_data = any(data[key]["diff"] == cat for key in data)
        if has_data:
            display_categories.append(cat)
            display_ratios.append(ratio)

    bars2 = ax2.bar(display_categories, display_ratios, color="steelblue", alpha=0.7)
    ax2.set_ylabel("Time ratio")
    ax2.set_title("Time Ratio Comparison")
    ax2.set_ylim(0, max(display_ratios) * 1.1 if display_ratios else 1.1)

    # Rotate x-axis labels for better readability
    ax2.tick_params(axis="x", rotation=45)

    # Add value labels on bars
    for bar, ratio in zip(bars2, display_ratios):
        height = bar.get_height()
        ax2.text(
            bar.get_x() + bar.get_width() / 2.0,
            height,
            f"{ratio:.2f}",
            ha="center",
            va="bottom",
            fontsize=9,
        )

    # --- 3. Bar plot: ratio of SGMs for nodes ---
    ax3 = plt.subplot(2, 2, 4)
    categories, nodes_ratios = compute_ratios(data, "nnodes")

    # Filter out categories with no data (same logic as time plot)
    nodes_display_categories = []
    nodes_display_ratios = []
    for cat, ratio in zip(categories, nodes_ratios):
        # Check if this category has actual data
        has_data = any(data[key]["diff"] == cat for key in data)
        if has_data:
            nodes_display_categories.append(cat)
            nodes_display_ratios.append(ratio)

    bars3 = ax3.bar(
        nodes_display_categories, nodes_display_ratios, color="forestgreen", alpha=0.7
    )
    ax3.set_ylabel("Nodes ratio")
    ax3.set_title("Nodes Ratio Comparison")
    ax3.set_ylim(0, max(1, max(nodes_display_ratios)) * 1.1)

    # Rotate x-axis labels for better readability
    ax3.tick_params(axis="x", rotation=45)

    # Add value labels on bars
    for bar, ratio in zip(bars3, nodes_display_ratios):
        height = bar.get_height()
        ax3.text(
            bar.get_x() + bar.get_width() / 2.0,
            height,
            f"{ratio:.2f}",
            ha="center",
            va="bottom",
            fontsize=9,
        )

    plt.tight_layout()
    plt.savefig(out_file)
    print(f"Saved plot to {out_file}")


def main():
    if len(sys.argv) != 3:
        print(f"Usage: {Path(__file__).name} <json1> <json2>")
        sys.exit(1)

    json1, json2 = sys.argv[1], sys.argv[2]
    names = [Path(json1).stem, Path(json2).stem]
    data = prepare_data(json1, json2, "time")
    os.makedirs("plots", exist_ok=True)
    plot_comparison(data, names)


if __name__ == "__main__":
    main()
