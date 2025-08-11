import sys

sys.dont_write_bytecode = True

import json
from pathlib import Path
import numpy as np


def load_data_from_file(json_file: str) -> dict:
    """Load data from JSON file, same as in comparison.py"""
    if not Path(json_file).is_file():
        print(f"ERROR: {json_file} is not an existing file.")
        exit(1)

    with open(json_file) as f:
        data = json.load(f)
    if "results" not in data:
        print(f"Cannot find key 'results' in this json file.")
        exit(1)

    return data["results"]


def filter_data(data: dict, status_list: list[int] = [0, 2]) -> dict:
    """Filter data by status, same as in comparison.py"""
    copy = {}
    for key in data:
        if data[key]["status"] not in status_list:
            continue
        copy[key] = data[key]
        if data[key]["status"] not in (0, 1) or data[key]["time"] > 900:
            copy[key]["time"] = 900

    return copy


def sgm(values: list[float]) -> float:
    """Compute shifted geometric mean, same as in comparison.py"""
    return np.exp(np.mean(np.log([x + 1 for x in values]))) - 1


def categorize(data: dict) -> dict:
    """Categorize instances by time ranges, same as in comparison.py"""
    new_data = data

    for key in new_data:
        time_val = new_data[key]["time"]
        if time_val < 1:
            new_data[key]["diff"] = "[0,1)"
        elif time_val < 10:
            new_data[key]["diff"] = "[1,10)"
        elif time_val < 100:
            new_data[key]["diff"] = "[10,100)"
        elif time_val < 900:
            new_data[key]["diff"] = "[100,900)"
        else:
            new_data[key]["diff"] = "[900+)"

    return new_data


def compute_sgm_by_category(
    data: dict, metric_key: str
) -> tuple[list[str], list[float], dict]:
    """
    Compute SGM for each time category.
    Returns categories, SGM values, and detailed stats.
    """
    categories = ["[0,1)", "[1,10)", "[10,100)", "[100,900)", "[900+)"]
    values_by_cat = {cat: [] for cat in categories}
    sgm_values = []

    # Group values by category
    for instance_key in data:
        if metric_key in data[instance_key] and "diff" in data[instance_key]:
            cat = data[instance_key]["diff"]
            values_by_cat[cat].append(data[instance_key][metric_key])

    # Compute SGM for each category
    for cat in categories:
        if values_by_cat[cat]:
            sgm_val = sgm(values_by_cat[cat])
        else:
            sgm_val = 0.0
        sgm_values.append(sgm_val)

    return categories, sgm_values, values_by_cat


def print_category_table(
    categories: list[str],
    values_by_cat: dict,
    sgm_values: list[float],
    metric_name: str,
    description: str,
) -> None:
    """Print results table by category, similar to comparison.py"""
    print("=" * 70)
    print(f"{metric_name.upper()} ANALYSIS BY TIME CATEGORY ({description})")
    print("=" * 70)
    print(f"{'Category':<15} || {'Count':>10} || {'SGM':>15}")
    print("-" * 70)

    overall_values = []
    for i, cat in enumerate(categories):
        count = len(values_by_cat[cat])
        sgm_val = sgm_values[i]
        overall_values.extend(values_by_cat[cat])
        print(f"{cat:<15} || {count:>10} || {sgm_val:>15.3f}")

    # Overall SGM
    overall_sgm = sgm(overall_values) if overall_values else 0.0
    print("-" * 70)
    print(f"{'OVERALL':<15} || {len(overall_values):>10} || {overall_sgm:>15.3f}")
    print("=" * 70)

    return overall_sgm


def compute_nconst_acyc(data: dict) -> tuple[dict, list[float]]:
    """
    Extract nconst_acyc values for each instance.
    Returns the modified data dict and a list of nconst_acyc values.
    """
    values = []

    for instance_key in data:
        instance_data = data[instance_key]

        # Check if required metric exists
        if "nconst_acyc" not in instance_data:
            print(f"Warning: Missing nconst_acyc metric in instance {instance_key}")
            continue

        nconst_acyc_val = instance_data["nconst_acyc"]
        values.append(nconst_acyc_val)

    return data, values


def compute_usercuts_sum(data: dict) -> tuple[dict, list[float]]:
    """
    Compute sum of usercuts_lm and usercuts_sec for each instance.
    Returns the modified data dict and a list of sum values.
    """
    sum_values = []

    for instance_key in data:
        instance_data = data[instance_key]

        # Check if required metrics exist
        if "usercuts_lm" not in instance_data or "usercuts_sec" not in instance_data:
            print(f"Warning: Missing usercuts metrics in instance {instance_key}")
            continue

        # Compute sum
        usercuts_sum = instance_data["usercuts_lm"] + instance_data["usercuts_sec"]
        data[instance_key]["usercuts_sum"] = usercuts_sum
        sum_values.append(usercuts_sum)

    return data, sum_values


def print_statistics(
    values: list[float], metric_name: str, description: str = ""
) -> None:
    """Print detailed statistics about the given values"""
    total_instances = len(values)

    if total_instances == 0:
        print(f"No valid instances found with {metric_name} metric.")
        return

    # Basic statistics
    min_val = min(values)
    max_val = max(values)
    mean_val = np.mean(values)
    median_val = np.median(values)
    std_val = np.std(values)
    sgm_val = sgm(values)

    header = f"{metric_name.upper()} ANALYSIS"
    if description:
        header += f" ({description})"

    print("=" * 70)
    print(header)
    print("=" * 70)
    print(f"Total instances processed: {total_instances}")
    print(f"Minimum value:            {min_val}")
    print(f"Maximum value:            {max_val}")
    print(f"Arithmetic mean:          {mean_val:.3f}")
    print(f"Median:                   {median_val}")
    print(f"Standard deviation:       {std_val:.3f}")
    print(f"Shifted geometric mean:   {sgm_val:.3f}")
    print("=" * 70)

    # Distribution analysis based on metric type
    if metric_name.lower() == "usercuts":
        zero_vals = sum(1 for x in values if x == 0)
        low_vals = sum(1 for x in values if 0 < x <= 10)
        medium_vals = sum(1 for x in values if 10 < x <= 100)
        high_vals = sum(1 for x in values if x > 100)

        print("DISTRIBUTION:")
        print(
            f"Zero cuts (0):           {zero_vals:4d} ({100*zero_vals/total_instances:.1f}%)"
        )
        print(
            f"Low cuts (1-10):         {low_vals:4d} ({100*low_vals/total_instances:.1f}%)"
        )
        print(
            f"Medium cuts (11-100):    {medium_vals:4d} ({100*medium_vals/total_instances:.1f}%)"
        )
        print(
            f"High cuts (>100):        {high_vals:4d} ({100*high_vals/total_instances:.1f}%)"
        )
    else:
        zero_vals = sum(1 for x in values if x == 0)
        low_vals = sum(1 for x in values if 0 < x <= 100)
        medium_vals = sum(1 for x in values if 100 < x <= 1000)
        high_vals = sum(1 for x in values if x > 1000)

        print("DISTRIBUTION:")
        print(
            f"Zero values (0):         {zero_vals:4d} ({100*zero_vals/total_instances:.1f}%)"
        )
        print(
            f"Low values (1-100):      {low_vals:4d} ({100*low_vals/total_instances:.1f}%)"
        )
        print(
            f"Medium values (101-1000): {medium_vals:4d} ({100*medium_vals/total_instances:.1f}%)"
        )
        print(
            f"High values (>1000):     {high_vals:4d} ({100*high_vals/total_instances:.1f}%)"
        )

    print("=" * 70)


def print_detailed_instances_nconst(
    data: dict, values: list[float], limit: int = 10
) -> None:
    """Print detailed information for instances with highest nconst_acyc values"""
    instances_with_nconst = []

    for instance_key in data:
        if "nconst_acyc" in data[instance_key]:
            instances_with_nconst.append(
                (
                    instance_key,
                    data[instance_key]["nconst_acyc"],
                    data[instance_key].get("time", -1),
                    data[instance_key].get("status", -1),
                )
            )

    # Sort by nconst_acyc in descending order
    instances_with_nconst.sort(key=lambda x: x[1], reverse=True)

    print(f"TOP {min(limit, len(instances_with_nconst))} INSTANCES BY NCONST_ACYC:")
    print("-" * 80)
    print(f"{'Instance':<50} {'nconst_acyc':>12} {'Time':>8} {'Status':>6}")
    print("-" * 80)

    for i, (instance, nconst, time, status) in enumerate(instances_with_nconst[:limit]):
        print(f"{instance:<50} {nconst:12d} {time:8.1f} {status:6d}")

    print("-" * 80)


def print_detailed_instances(data: dict, limit: int = 10) -> None:
    """Print detailed information for instances with highest usercuts sum"""
    instances_with_cuts = []

    for instance_key in data:
        if "usercuts_sum" in data[instance_key]:
            instances_with_cuts.append(
                (
                    instance_key,
                    data[instance_key]["usercuts_sum"],
                    data[instance_key].get("usercuts_lm", 0),
                    data[instance_key].get("usercuts_sec", 0),
                    data[instance_key].get("time", -1),
                    data[instance_key].get("status", -1),
                )
            )

    # Sort by usercuts_sum in descending order
    instances_with_cuts.sort(key=lambda x: x[1], reverse=True)

    print(f"TOP {min(limit, len(instances_with_cuts))} INSTANCES BY USERCUTS SUM:")
    print("-" * 100)
    print(f"{'Instance':<40} {'Sum':>6} {'LM':>6} {'SEC':>6} {'Time':>8} {'Status':>6}")
    print("-" * 100)

    for i, (instance, total, lm, sec, time, status) in enumerate(
        instances_with_cuts[:limit]
    ):
        print(f"{instance:<40} {total:6d} {lm:6d} {sec:6d} {time:8.1f} {status:6d}")

    print("-" * 100)


def main():
    if len(sys.argv) != 3:
        print(f"Usage: > python3 {Path(__file__).name} <json_file1> <json_file2>")
        print("json_file1: Computes SGM of nconst_acyc metric by time category")
        print(
            "json_file2: Computes SGM of sum of usercuts_lm and usercuts_sec by time category"
        )
        exit(1)

    json_file1 = sys.argv[1]  # For nconst_acyc analysis
    json_file2 = sys.argv[2]  # For usercuts analysis

    # Process first file - nconst_acyc analysis
    print("PROCESSING FIRST FILE - NCONST_ACYC ANALYSIS")
    print("=" * 70)
    raw_data1 = load_data_from_file(json_file1)
    filtered_data1 = filter_data(raw_data1, [0, 2])  # Only successful instances
    categorized_data1 = categorize(filtered_data1)

    data1_with_nconst, nconst_values = compute_nconst_acyc(categorized_data1)

    # Compute and print SGM by category for nconst_acyc
    categories1, sgm_values1, values_by_cat1 = compute_sgm_by_category(
        data1_with_nconst, "nconst_acyc"
    )
    overall_sgm1 = print_category_table(
        categories1,
        values_by_cat1,
        sgm_values1,
        "nconst_acyc",
        "acyclicity constraints",
    )

    print("\n" + "=" * 70 + "\n")

    # Process second file - usercuts analysis
    print("PROCESSING SECOND FILE - USERCUTS ANALYSIS")
    print("=" * 70)
    raw_data2 = load_data_from_file(json_file2)
    filtered_data2 = filter_data(raw_data2, [0, 2])  # Only successful instances
    categorized_data2 = categorize(filtered_data2)

    data2_with_sums, sum_values = compute_usercuts_sum(categorized_data2)

    # Compute and print SGM by category for usercuts
    categories2, sgm_values2, values_by_cat2 = compute_sgm_by_category(
        data2_with_sums, "usercuts_sum"
    )
    overall_sgm2 = print_category_table(
        categories2,
        values_by_cat2,
        sgm_values2,
        "usercuts",
        "usercuts_lm + usercuts_sec",
    )

    # Print final summary
    print("\n" + "=" * 70)
    print("FINAL RESULTS SUMMARY")
    print("=" * 70)
    print(f"File 1 ({Path(json_file1).name}) - nconst_acyc:")
    print(f"  Overall SGM:                  {overall_sgm1:.6f}")
    print(f"File 2 ({Path(json_file2).name}) - usercuts sum:")
    print(f"  Overall SGM:                  {overall_sgm2:.6f}")
    print("=" * 70)

    # Print detailed category breakdown
    print("\nDETAILED CATEGORY BREAKDOWN:")
    print("-" * 50)
    print(f"{'Category':<12} {'nconst_acyc':>12} {'usercuts':>12}")
    print("-" * 50)
    for i, cat in enumerate(categories1):
        print(f"{cat:<12} {sgm_values1[i]:>12.3f} {sgm_values2[i]:>12.3f}")
    print("-" * 50)


if __name__ == "__main__":
    main()
