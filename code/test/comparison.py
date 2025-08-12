import sys

sys.dont_write_bytecode = True

import json
from pathlib import Path
import numpy as np


def load_data_from_file(json_file: str) -> dict:

    if not Path(json_file).is_file():
        print(f"ERROR: {json_file} is not an existing file.")
        exit(1)

    with open(json_file) as f:
        data = json.load(f)
    if "results" not in data:
        print(f"Cannot find key 'results' in this json file.")
        exit(1)

    return data["results"]


def filter_data(data: dict, status_list: list[int] = (0, 2)) -> dict:
    copy = {}
    for key in data:
        if data[key]["status"] not in status_list:
            continue
        copy[key] = data[key]
        if data[key]["status"] not in (0, 1) or data[key]["time"] > 900:
            copy[key]["time"] = 900

    return copy


def merge(data_list: list[dict]) -> dict:
    merged = {}
    for key in data_list[0]:
        if all(key in data for data in data_list):
            merged[key] = {}
            for i, data in enumerate(data_list):
                for x in data[key]:
                    merged[key][f"{x}_{i+1}"] = data[key][x]
    return merged


def find_min_time(data: dict, num_files: int) -> dict:
    new_data = data

    for key in new_data:
        new_data[key]["min_time"] = min(
            new_data[key][f"time_{i+1}"] for i in range(num_files)
        )

    return new_data


def categorize(data: dict) -> dict:
    new_data = data

    for key in new_data:
        if new_data[key]["min_time"] < 1:
            new_data[key]["diff"] = "[0,1)"
        elif new_data[key]["min_time"] < 10:
            new_data[key]["diff"] = "[1,10)"
        elif new_data[key]["min_time"] < 100:
            new_data[key]["diff"] = "[10,100)"
        elif new_data[key]["min_time"] < 900:
            new_data[key]["diff"] = "[100,900)"
        else:
            new_data[key]["diff"] = "[900+)"

    return new_data


def prepare_data(json_files: list[str], metric: str) -> dict:
    data_list = [load_data_from_file(jf) for jf in json_files]
    if metric not in data_list[0][list(data_list[0].keys())[0]]:
        print(f"{metric} not found in the json files.\nList of the available metrics:")
        print(data_list[0][list(data_list[0].keys())[0]].keys())
        exit(1)
    filtered_data = [filter_data(data) for data in data_list]
    merged = merge(filtered_data)
    merged = find_min_time(merged, len(json_files))
    merged = categorize(merged)
    return merged


def sgm(values: list[float]) -> float:
    return np.exp(np.mean(np.log([x + 1 for x in values]))) - 1


def compute_ratios(data: dict, metric: str) -> tuple[list[str], list[float]]:
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


def print_table(merged: dict, metric: str, num_files: int) -> None:
    categories = ["[0,1)", "[1,10)", "[10,100)", "[100,900)", "[900+)"]
    data = {cat: [[] for _ in range(num_files)] for cat in categories}

    for key in merged:
        for i in range(num_files):
            data[merged[key]["diff"]][i].append(merged[key][f"{metric}_{i+1}"])

    header = f"{'Category':<15} || {'Count':>15} || " + " | ".join(
        [f"{metric + '_' + str(i + 1):>15}" for i in range(num_files)]
    )
    if num_files == 2:
        header += f" | {f'{metric}_ratio':>15}"
    print("=" * len(header))
    print(header)
    print("-" * len(header))

    for key in categories:
        sgms = [sgm(data[key][i]) for i in range(num_files)]
        line = f"{key:<15} || {f'{len(data[key][0])}':>15} || " + " | ".join(
            [f"{s:>15.3f}" for s in sgms]
        )
        if num_files == 2:
            ratio = sgms[1] / sgms[0] if sgms[0] != 0 else 0
            line += f" | {f'{ratio:.3f}':>15}"
        print(line)
    print("=" * len(header))


def main():
    if len(sys.argv) < 4:
        print(
            f"Usage: > python3 {Path(__file__).name} <metric> <json1> <json2> ... <jsonN>"
        )
        exit(1)

    metric = sys.argv[1]
    json_files = sys.argv[2:]
    data = prepare_data(json_files, metric)
    print_table(data, metric, len(json_files))


if __name__ == "__main__":
    main()
