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


def merge(data1: dict, data2: dict) -> dict:
    merged = {}
    for key in data1:
        if key not in data2:
            continue
        merged[key] = {}
        for x in data1[key]:
            merged[key][f"{x}_1"] = data1[key][x]
        for x in data2[key]:
            merged[key][f"{x}_2"] = data2[key][x]

    return merged


def find_min_time(data: dict) -> dict:
    new_data = data

    for key in new_data:
        new_data[key]["min_time"] = min(
            (new_data[key]["time_1"], new_data[key]["time_2"])
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


def prepare_data(json1: str, json2: str, metric: str) -> dict:
    data1 = load_data_from_file(json1)
    data2 = load_data_from_file(json2)
    if metric not in data1[list(data1.keys())[0]]:
        print(f"{metric} not found in the json files.\nList of the available metrics:")
        print(data1[list(data1.keys())[0]].keys())
        exit(1)
    data1_filtered = filter_data(data1)
    data2_filtered = filter_data(data2)
    merged = merge(data1_filtered, data2_filtered)
    merged = find_min_time(merged)
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


def print_table(merged: dict, metric: str) -> None:
    data1 = {"[0,1)": [], "[1,10)": [], "[10,100)": [], "[100,900)": [], "[900+)": []}
    data2 = {"[0,1)": [], "[1,10)": [], "[10,100)": [], "[100,900)": [], "[900+)": []}

    for key in merged:
        data1[merged[key]["diff"]].append(merged[key][f"{metric}_1"])
        data2[merged[key]["diff"]].append(merged[key][f"{metric}_2"])

    print("=" * 89)
    print(
        f"{'Category':<15} || {'Count':>15} || {f'{metric}_1':>15} | {f'{metric}_2':>15} | {f'{metric}_ratio':>15}"
    )
    print("-" * 89)
    for key in data1:
        list1 = data1[key]
        list2 = data2[key]
        sgm1 = sgm(list1)
        sgm2 = sgm(list2)
        print(
            f"{key:<15} || {f'{len(list1)}':>15} || {f'{sgm1:.3f}':>15} | {f'{sgm2:.3f}':>15} | {f'{(sgm2 / sgm1):.3f}':>15}"
        )
    print("=" * 89)


def main():
    if len(sys.argv) != 4:
        print(f"Usage: > python3 {Path(__file__).name} <metric> <json1> <json2>")
        exit(1)

    metric = sys.argv[1]
    json1 = sys.argv[2]
    json2 = sys.argv[3]
    data = prepare_data(json1, json2, metric)
    print_table(data, metric)


if __name__ == "__main__":
    main()
