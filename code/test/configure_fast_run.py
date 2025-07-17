# ONLY FOR CLUSTER USE
import sys

sys.dont_write_bytecode = True

import os, random, json


def main():
    # verify input correctness
    if len(sys.argv) <= 1 or sys.argv[1] in ["-h", "--h", "-help", "--help"]:
        print(f"Usage: >> python3 {os.path.basename(__file__)} <instances_folder>.")
        exit(0)

    # read inputs
    instances_folder = os.path.abspath(sys.argv[1])

    # verify inputs
    if (
        input(
            f"The instances will be taken from the folder {instances_folder}\nInsert y if it's correct: "
        )
        != "y"
    ):
        exit(0)

    # select 1000 random instances
    instances_list = os.listdir(instances_folder)
    random.shuffle(instances_list)
    selected_instances = instances_list[:1000]
    selected_instances = sorted(selected_instances)

    # Create instances configuration file
    instances_config = {
        "instances_folder": instances_folder,
        "selected_instances": selected_instances,
    }

    code_dir = os.path.abspath("..")

    config_path = os.path.join(code_dir, "test/fast_run_config.json")
    os.makedirs(os.path.dirname(config_path), exist_ok=True)

    with open(config_path, "w") as f:
        json.dump(instances_config, f, indent=4)

    print(
        f"Selected {len(selected_instances)} instances out of {len(instances_list)} total instances"
    )
    print(f"Configuration saved to: {config_path}")
    print("You can now use fast_create.py to create the jobs")


if __name__ == "__main__":
    main()
