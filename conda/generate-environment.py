#!/usr/bin/env python3

import os
import sys
import json

conda_env = {
    "version": 6,
    "cmakeMinimumRequired": {
        "major": 3,
        "minor": 14,
        "patch": 0,
    },
    "configurePresets": [
        {
            "name": "conda-env",
            "hidden": True,
            "cacheVariables": {},
            "environment": {},
        }
    ]
}

def main():
    conda_env["configurePresets"][0]["environment"] = os.environ.copy()

    # parse out CMAKE_ARGS environment variable as cache variables
    try:
        cmake_args = os.environ["CMAKE_ARGS"]
        for arg in cmake_args.split():
            name, val = arg.split('=')

            # remove leading '-D'
            name = name[2:]

            # add to cacheVariables
            conda_env["configurePresets"][0]["cacheVariables"][name] = {
                "type": "STRING",
                "value": val,
            }
    except:
        pass

    print(json.dumps(conda_env, indent=2, sort_keys=True))

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print('Interrupted')
        try:
            sys.exit(130)
        except SystemExit:
            os._exit(130)
