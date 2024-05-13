#!/bin/bash

# conda devenv --lock -f conda/environment.devenv.yml

for platform in "linux" "osx" "win"
do
    cat conda/.freecad.$platform-64.conda-lock.yml | grep '^  url:' | awk '{print $2}' > conda/freecad.$platform-64.conda-lock.yml
done
