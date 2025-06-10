#!/bin/bash
# setup-remotes.sh

echo "Adding origin and upstream remotes..."
git remote rm upstream
git remote add upstream https://github.com/ArduPilot/ardupilot.git
git remote rm origin
git remote add origin https://github.com/inertiallabs/ardupilot.git

cd modules/mavlink
git remote rm upstream
git remote add upstream https://github.com/ArduPilot/mavlink.git
git remote rm origin
git remote add origin https://github.com/inertiallabs/mavlink.git

echo "Fetching origin and upstream"
git fetch origin
git fetch upstream
cd ../..
git fetch origin
git fetch upstream

echo "Done."
