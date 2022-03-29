#!/bin/bash
export FIRESIM_ENV_SOURCED=1
export FIRESIM_STANDALONE=1
if [ $# -ne 2 ]; then
    echo "usage: $0 INPUT_FIR OUTPUT_DIR" >&2
    exit 1
fi
mkdir -p $2
make TARGET_PROJECT=midasexamples SBT_COMMAND="runMain midas.stage.GoldenGateMain \
  -i $1 \
  -td $2 \
  -faf ${1%.fir}.anno.json \
  -ggcp firesim.midasexamples \
  -ggcs HostDebugFeatures_DefaultF1Config \
  --output-filename-base FireSim-generated \
  --no-dedup" sbt
