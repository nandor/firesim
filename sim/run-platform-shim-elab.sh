#!/bin/bash
set -e

# Generate `.classpath` file with the output of `export runtime:fullClasspath`
# executed from the sbt shell.
BASEDIR=$(cd $(dirname $BASH_SOURCE[0]) && pwd)
CLASSPATH_FILE=$BASEDIR/.classpath

if [ ! -f "$CLASSPATH_FILE" ]; then
  java -Xmx16G -Dsbt.supershell=false -jar \
    $BASEDIR/../target-design/chipyard/generators/rocket-chip/sbt-launch.jar \
    --error \
    "export runtime:fullClasspath" > $CLASSPATH_FILE
fi

CLASSPATH=`cat $CLASSPATH_FILE`

TMPFIR=`mktemp /tmp/firesim-XXXXXXXX.fir`
trap 'rm -- "$TMPFIR"' EXIT
TMPANNOS=`mktemp /tmp/firesim-XXXXXXXX.anno.json`
trap 'rm -- "$TMPANNOS"' EXIT

java -classpath $CLASSPATH midas.stage.PlatformShimElabMain -i "$1" -o "$TMPFIR" "$TMPANNOS" >&2
firtool "$TMPFIR" --annotation-file "$TMPANNOS" --ir-hw -o "$2"
