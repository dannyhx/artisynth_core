#!/bin/bash

if ! command -v javac &> /dev/null
then
    echo "javac program could not be found. Please install the Java SDK (19+)."
    exit
fi

if ! command -v javaw &> /dev/null
then
    echo "javaw program could not be found. Please install the Java SDK (19+)."
    exit
fi

echo "Downloading dependenecies..."
bin/updateArtisynthLibs

echo "Compiling..."
mkdir classes 
bin/compile 

# See the "Paper Experiments" section in this README.md for other experiments.
EXPERIMENT="artisynth.demos.growth.models.paper.Intricate_RippleBouquet"

CLASS_PATH="classes;lib\argparser.jar;lib\jass.jar;lib\jython.jar;lib\jmf.jar;lib\quickhull3d.jar;lib\vclipx.jar;lib\jipopt.jar;lib\javaosc.jar;lib\vfs2.jar;lib\gluegen-rt-2.3.2.jar;lib\jogl-all-2.3.2.jar;lib\matconsolectl-4.4.4.jar;lib\jsoup-1.11.2.jar;lib\gdcm.jar"

# Run the experiment. Use `jawaw.exe` instead of `javaw` if on Windows.
javaw \
-Xmx10g \
-Dfile.encoding=UTF-8 \
-classpath $CLASS_PATH \
-XX:+ShowCodeDetailsInExceptionMessages artisynth.core.driver.Launcher \
-model $EXPERIMENT \
-noTimeline \
-play \
-disableHybridSolves \
-numSolverThreads 1 