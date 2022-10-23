#!/bin/bash 

# Please run this script while in the repo's root directory 
# (i.e. not in this demo_script directory).

javaw.exe \
-Xmx10g \
-Dfile.encoding=UTF-8 \
-classpath "classes;lib\argparser.jar;lib\jass.jar;lib\jython.jar;lib\jmf.jar;lib\quickhull3d.jar;lib\vclipx.jar;lib\jipopt.jar;lib\javaosc.jar;lib\vfs2.jar;lib\gluegen-rt-2.3.2.jar;lib\jogl-all-2.3.2.jar;lib\matconsolectl-4.4.4.jar;lib\jsoup-1.11.2.jar;lib\gdcm.jar" \
-XX:+ShowCodeDetailsInExceptionMessages artisynth.core.driver.Launcher \
-model artisynth.demos.growth.models.paper.Intricate_Wrinkles \
-noTimeline \
-play \
-disableHybridSolves \
-numSolverThreads 1 