# Large Growth Deformations of Thin Tissue using Solid-Shells

This is a fork of [ArtiSynth](https://www.artisynth.org) to enable growth for
solid-shells. The growth simulator was developed for the paper titled
[_Large Growth Deformations of Thin Tissue using Solid-Shells_](https://dannyhx.github.io/ieee_tvcg_2022_9928368.pdf), and has been made open-source to 
ensure reproducibility of the methods and experiments.

Video Demo (110 MB Stream): https://drive.google.com/file/d/1N363GPpE-rwxP25TtFLOj4U_sK0gN8uT/view?usp=sharing

## Table of Contents

1. [Installation (Eclipse IDE)](#installation-eclipse-ide)
1. [Installation (Standalone)](#installation-standalone)
1. [Source Code Layout](#source-code-layout)
1. [Paper Experiments](#paper-experiments)
1. [Coding Settings for Eclipse IDE](#coding-settings-for-eclipse-ide)

## Installation (Eclipse IDE)

The growth simulator can be setup alongside with the Eclipse IDE. This is the recommended method if you plan to get involved with the source code.

```bash
# Install Git (if haven't done so).
https://git-scm.com/downloads

# Install Java SDK (19 or latest is fine)
https://www.oracle.com/ca-en/java/technologies/javase-downloads.html

# Download the Eclipse IDE. 
# When prompted by the installer, choose "Eclipse IDE for Java Developers".
# When installed, close the installer; do not open the application just yet.
https://www.eclipse.org/downloads/

# Create an Eclipse IDE workspace directory:
mkdir eclipse-workspace
cd eclipse-workspace

# Download the ArtiSynth repository.
git clone https://github.com/dannyhx/artisynth_core.git
cd artisynth_core

# Switch to the growth branch.
git checkout growth

# Download the libraries required for ArtiSynth.
bin/updateArtisynthLibs     # Linux/Mac
bin/updateArtisynthLibs.bat # Windows

# Unzip the eclipse settings.
# Accept any overwrite if prompted.
unzip eclipseSettings.zip

# Open Eclipse.
# When prompted for the workspace directory, specify the workspace
# directory that was created earlier.

# Import the Artisynth git repository as follows:
> File (located in top taskbar) 
> Import 
> Git 
> Projects from Git 
> Next 
> Existing local repository 
> Next 
> Add 
> Browse 
> Select the artisynth_core directory

# `artisynth_core` repository should automatically appear under Search results now.
# Proceed with its import:
Tick its checkbox > Add > Next > Next (Import existing Eclipse projects) > Finish

# The Package Explorer (left sidebar) should appear now, 
# with artisynth_core visible.

# Setup the Run configurations.
> Run (located in top taskbar) 
> Run Configurations ... 
> Java Application (located in left side-bar) 
> ArtiSynth 
> Arguments tab 
> Set the Program arguments to

    -model artisynth.demos.growth.models.paper.Intricate_Wrinkles   
    -noTimeline     
    -play                   
    -disableHybridSolves    
    -numSolverThreads 1  

> Set the VM arguments to 

    -Xmx10g                 

> Apply > Close

# Note that -disableHybridSolves and -numSolverThreads 1 are specified in ensure
# reproducibility of the experiments; hybrids solves and multi-threading have a 
# side-effect of generating non-deterministic behavior.

# Run the growth experiment by clicking on the Play button, or by Run > Run.
# Make sure that the artisynth_core directory is selected in the Package Explorer
# beforehand so Eclipse knows which package to run.

# You can switch to different growth experiments by changing the argument to 
# -model. See the "Paper Experiments" section in this README.md for other existing 
# experiments.
```

## Installation (Standalone)

Alternatively, the growth simulator can be setup without any IDE dependency.

```bash
# Install Git (and the included Git Bash if on Windows)
https://git-scm.com/downloads

# Install Java SDK (19 or latest is fine)
https://www.oracle.com/java/technologies/downloads/

# Open the terminal. If on Windows, open the Git Bash console 
# (not the Windows Command Prompt).

git clone https://github.com/dannyhx/artisynth_core.git
cd artisynth_core

git checkout growth 

# Download .jar dependencies.
bin/updateArtisynthLibs

# Compile.
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
```

## Source Code Layout

The source code relating to growth is contained in the `artisynth.demos.growth.`
java package. Within the growth package, the components are organized as follows

```bash
growth                               # Growth Tensor Computation and support for Plastic Embedding.
growth.collision                     # Discrete and Continuous Collision Detection.
growth.diffusion                     # Handles diffusion of morphogen.
growth.remesh                        # Provides remeshing.
growth.util                          # Mesh and math utilities.
growth.models.base.ShellPatch.java   # Sandbox demo for elastic solid-shells.
growth.models.base.GrowthDemo.java   # Subclass of ShellPatch.java to enable growth.
growth.models.ts.evouga              # Java port of Discrete Thin-Shells
growth.models.ts.narain              # Java port of Hinge-based Thin-Shells                
growth.models.paper                  # Subclasses of GrowthDemo.java for specific growth experiments.
```

Before creating new growth experiments, `ShellPatch.java` and `GrowthDemo.java` should be read first to 
get a feel of how things to setup and configured. Afterwards, new growth experiments can be 
created in the `growth.models.paper` package. The paper package itself provides many examples of
custom growth experiments.

Documentation on ArtiSynth itself can be found at https://www.artisynth.org

## Paper Experiments

You can update the java command-line arguments in the Run Configurations to specify the desired experiment to be simulated.

Note that each of these experiments have hard-coded parameters in the `build_pre()` function of their respective source file. For example, in `DualCurl.java`, the `mEleClass` variable can be set to `ElementClass.SHELL` to simulate the Solid-Shell curling.

```bash
# Basic Shapes
-model artisynth.demos.growth.models.paper.Basic_Base
-model artisynth.demos.growth.models.paper.Basic_Boundary
-model artisynth.demos.growth.models.paper.Basic_Strip

# Intricate Wrinkle
-model artisynth.demos.growth.models.paper.Intricate_Wrinkles

# Ripple Cascade
-model artisynth.demos.growth.models.paper.Intricate_RippleCascade

# Fruit-like
-model artisynth.demos.growth.models.paper.Intricate_Fruit

# Curling Sheet
-model artisynth.demos.growth.models.paper.Basic_Curl

# Collision Handling Demo 1 - Teaser figure
-model artisynth.demos.growth.models.paper.Intricate_RippleBouquet

# Collision Handling Demo 2 - Confined growth
-model artisynth.demos.growth.models.paper.Confined_Growth

# Volume, Solid-Shell, and Thin-Shell Cylinder Curling
-model artisynth.demos.growth.models.ts.test.DualCurl
```

## Coding Settings for Eclipse IDE

I recommend using an Eclipse plugin that allows you to cut/copy/paste lines of code
much like in Visual Studio Code.
[https://code.google.com/archive/p/copycutcurrentline/](https://code.google.com/archive/p/copycutcurrentline/)

The CTRL+SHIFT+R shortcut is also very handy, which allows you quick search for a specific filename.

You can enable automatic formatting and automatic import organization:

`Window > Preferences > Java > Editor > Save Actions`
