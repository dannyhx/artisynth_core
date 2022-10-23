# Large Growth Deformations of Thin Tissue using Solid-Shells

This is a fork of [ArtiSynth](https://www.artisynth.org) to enable growth for
solid-shells. The growth simulator was developed for the paper titled
_Large Growth Deformations of Thin Tissue using Solid-Shells_, and has been made open-source to 
ensure reproducibility of the methods and experiments.

## Installation

The growth simulator can be setup by downloading the code repository and 
compiling the code using the Eclipse IDE. The steps below can be followed 
to setup and run the growth simulator.

```bash
# Download and install Git (if haven't done so).
https://git-scm.com/downloads

# Download and install the Java development kit (JDK).
# We use 19.0.1, but the latest JDK will be sufficient.
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
# -model.
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

Documentation on ArtiSynth itself can be found at https://www.artisynth.org/Software/Documentation.

## Paper Experiments

You can update the java command-line arguments in the Run Configurations to specify the desired experiment to be simulated.

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

## Coding Settings

I recommend using an Eclipse plugin that allows you to cut/copy/paste lines of code
much like in Visual Studio Code.
[https://code.google.com/archive/p/copycutcurrentline/](https://code.google.com/archive/p/copycutcurrentline/)

The CTRL+SHIFT+R shortcut is also very handy, which allows you quick search for a specific filename.

You can enable automatic formatting and automatic import organization:
`Window > Preferences > Java > Editor > Save Actions`