# LG

This is a fork of [ArtiSynth](https://www.artisynth.org) to enable growth for
solid-shells. The growth simulator was developed for the thesis titled
_LG_, and has been made open-source to ensure reproducibility of the methods and 
experiments.

## Installation

The growth simulator can be setup by downloading the code repository and 
compiling the code using the Eclipse IDE. The steps below can be followed 
to setup and run the growth simulator.

```bash
# Download and install Git (if haven't done so).
https://git-scm.com/downloads

# Download and install the Java development kit (JDK).
https://www.oracle.com/ca-en/java/technologies/javase-downloads.html

# Download and install the "Eclipse IDE for Java Developers".
# When installed, do not open the application just yet.
https://www.eclipse.org/downloads/

# Create an Eclipse IDE workspace directory:
mkdir eclipse-workspace
cd eclipse-workspace

# Download the ArtiSynth repository.
git clone https://github.com/dannyhx/artisynth_core.git
cd artisynth_core

# Switch to the shell_growth branch.
git checkout shell_growth

# Download the libraries required for ArtiSynth.
cd bin
./updateArtisynthLibs     # Linux/Mac
./updateArtisynthLibs.bat # Windows

# Open Eclipse.
# When prompted for the workspace directory, specify the workspace
# directory that was created earlier.

# Restore the UI view by clicking on the "dual window" symbol located 
# in the top-left corner. This should automatically open the 
# Java Perspective view. 

# Import the Artisynth git repository.
File (located in top taskbar) > Import > Git > Projects from Git > Next >

Existing local repository > Next >

Add... > Set the Directory to your eclipse workspace. The
`artisynth_core` repository should automatically appear
in the search results afterwards; tick its checkbox > Finish >

Next > Next (Import existing Eclipse projects) > Finish

# The Package Explorer (left sidebar) should appear now, 
# with artisynth_core visible.

# Setup the Run configurations.
Run (located in top taskbar) > Run Configurations ... > Java Application > 
ArtiSynth > Arguments tab > Set the Program arguments to

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
growth.models.paper                  # Subclasses of GrowthDemo.java for specific growth experiments.
```

Before creating new growth experiments, `ShellPatch.java` and `GrowthDemo.java` should be read first to 
get a feel of how things to setup and configured. Afterwards, new growth experiments can be 
created in the `growth.models.paper` package. The paper package itself provides many examples of
custom growth experiments.

Documentation on ArtiSynth itself can be found at https://www.artisynth.org/Software/Documentation.