# Overview

This document contains instruction for replicating a representive figure from the paper titled "Large Growth Deformations of Thin Tissue using Solid-Shells", authored by Danny Huang and Ian Stavness.

The following instructions have been tested on `Windows 10`. 

## Prerequisite software

```bash
# Install Git (and the included Git Bash)
https://github.com/git-for-windows/git/releases/download/v2.38.1.windows.1/Git-2.38.1-64-bit.exe

# Install Java SDK (19 or latest is fine)
https://download.oracle.com/java/19/latest/jdk-19_windows-x64_bin.msi
```

## Run the script

```bash
# NB: Open the Git Bash console (not the Windows Command Prompt), and
# run the script:

./replicate_representative_image.sh

# Simulation will take a few hours and will automatically stop when the time
# that is displayed in the GUI reaches 25.0.

# After simulation finishes, the resulting model that is 
# displayed in the GUI should match the provided
# representive figure (including alongside with this README.md).
```