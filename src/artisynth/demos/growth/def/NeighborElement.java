package artisynth.demos.growth.def;

import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.ShellTriElement;

public class NeighborElement {
   public ShellTriElement ele;
   public FemNode3d[] edgeNodes;
   
   public NeighborElement() {
      this.ele = null; 
      this.edgeNodes = new FemNode3d[2];
   }
}
