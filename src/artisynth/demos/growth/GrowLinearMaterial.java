package artisynth.demos.growth;

import artisynth.core.materials.LinearMaterial;

public class GrowLinearMaterial extends LinearMaterial {

   public GrowLinearMaterial (double E, double nu) {
      super (E, nu, /*corotated=*/true);
   }
   
}
