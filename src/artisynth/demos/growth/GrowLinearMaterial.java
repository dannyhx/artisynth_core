            // Should point away from edge.
package artisynth.demos.growth;

import artisynth.core.materials.LinearMaterial;

public class GrowLinearMaterial extends LinearMaterial {

   /**
    * LinearMaterial adapted for growth. Currently there's no distinction.
    */
   public GrowLinearMaterial (double E, double nu) {
      super (E, nu, /*corotated=*/true);
   }
   
}
