package artisynth.demos.growth;

import artisynth.core.femmodels.FemElement3dBase;
import artisynth.core.femmodels.StiffnessWarper3d;

public class GrowStiffnessWarper3d extends StiffnessWarper3d {

   public GrowStiffnessWarper3d (FemElement3dBase elem) {
      super (elem);
   }

   protected GrowLinearMaterialCache getOrCreateLinearCache (
      FemElement3dBase e) {
      if (linear == null) {
         linear = new GrowLinearMaterialCache (e);
      }
      return (GrowLinearMaterialCache)linear;
   }

   protected GrowLinearMaterialCache getOrCreateCorotatedCache (
      FemElement3dBase e) {
      if (corotated == null) {
         corotated = new GrowLinearMaterialCache (e);
      }
      return (GrowLinearMaterialCache)corotated;
   }
}
