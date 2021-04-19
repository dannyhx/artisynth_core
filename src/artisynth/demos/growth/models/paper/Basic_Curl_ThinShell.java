package artisynth.demos.growth.models.paper;

import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.demos.growth.GrowModel3d;
import artisynth.demos.growth.GrowNode3d;
import artisynth.demos.growth.GrowTriElement;
import maspack.matrix.Point3d;
import maspack.matrix.VectorNd;

public class Basic_Curl_ThinShell extends Basic_Curl {

   /* --- Helper methods, which can overridden, for building --- */
   
   protected FemModel3d createFemModel() {
      return new GrowModel3d();
   }
   
   protected GrowNode3d createNode(Point3d pt) {
      return new GrowNode3d(pt, new VectorNd(mNumChemTypes));
   }
   
   protected GrowTriElement createElement(FemNode3d n0, FemNode3d n1,
   FemNode3d n2, double thickness)
   {
      return new GrowTriElement(
         (GrowNode3d)n0, 
         (GrowNode3d)n1,
         (GrowNode3d)n2, thickness, true);
   }
}
