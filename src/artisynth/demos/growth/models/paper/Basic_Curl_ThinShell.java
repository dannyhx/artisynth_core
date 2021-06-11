package artisynth.demos.growth.models.paper;

import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.demos.growth.GrowModel3d;
import artisynth.demos.growth.GrowNode3d;
import artisynth.demos.growth.GrowTriElement;
import maspack.matrix.Point3d;
import maspack.matrix.VectorNd;

public class Basic_Curl_ThinShell extends Basic_Curl {

   protected void build_pre() {
      super.build_pre();
      m_isMembrane = false;
   }
}
