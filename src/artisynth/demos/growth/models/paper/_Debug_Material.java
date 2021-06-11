package artisynth.demos.growth.models.paper;

import artisynth.core.materials.BlemkerMuscle;
import artisynth.core.materials.FemMaterial;
import artisynth.core.materials.FungMaterial;
import artisynth.core.materials.MooneyRivlinMaterial;
import artisynth.core.materials.MuscleMaterial;
import artisynth.core.materials.NeoHookeanMaterial;
import artisynth.core.materials.OgdenMaterial;
import artisynth.demos.growth.GrowChemical;
import artisynth.demos.growth.GrowColorer;
import artisynth.demos.growth.GrowLinearMaterial;
import artisynth.demos.growth.GrowModel3d;
import artisynth.demos.growth.GrowNode3d;
import artisynth.demos.growth.GrowRemesher;
import artisynth.demos.growth.Morphogen2GrowthTensor;
import artisynth.demos.growth.PlasticEmbedder;
import artisynth.demos.growth.diffusion.Diffusion;
import artisynth.demos.growth.diffusion.MeshChemicals;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;

public class _Debug_Material extends Intricate_Cylinder {
   
   protected void build_pre() {
      super.build_pre();
      
      m_shellThickness = 0.001; // 0.01
      m_youngsModulus = 1e4;  // 1e7
   }
   
   protected void build_modelProperties() {
      super.build_modelProperties();
      
      FemMaterial mat = null;
      mat = new NeoHookeanMaterial(m_youngsModulus, m_poissonsRatio);  // Corset shape. Very similar to LinearMaterial
      mat = new MooneyRivlinMaterial(1500, 0, 0, 0, 0, 150000);
//      mat = new OgdenMaterial();   // Pill shape 

      
      for (int m=0; m<M; m++) {
        mFemModel[m].setMaterial (mat);
      }
   }
   

   
   
   /** -- Engine Loop -- **/

}
