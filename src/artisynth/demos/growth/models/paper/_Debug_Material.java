package artisynth.demos.growth.models.paper;

import artisynth.core.materials.FemMaterial;
import artisynth.core.materials.MooneyRivlinMaterial;
import artisynth.core.materials.NeoHookeanMaterial;

public class _Debug_Material extends Intricate_Cylinder {

   protected void build_pre () {
      super.build_pre ();

      m_shellThickness = 0.001; // 0.01
      m_youngsModulus = 1e4; // 1e7
   }

   protected void build_modelProperties () {
      super.build_modelProperties ();

      FemMaterial mat = null;
      mat = new NeoHookeanMaterial (m_youngsModulus, m_poissonsRatio); // Corset
                                                                       // shape.
                                                                       // Very
                                                                       // similar
                                                                       // to
                                                                       // LinearMaterial
      mat = new MooneyRivlinMaterial (1500, 0, 0, 0, 0, 150000);
      // mat = new OgdenMaterial(); // Pill shape

      for (int m = 0; m < M; m++) {
         mFemModel[m].setMaterial (mat);
      }
   }

   /** -- Engine Loop -- **/

}
