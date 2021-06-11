package artisynth.demos.growth.models.paper;

import artisynth.core.materials.FemMaterial;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.materials.MooneyRivlinMaterial;
import artisynth.core.materials.NeoHookeanMaterial;
import artisynth.demos.growth.GrowChemical;
import artisynth.demos.growth.GrowColorer;
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

public class _Debug_ThinShell extends Basic_Base {
   
   protected void build_pre() {
      super.build_pre();
      
      m_isMembrane = true;
      
      mEnableDiffusion = false;
      mEnableGrowth = false; 
      mEnableRemesh = false; 
      mEnablePlasticEmbedding = false; 
      
      mEnableCollisionHandling = false;
      
      //
      
      mMeshX = 1;
      mMeshY = 1;
      
      mMeshXDiv = 1;
      mMeshYDiv = 1;
      
//       mMeshX = 5;
//       mMeshY = 1;
//       
//       mMeshXDiv = 25;
//       mMeshYDiv = 5;
      
      //
      
      mRenderMode = RenderMode.DEFAULT;
      mSurfaceColor = SurfaceColor.DEFAULT;
      
      //
      
      m_shellThickness = 0.001;
      m_youngsModulus = 1e4;
      mSizeMin = 0.05;
      mSizeMax = mSizeMin*5;
      mDiffusionTimestepScale = 0.01;
      mPauseEveryInterval = 999.00; 
      
   }
   
   protected void build_modelProperties() {
      super.build_modelProperties();
      
      FemMaterial mat = null;
//      mat = new LinearMaterial();
      mat = new NeoHookeanMaterial(m_youngsModulus, m_poissonsRatio);  // Corset shape. Very similar to LinearMaterial
//      mat = new MooneyRivlinMaterial(1500, 0, 0, 0, 0, 150000);
//      mat = new OgdenMaterial();   // Pill shape 

      
      for (int m=0; m<M; m++) {
        mFemModel[m].setMaterial (mat);
      }
   }
   
   
   protected void build_renderConfig() {
      super.build_renderConfig ();
      
      mRendCfg = mRendCfgPresets.get (RenderMode.DEFAULT);
      mRendCfg.mNodeRadius = 0.05;
   }
   
   protected void build_utils() {
      super.build_utils ();

      mMorphogen2GrowthTensor.setTarget ((GrowModel3d)mFemModel[0], mMesh[0]);
   }
   
   protected void build_post() {
      super.build_post ();
      mMorphogen2GrowthTensor.isBendingMorphogenHack = true;
   }
   
   public boolean isMorphogenSrcNode(int v) {
//      Vertex3d vtx = mMesh[0].getVertex (v);
//      Point3d pnt = vtx.getPosition ();

      return true;
   }
   
   /**
    * Create a mesh, which will be used as a template for building 
    * the FEM model.
    */
   public void advanceCustom_applyMorphogen(double t0) {
      for (int v = 0; v < mMesh[0].numVertices (); v++) {
         if ( isMorphogenSrcNode(v) && t0 < morphogenSrcDuration) {
            GrowNode3d gNode = (GrowNode3d)mFemModel[0].getNode (v);
            gNode.mChems.set (GrowChemical.PAR.mIdx, 20);
         }
      }
   }
   
   
   /** -- Engine Loop -- **/

}
