package artisynth.demos.growth.models.paper;

import artisynth.core.materials.FemMaterial;
import artisynth.core.materials.NeoHookeanMaterial;
import artisynth.demos.growth.GrowChemical;
import artisynth.demos.growth.GrowModel3d;
import artisynth.demos.growth.GrowNode3d;
import maspack.geometry.PolygonalMesh;

public class _Debug_ThinShell extends Basic_Base {
   
   protected void build_pre() {
      super.build_pre();
      
      m_isMembrane = true;
      
      mEnableDiffusion = true;
      mEnableGrowth = true; 
      mEnableRemesh = true; 
      mEnablePlasticEmbedding = false; 
      
      mEnableCollisionHandling = false;
      
      // If half the area, then bending is more volatile.
      
//      mMeshX = 12.5;
//      mMeshY = 2.5;
//      mMeshX = 25;
//      mMeshY = 6;
//         
//      mMeshXDiv = 25;
//      mMeshYDiv = 6;
      
//       mMeshX = 25;
//       mMeshY = 5;
//       
//       mMeshXDiv = 50;
//       mMeshYDiv = 10;
      
      mMeshX = 1;
      mMeshY = 1;
       
//      mMeshX = .1;
//      mMeshY = .1;
      
      mMeshXDiv = 1;
      mMeshYDiv = 1;
      
      morphogenSrcDuration = 0.01;
      
      //
      
      mRenderMode = RenderMode.DEFAULT;
      mSurfaceColor = SurfaceColor.DEFAULT;
      
      //
      
      m_shellThickness = 0.001;
      m_youngsModulus = 1e3;
      mSizeMin = 0.05;
      mSizeMax = mSizeMin*5;
      mDiffusionTimestepScale = 0.01;
      mPauseEveryInterval = 999.00; 
      
//      ShellPatch.m_particleDamping = 10;
      
   }
   
//   protected void build_modelSkeleton() {
//      mMesh = new PolygonalMesh[M];
//      PolygonalMesh m = new PolygonalMesh();
//      
//      m.addVertex (0, 0, 0); 
//      m.addVertex (1, 0, 0); 
//      m.addVertex (1, 1, 0);
//      m.addFace (0, 1, 2);
//      
//      m.scale (0.1);
//      
//      mMesh[0] = m;
//   }
   
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
            gNode.mChems.set (GrowChemical.PAR.mIdx, 0.1);
            
//            gNode.setVelocity (0, 0, 1);
//            break;
         }
      }
   }
   
   
   /** -- Engine Loop -- **/

   
   public void advanceCustom(double t0, double t1, int flags) {
      super.advanceCustom (t0, t1, flags);
//      if (t0 == 0.00) {
//         PointList<FemNode3d> nodes = this.mFemModel[0].getNodes ();
//         nodes.get (0).setRestPosition (new Point3d(0,0,0));
//         nodes.get (1).setRestPosition (new Point3d(-1,0,0));
//         nodes.get (2).setRestPosition (new Point3d(-1,1,0));
//         System.out.println ("Switched");
//      }
   }
}
