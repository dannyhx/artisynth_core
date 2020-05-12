package artisynth.demos.growth.models.paper;

import java.awt.Color;

import artisynth.demos.growth.GrowNode3d;
import maspack.matrix.Point3d;

public class _Debug_Diffusion_Base extends Basic_Base {

   double size = -1;
   protected void build_pre() {
      super.build_pre();
      
      size = 1;
      mMeshX = size;
      mMeshY = size;
      
      int div = 32;
      mMeshXDiv = div;
      mMeshYDiv = div;
      
      mEnablePlasticEmbedding = false;
      mEnableRemesh = false;
     
      mDiffusionTimestepScale = 0.05;
      mChemCvtRate = 0;// 0.001
      
      mSurfaceColor = SurfaceColor.MORPHOGEN;
   }
   
   protected boolean mIsFirstTimestepPending = true;
   
   public void advanceCustom(double t0, double t1, int flags) {
      // Background and camera
      if (mIsFirstTimestepPending) {
         getMainViewer ().setBackgroundColor (Color.WHITE);
         
         getMainViewer().setEye (new Point3d(0, 0, 3.67504));
         getMainViewer().setCenter (new Point3d(0, 0, 0));
         
         mIsFirstTimestepPending = false;
         
         System.out.println ("Number of elements: " + mMesh[0].numFaces ());
         advanceCustom_applyMorphogen (t0);
      }

      mDiffusion.advance ((t1-t0)*mDiffusionTimestepScale);
      
      // Color update
//      mGrowColorer.computeMorphogenColors ();
      this.updateNodesColor ();
      mFemModel[0].getRenderProps ().setLineColor (Color.BLACK);
   }
   
   public boolean isMorphogenSrcNode(int v) {
      return true;
   }
   
   public void advanceCustom_applyMorphogen(double t0) {
      for (int v = 0; v < mMesh[0].numVertices (); v++) {
         if ( isMorphogenSrcNode(v)) {
            GrowNode3d gNode = (GrowNode3d)mFemModel[0].getNode (v);
            Point3d pnt = gNode.getPosition ();
            
            double conc = pnt.x/size+0.5;
            gNode.mChems.set (3, conc);
         }
      }
   }
   
   
}
