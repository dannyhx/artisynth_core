package artisynth.demos.growth.models.paper;

import artisynth.demos.growth.GrowNode3d;
import artisynth.demos.growth.models.base.GrowDemo;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;

/**
 * Baseline: 4.6s
 * Remeshing: 12s
 * Plastic-Embedding: 6s
 * Remeshing + Plastic-Embedding: 12s 
 * 
 * Top-down material mesh:
 *     0.000757376 -0.022066 1.58585
 *     0.000757376 -0.022066 -0.452017
 */
public class Abalation_RippleCascade extends GrowDemo {
   
   protected double morphogenSrcDuration = 999;
   
   protected void build_pre() {
      super.build_pre();
      
      mMeshX = 2;
      mMeshY = 1;
      
      mSizeMin = 0.01;
      mSizeMax = mSizeMin*100;
      mMeshXDiv = 16;
      mMeshYDiv = 8;
      m_shellThickness = 0.001;
      m_youngsModulus = 1e6;  
    
      mRenderMode = RenderMode.MORPHOLOGY;
      
      mPauseEveryInterval = 4.5;
    
      mEnableRemesh = false;
      mEnablePlasticEmbedding = false;
      mDiffusionTimestepScale = 0.00001;
   }
   
   protected void build_post() {
      super.build_post ();
      
      for (int v = 0; v < mMesh[0].numVertices (); v++) {
         if ( isMorphogenSrcNode(v) ) {
            GrowNode3d gNode = (GrowNode3d)mFemModel[0].getNode (v);
            gNode.mIsMorphogenSrc = true;
         }
      }
   }
   
   public void advanceCustom(double t0, double t1, int flags) { 
      for (int v = 0; v < mMesh[0].numVertices (); v++) {
         GrowNode3d gNode = (GrowNode3d)mFemModel[0].getNode (v);
         if (gNode.mIsMorphogenSrc) {
            gNode.mChems.set (3, 1);
         }
      }
      
      super.advanceCustom (t0, t1, flags);
   }
   
   public boolean isMorphogenSrcNode(int v) {
      Vertex3d vtx = mMesh[0].getVertex (v);
      Point3d pnt = vtx.pnt;
      
      return (pnt.y < -0.499);
   }
}
