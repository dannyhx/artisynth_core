package artisynth.demos.growth.models.paper;

import artisynth.demos.growth.GrowNode3d;
import artisynth.demos.growth.models.base.GrowDemo;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;

public class _Exp_Leaf extends GrowDemo {
   
   protected double morphogenSrcDuration = 0.5;
   
   protected void build_pre() {
      super.build_pre();
      
      mSizeMin = 0.01;
      mSizeMax = mSizeMin*100;
      mMeshXDiv = 10;
      mMeshYDiv = 10;
      m_shellThickness = 0.01;
      m_youngsModulus = 1e6;   
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
         if (gNode.mIsMorphogenSrc && t0 < morphogenSrcDuration) {
            gNode.mChems.set (3, 1);
         }
      }
      super.advanceCustom (t0, t1, flags);
   }
   
   public boolean isMorphogenSrcNode(int v) {
      Vertex3d vtx = mMesh[0].getVertex (v);
      Point3d pnt = vtx.getPosition ();
      
      return (
         pnt.norm () > 0.5 &&
         pnt.x > -0.001 &&
         pnt.y > -0.001
      );
   }
}
