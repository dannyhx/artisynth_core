package artisynth.demos.growth.models.paper;

import artisynth.demos.growth.models.base.GrowDemo;

public class _Debug_Growth extends GrowDemo {

   protected void build_modelSkeleton() {
      super.build_modelSkeleton ();
   }
   
   protected void build_pre() {
      super.build_pre();
      
      m_shellThickness = 0.1/(1e2);
      m_youngsModulus = 1e4;
      m_density = 100;

      mMeshX = 1;
      mMeshY = 1;
      mMeshXDiv = 4;
      mMeshYDiv = 4;
   }
   
   protected void build_post() {
      mIsShowRefMesh = false;
      mEnableDiffusion = false;
      mEnablePlasticEmbedding = false;
      mEnableRemesh = false;
      mEnableCollisionHandling = true;
      
      super.build_post ();
   }
   
   public void advanceCustom(double t0, double t1, int flags) {
      super.advanceCustom (t0, t1, flags);
   }

}
