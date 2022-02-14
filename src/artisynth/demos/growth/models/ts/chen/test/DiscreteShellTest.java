package artisynth.demos.growth.models.ts.chen.test;

import java.io.File;
import java.io.IOException;

import artisynth.core.femmodels.FemElement.ElementClass;
import artisynth.demos.growth.models.base.GrowDemo;
import artisynth.demos.growth.models.ts.ThinShellType;
import maspack.geometry.PolygonalMesh;

public class DiscreteShellTest extends GrowDemo {
   
   protected void build_pre() {
      super.build_pre();

      mEnableDiffusion = false;
      mEnableGrowth = false;
      mEnableRemesh = false;
      mEnablePlasticEmbedding = false; 
      mEnableCollisionHandling = false;
      
      mEleClass = ElementClass.MEMBRANE;
      mTsType = ThinShellType.CHEN;
      
      m_shellThickness = 1e-1;
      m_poissonsRatio = 1.0/2.0;
   }
   
   protected void build_modelSkeleton() {
      mMesh = new PolygonalMesh[M];
      for (int m=0; m<M; m++) { 
         mMesh[m] = new PolygonalMesh();
         
         try {
            mMesh[m].read (new File("C:\\Users\\dan\\pj\\libshell\\example\\bunny.obj"));
         }
         catch (IOException e) {
            e.printStackTrace();
         }
      }
   }
   
   protected void build_post() {
      mMechModel.setDynamicsEnabled (false);
   }
   
   public void advanceCustom(double t0, double t1, int flags) {
      mFemModel[0].myThinShellAux.advance ();
      super.advanceCustom (t0, t1, flags);
   }
}
