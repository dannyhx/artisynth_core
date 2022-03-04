package artisynth.demos.growth.models.ts.chen.test;

import java.io.File;
import java.io.IOException;

import artisynth.core.femmodels.FemElement.ElementClass;
import artisynth.demos.growth.models.base.GrowDemo;
import artisynth.demos.growth.models.ts.ThinShellType;
import artisynth.demos.growth.models.ts.chen.DiscreteShell;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Matrix2d;

public class DiscreteShellTest extends GrowDemo {
   
   protected Matrix2d II = new Matrix2d();
   
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
      
      //
      
//      m_shellThickness = 5e-4;
//      II.set(1, 0, 1, 0);
   }
   
   protected void build_modelSkeleton() {
      mMesh = new PolygonalMesh[M];
      for (int m=0; m<M; m++) { 
         mMesh[m] = new PolygonalMesh();
         
         try {
            mMesh[m].read (new File("C:\\Users\\dan\\pj\\libshell\\example\\bunny.obj"));
            mMesh[m] = MeshFactory.createBox (1, 1, 1);
//            mMesh[m] = MeshFactory.createPlane (1, 1, 10, 10);
//            mMesh[0].write (new File("C:\\Users\\dan\\pj\\libshell\\example\\box.obj"));
            
         }
         catch (IOException e) {
            e.printStackTrace();
         }
      }
      
   }
   
   protected void build_post() {
      mMechModel.setDynamicsEnabled (false);
      
//      ((DiscreteShell)mFemModel[0].myThinShellAux.mTS).setI (Matrix2d.IDENTITY);
      ((DiscreteShell)mFemModel[0].myThinShellAux.getThinShellBase ()).setII (this.II);
   }
   
   public void advanceCustom(double t0, double t1, int flags) {
      mFemModel[0].myThinShellAux.advance ();
      super.advanceCustom (t0, t1, flags);
   }
}
