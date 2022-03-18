package artisynth.demos.growth.models.ts.evouga.test;

import java.io.File;
import java.io.IOException;

import artisynth.core.femmodels.FemElement.ElementClass;
import artisynth.demos.growth.models.base.GrowDemo;
import artisynth.demos.growth.models.ts.ThinShellType;
import artisynth.demos.growth.models.ts.evouga.DiscreteShell;
import artisynth.demos.growth.util.MeshUtil;
import maspack.geometry.PolygonalMesh;

// -model artisynth.demos.growth.models.ts.evouga.test.DiscreteShellTest

public class DiscreteShellTest extends GrowDemo {
   
   /** Convenient reference to the Thin-Shell model */ 
   protected DiscreteShell mDS = null;
   
   protected double mReg = 0.25;
   
   protected void build_pre() {
      super.build_pre();

      mEnableDiffusion = false;
      mEnableGrowth = false;
      mEnableRemesh = false;
      mEnablePlasticEmbedding = false; 
      mEnableCollisionHandling = false;
      
      mEleClass = ElementClass.MEMBRANE;
      mTsType = ThinShellType.EVOUGA;
      
      m_shellThickness = 1e-1;
      m_poissonsRatio = 1.0/2.0;
      
      // Free
      
      m_shellThickness = 5e-2;
      m_youngsModulus = 1.0;
   }
   
   protected void build_modelSkeleton() {
      mMesh = new PolygonalMesh[M];
      for (int m=0; m<M; m++) { 
         mMesh[m] = new PolygonalMesh();
         
         try {
            mMesh[m].read (new File("C:\\Users\\dan\\pj\\libshell\\example\\bunny.obj"));
//            mMesh[m] = MeshFactory.createPlane (1,1,10,10);
            mMesh[m] = MeshUtil.createCylinderFromPlane(1,1,25,25,1);
//            mMesh[0].write (new File("C:\\Users\\dan\\pj\\libshell\\example\\plane.obj"));
         }
         catch (IOException e) {
            e.printStackTrace();
         }
      }
   }
   
   protected void build_post() {
      mMechModel.setDynamicsEnabled (true);
      
      mDS = ((DiscreteShell)mFemModel[0].myThinShellAux.getThinShellBase ());
      mDS.saveState ("C:\\Users\\dan\\pj\\libshell\\example\\cylinder_7.5x.rest");
      
//      mDS.loadRestState ("C:\\Users\\dan\\pj\\libshell\\example\\cylinder.rest");
      mDS.loadRestState ("C:\\Users\\dan\\pj\\libshell\\example\\cylinder_7.5x.rest");
      
      mDS.mReg = mReg;
   }
   
   public void advanceCustom(double t0, double t1, int flags) {
      mFemModel[0].myThinShellAux.advance ();
      super.advanceCustom (t0, t1, flags);
   }
}
