package artisynth.demos.growth.models.ts.evouga.test;

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

   protected void build_pre () {
      super.build_pre ();

      mEnableDiffusion = false;
      mEnableGrowth = false;
      mEnableRemesh = false;
      mEnablePlasticEmbedding = false;
      mEnableCollisionHandling = false;

      mEleClass = ElementClass.MEMBRANE;
      mTsType = ThinShellType.EVOUGA;

      m_shellThickness = 5e-2;
      m_youngsModulus = 1.0;

      // Free

      m_shellThickness = 1e-2;
      m_youngsModulus = 1e1;
      mReg = 0.25;

      //

      mReg = 1e-6;
      m_youngsModulus = 1;
      m_shellThickness = 1e-1;
      m_poissonsRatio = 0.5;

      //
   }

   protected void build_modelSkeleton () {
      mMesh = new PolygonalMesh[M];
      for (int m = 0; m < M; m++) {
         // try {
         // mMesh[m].read (new
         // File("C:\\Users\\dan\\pj\\libshell\\example\\bunny.obj"));
         // }
         // catch (IOException e) {
         // e.printStackTrace();
         // }

         // mMesh[m] = MeshFactory.createPlane (1,1,25,25);
         // mMesh[m] = MeshFactory.createBox (1, 1, 1);
         mMesh[m] =
            MeshUtil.createCylinderFromPlane_YAxisCurved (1, 1, 25, 25, 2);
      }
   }

   protected void build_renderConfig () {
      super.build_renderConfig ();

      mRendCfg = mRendCfgPresets.get (RenderMode.DEFAULT);
      mRendCfg.mNodeRadius = 0.0005;
   }

   protected void build_post () {
      mMechModel.setDynamicsEnabled (false);

      mDS = ((DiscreteShell)mFemModel[0].myThinShellAux.getThinShellBase ());
      // mDS.saveState ("C:\\Users\\dan\\pj\\libshell\\example\\cylinder.rest");

      // mDS.loadRestState
      // ("C:\\Users\\dan\\pj\\libshell\\example\\cylinder.rest");

      mDS.mReg = mReg;
   }

   public void advanceCustom (double t0, double t1, int flags) {
      mFemModel[0].myThinShellAux.advance ();
      super.advanceCustom (t0, t1, flags);
   }

   // public static void main(String[] args) {
   // Matrix3x3Block block3x3 = new Matrix3x3Block();
   // block3x3.setIdentity ();
   //
   // MatrixNdBlock blockNd = new MatrixNdBlock(6, 6);
   // blockNd.setIdentity ();
   //
   // SparseBlockMatrix M = new SparseBlockMatrix();
   // M.addBlock (0, 0, block3x3);
   // M.addBlock (1, 1, blockNd);
   //
   // System.out.println (M.toString ("%.2f"));
   // }
}
