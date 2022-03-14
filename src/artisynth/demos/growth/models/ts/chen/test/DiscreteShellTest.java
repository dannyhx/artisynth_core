package artisynth.demos.growth.models.ts.chen.test;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;

import artisynth.core.femmodels.FemElement.ElementClass;
import artisynth.demos.growth.models.base.GrowDemo;
import artisynth.demos.growth.models.ts.ThinShellType;
import artisynth.demos.growth.models.ts.chen.DiscreteShell;
import artisynth.demos.growth.models.ts.chen.GeometryDerivative;
import artisynth.demos.growth.util.MeshUtil;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Matrix2d;
import maspack.matrix.Matrix3d;
import maspack.matrix.RotationMatrix2d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.Vector3d;

public class DiscreteShellTest extends GrowDemo {
   
   protected Matrix2d II = new Matrix2d();
   protected DiscreteShell mDS = null;
   
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
      
      // Bending
      
      m_shellThickness = 2.5e-3;
      II.set(
         0, 0, 
         0, 0
      );
      
      // Free
      
//      m_shellThickness = 5e-4;
      
      m_shellThickness = 2.5e-3; // 1,1
//      m_shellThickness = 3e-3; // 10,10
      
      m_shellThickness = 5e-2;
      
      /*
      For given flat sheet, bend each column of triangles until form a
      cylinder. THen precalculate the II for each, and use that as a prestrain.
       */
   }
   
   protected void build_modelSkeleton() {
      mMesh = new PolygonalMesh[M];
      for (int m=0; m<M; m++) { 
         mMesh[m] = new PolygonalMesh();
         
         try {
            mMesh[m].read (new File("C:\\Users\\dan\\pj\\libshell\\example\\bunny.obj"));
//            mMesh[m] = MeshFactory.createBox (1, 1, 1);
            mMesh[m] = MeshFactory.createPlane (1,1,10,10);
//            mMesh[m] = MeshUtil.createCylinderFromPlane(1,1,10,10,7.5);
//            mMesh[0].write (new File("C:\\Users\\dan\\pj\\libshell\\example\\plane.obj"));
//            mMesh[m] = MeshFactory.createOpenCylinder (0.5, 0.5, 8, 3);
            
         }
         catch (IOException e) {
            e.printStackTrace();
         }
      }
   }
   
   protected void build_post() {
      mMechModel.setDynamicsEnabled (true);
      
//      ((DiscreteShell)mFemModel[0].myThinShellAux.mTS).setI (Matrix2d.IDENTITY);
      
      mDS = ((DiscreteShell)mFemModel[0].myThinShellAux.getThinShellBase ());
//      mDS.setBarycentricRestII (this.II);
//      mDS.saveState ("C:\\Users\\dan\\pj\\libshell\\example\\cylinder_7.5x.rest");
      
      mDS.loadRestState ("C:\\Users\\dan\\pj\\libshell\\example\\cylinder.rest");
//      mDS.loadRestState ("C:\\Users\\dan\\pj\\libshell\\example\\cylinder_7.5x.rest");
   }
   
   public void advanceCustom(double t0, double t1, int flags) {
      mFemModel[0].myThinShellAux.advance ();
      super.advanceCustom (t0, t1, flags);
      
//      RotationMatrix2d R = new RotationMatrix2d();
//      R.setAngle (Math.PI * 0.25);
//      
      for (int i : new int[] {0,1} ) {
//         Matrix2d I = mDS.getI (i);
//         Matrix2d II = mDS.getII (i);
//         System.out.printf ("Element %d: \n", i);
//         System.out.print (I);
//         System.out.print (II);
//         System.out.println (Arrays.toString (mMesh[0].getFace (i).getVertexIndices ()));
         
//         Vector3d n = mDS.getFaceNormal (i, 0);
//         System.out.println (n);
      }
      
      
   }
   
//   public static void main(String[] args) {
//      Matrix2d II = new Matrix2d(
//         -0.3802644557588878, -0.3802644557588878, 
//         -0.3802644557588878, -0.3802644557588878 
//      ); 
//      
//      RotationMatrix2d R = new RotationMatrix2d();
//      R.setAngle (Math.PI / 2);
//      
//      II.mul (R);
//      System.out.println (II);
//   }
}
