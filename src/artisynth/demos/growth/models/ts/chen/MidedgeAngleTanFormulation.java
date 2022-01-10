package artisynth.demos.growth.models.ts.chen;

import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.ShellElement3d;
import artisynth.demos.growth.util.MeshUtil;
import maspack.geometry.HalfEdge;
import maspack.geometry.Vertex3d;
import maspack.matrix.Matrix3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;

public class MidedgeAngleTanFormulation extends SFF {
   
   /**
    * 
    * @param f
    * @param derivative 3x21
    * @param hessian 21x21
    */
   public static void secondFundamentalFormEntries(
      ShellElement3d ele, MatrixNd derivative, MatrixNd[] hessian) 
   {
      if (derivative != null) {
         derivative.setZero ();
      }
      
      if (hessian != null) {
         for (int i = 0; i < hessian.length; i++) {
            hessian[i].setZero ();
         }
      }
      
      Vector3d II = new Vector3d();
      for (int i = 0; i < 3; i++) {
         MatrixNd hderiv = new MatrixNd(1, 9);
         MatrixNd hhess = new MatrixNd(9, 9);
         double altitude = GeometryDerivative.triangleAltitude(ele, i, hderiv, hhess);
         
         MatrixNd thetaderiv = new MatrixNd(1, 12);
         MatrixNd thetahess = new MatrixNd(12, 12);
         
         
         
      }
     
      
   }
   
   public static void secondFundamentalForm() {
      
   }
   
   /////////////
   
   protected static double edgeTheta(FemModel3d model, HalfEdge edge, MatrixNd derivative, MatrixNd hessian) {
      if (derivative != null) {
         derivative.setZero ();
      }
      
      if (hessian != null) {
         hessian.setZero ();
      }
      
      FemNode3d node0 = model.getNode( edge.head.getIndex () );
      FemNode3d node1 = model.getNode( edge.tail.getIndex () );
      Vertex3d[] oppVtxs = MeshUtil.getOppositeVtxs (edge);
      if (oppVtxs.length == 1) {
         // Boundary edge.
         return 0;
      }
      FemNode3d node2 = model.getNode( oppVtxs[0].getIndex () );
      FemNode3d node3 = model.getNode( oppVtxs[1].getIndex () );
      
      //
      
      Vector3d q0 = node0.getPosition ();
      Vector3d q1 = node1.getPosition ();
      Vector3d q2 = node2.getPosition ();
      Vector3d q3 = node3.getPosition ();
      
      Vector3d q0_q2 = new Vector3d().sub (q0,  q2);
      Vector3d q1_q3 = new Vector3d().sub (q1,  q3);
      Vector3d q1_q2 = new Vector3d().sub (q1,  q2);
      Vector3d q0_q3 = new Vector3d().sub (q0,  q3);
      
      Vector3d n0 = new Vector3d().cross (q0_q2, q1_q2);
      Vector3d n1 = new Vector3d().cross (q1_q3, q0_q3);
      Vector3d axis = new Vector3d().sub (q1, q0);
      
      MatrixNd angderiv = new MatrixNd(1, 9);
      MatrixNd anghess = new MatrixNd(9, 9);
      
      double theta = GeometryDerivative.angle (n0, n1, axis, 
         (derivative != null || hessian != null) ? angderiv : null, 
         (hessian !=null) ? anghess : null);
      
      Vector3d q2_q1 = new Vector3d().sub(q2, q1);
      Vector3d q1_q0 = new Vector3d().sub(q1, q0);
      Vector3d q3_q0 = new Vector3d().sub(q3, q0);
      Vector3d q0_q1 = new Vector3d().sub(q0, q1);
      
      if (derivative != null) {         
         MatrixNd block = new MatrixNd();
         
         MatrixNd angderiv_row = new MatrixNd(1, 3);
         angderiv.getSubMatrix (0, 0, angderiv_row);
         
         // Add blocks
         
         block.mul (angderiv_row, MatrixUtil.crossMatrix (q2_q1));
         MatrixUtil.addBlock (derivative, 0, 0, block);
         
         block.mul (angderiv_row, MatrixUtil.crossMatrix (q0_q2));
         MatrixUtil.addBlock (derivative, 0, 3, block);
         
         block.mul (angderiv_row, MatrixUtil.crossMatrix (q1_q0));
         MatrixUtil.addBlock (derivative, 0, 6, block);
         
         //
         
         block.mul (angderiv_row, MatrixUtil.crossMatrix (q1_q3));
         MatrixUtil.addBlock (derivative, 0, 0, block);
         
         block.mul (angderiv_row, MatrixUtil.crossMatrix (q3_q0));
         MatrixUtil.addBlock (derivative, 0, 3, block);
         
         block.mul (angderiv_row, MatrixUtil.crossMatrix (q0_q1));
         MatrixUtil.addBlock (derivative, 0, 9, block);
      }
      
      if (hessian != null) {
         Matrix3d[] vqm = new Matrix3d[3];
         vqm[0] = MatrixUtil.crossMatrix (q0_q2);
         vqm[1] = MatrixUtil.crossMatrix (q1_q0);
         vqm[2] = MatrixUtil.crossMatrix (q2_q1);
         
         Matrix3d[] wqm = new Matrix3d[3];
         wqm[0] = MatrixUtil.crossMatrix (q0_q1);
         wqm[1] = MatrixUtil.crossMatrix (q1_q3);
         wqm[2] = MatrixUtil.crossMatrix (q3_q0);
         
         int vindices[] = {3, 6, 0};
         int windices[] = {9, 0, 3};
         
         Matrix3d anghess_block_00 = new Matrix3d();
         anghess.getSubMatrix (0, 0, anghess_block_00);
         
         Matrix3d anghess_block_03 = new Matrix3d();
         anghess.getSubMatrix (0, 3, anghess_block_03);
         
         Matrix3d anghess_block_30 = new Matrix3d();
         anghess.getSubMatrix (3, 0, anghess_block_30);

         Matrix3d anghess_block_33 = new Matrix3d();
         anghess.getSubMatrix (3, 3, anghess_block_33);
         
         
         for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
               
            }
         }
        
         
         
         
         
         
         
         
         
      }
   }
}
