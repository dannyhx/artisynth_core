package artisynth.demos.growth.models.ts.chen;

import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.ShellElement3d;
import maspack.matrix.Matrix2d;
import maspack.matrix.Matrix3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;

public class GeometryDerivative {
   
   public static class FirstFundamentalFormRv {
      MatrixNd Derivative;
      MatrixNd[] Hessian;
      Matrix2d Result;
   }
   
   public static FirstFundamentalFormRv firstFundamentalForm(ShellElement3d ele) {
      FemNode3d[] nodes = ele.getNodes ();
      
      // Nodal positions.
      Vector3d[] q = new Vector3d[3];
      for (int i = 0; i < 3; i++) {
         q[i] = nodes[i].getPosition ();
      }
      
      Vector3d q1_q0 = new Vector3d().sub (q[1], q[0]);
      Vector3d q2_q0 = new Vector3d().sub (q[2], q[0]);
      
      Matrix2d result = new Matrix2d(
         q1_q0.dot (q1_q0), q1_q0.dot (q2_q0), 
         q2_q0.dot (q1_q0), q2_q0.dot (q2_q0)
      );
      
      MatrixNd derivative = new MatrixNd(4, 9);
      MatrixUtil.add1x3Block (derivative, 0, 3, new Vector3d(q1_q0).scale (2));
      MatrixUtil.add1x3Block (derivative, 0, 0, new Vector3d(q1_q0).scale (-2));
      MatrixUtil.add1x3Block (derivative, 1, 6, new Vector3d(q1_q0));
      MatrixUtil.add1x3Block (derivative, 1, 3, new Vector3d(q2_q0));
      MatrixUtil.add1x3Block (derivative, 1, 0, new Vector3d(q1_q0).add (q2_q0).scale (-1));
      MatrixUtil.add1x3Block (derivative, 2, 6, new Vector3d(q1_q0));
      MatrixUtil.add1x3Block (derivative, 2, 3, new Vector3d(q2_q0));
      MatrixUtil.add1x3Block (derivative, 2, 0, new Vector3d(q1_q0).add(q2_q0).scale (-1));
      MatrixUtil.add1x3Block (derivative, 3, 6, new Vector3d(q2_q0).scale (2));
      MatrixUtil.add1x3Block (derivative, 3, 0, new Vector3d(q2_q0).scale (-2));
      
      MatrixNd[] hessian = new MatrixNd[4];
      for (int i = 0; i < 4; i++) {
         hessian[i] = new MatrixNd(9, 9);
      }
      
      Matrix3d I = Matrix3d.IDENTITY;
      Matrix3d I2 = new Matrix3d();
      I2.scale (2, I);
      Matrix3d negI = new Matrix3d(I);
      negI.scale (-1);
      Matrix3d negI2 = new Matrix3d(I);
      negI2.scale (-2);

      MatrixUtil.add3x3Block (hessian[0], 0, 0, I2);
      MatrixUtil.add3x3Block (hessian[0], 3, 3, I2);
      MatrixUtil.add3x3Block (hessian[0], 0, 3, negI2);
      MatrixUtil.add3x3Block (hessian[0], 3, 0, negI2);
      
      MatrixUtil.add3x3Block (hessian[1], 3, 6, I);
      MatrixUtil.add3x3Block (hessian[1], 6, 3, I);
      MatrixUtil.add3x3Block (hessian[1], 0, 3, negI);
      MatrixUtil.add3x3Block (hessian[1], 0, 6, negI);
      MatrixUtil.add3x3Block (hessian[1], 3, 0, negI);
      MatrixUtil.add3x3Block (hessian[1], 6, 0, negI);
      MatrixUtil.add3x3Block (hessian[1], 0, 0, I2);
      
      MatrixUtil.add3x3Block (hessian[2], 3, 6, I);
      MatrixUtil.add3x3Block (hessian[2], 6, 3, I);
      MatrixUtil.add3x3Block (hessian[2], 0, 3, negI);
      MatrixUtil.add3x3Block (hessian[2], 0, 6, negI);
      MatrixUtil.add3x3Block (hessian[2], 3, 0, negI);
      MatrixUtil.add3x3Block (hessian[2], 6, 0, negI);
      MatrixUtil.add3x3Block (hessian[2], 0, 0, I2);
      
      MatrixUtil.add3x3Block (hessian[3], 0, 0, I2);
      MatrixUtil.add3x3Block (hessian[3], 6, 6, I2);
      MatrixUtil.add3x3Block (hessian[3], 0, 6, negI2);
      MatrixUtil.add3x3Block (hessian[3], 6, 0, negI2);
      
      FirstFundamentalFormRv rv = new FirstFundamentalFormRv();
      rv.Derivative = derivative;
      rv.Hessian = hessian;
      rv.Result = result; 
      
      return rv;
   }
   
   /**
    * Calculate the triangle altitude.
    * 
    * @param ele
    * @param edgeIdx
    * @param derivative 1x9
    * @param hessian 9x9
    * @return
    */
   public static double triangleAltitude(
      ShellElement3d ele, int edgeIdx, MatrixNd derivative, MatrixNd hessian) 
   {
      if (derivative != null) {
         derivative.setZero ();
      }
      
      if (hessian != null) {
         hessian.setZero ();
      }
      
      // Face normal.
      
      MatrixNd nderiv = new MatrixNd(3, 9);
      MatrixNd[] nhess = new MatrixNd[3];
      for (int i = 0; i < 3; i++) {
         nhess[i] = new MatrixNd(9, 9);
      }
      Vector3d n = faceNormal(ele, nderiv, nhess);
      
      int v1 = (edgeIdx + 1) % 3;
      int v2 = (edgeIdx + 2) % 3;
      
      FemNode3d[] nodes = ele.getNodes ();
      Vector3d q1 = nodes[v1].getPosition ();
      Vector3d q2 = nodes[v2].getPosition ();
      
      Vector3d e = new Vector3d().sub (q2, q1);
      
      double nnorm = n.norm ();
      double enorm = e.norm ();
      double h = nnorm / enorm;
      
      if (derivative != null) {
         for (int i = 0; i < 3; i++) {
            // Get nderiv row.
            MatrixNd nderiv_irow = new MatrixNd(1, 9); 
            nderiv.getSubMatrix (i, 0, nderiv_irow);
            
            // Apply scaling and add to derivative.
            derivative.scaledAdd (n.get (i) / nnorm / enorm, nderiv_irow);
         }
      }
      
      if (hessian != null) {
         for (int i = 0; i < 3; i++) {
            hessian.scaledAdd (n.get (i) / nnorm / enorm, nhess[i]);
         }
         
         // P
         
         MatrixNd P = new MatrixNd(3,3);
         P.setIdentity ();
         P.scale (1/nnorm );
         
         Matrix3d _n_nT = new Matrix3d();
         _n_nT.outerProduct(n, n);
         _n_nT.scale (1/nnorm/nnorm/nnorm);
         
         P.sub (new MatrixNd(_n_nT));
         
         //
         
         MatrixNd nT_P = new MatrixNd(9, 9);
         nT_P.mulTransposeLeft (nderiv, P);
         hessian.add (nT_P);
         
         // Hessian blocks.
         
         Matrix3d _e_nT = new Matrix3d();
         _e_nT.outerProduct (e, n);
         _e_nT.scale (-1.0);
         MatrixNd block = new MatrixNd(3,9);
         block.mul (_e_nT, nderiv);
         block.scale (1/nnorm/enorm/enorm/enorm);
         MatrixUtil.addBlock (hessian, 6, 0, block);
         
         block.scale (-1.0);
         MatrixUtil.addBlock (hessian, 3, 0, block);
         
         Matrix3d _n_eT = new Matrix3d();
         _n_eT.mul (n, e);
         MatrixNd _nderivT = new MatrixNd(9, 3);
         nderiv.transpose(_nderivT);
         block.mul (_nderivT, _n_eT);
         block.scale (-1.0 / nnorm / enorm / enorm/ enorm);
         MatrixUtil.addBlock (hessian, 0, 6, block);

         block.scale (-1.0);
         MatrixUtil.addBlock (hessian, 0, 3, block);
         
         block.set (Matrix3d.IDENTITY);
         block.scale (-1.0 * nnorm / enorm / enorm / enorm);
         MatrixUtil.addBlock(hessian, 6, 6, block);
         
         block.scale (-1.0);
         MatrixUtil.addBlock(hessian, 6, 3, block);
         
         MatrixUtil.addBlock(hessian, 3, 6, block);
         
         block.scale (-1.0);
         MatrixUtil.addBlock(hessian, 3, 3, block);
         
         Matrix3d outer = new Matrix3d();
         outer.outerProduct (e, e);
         outer.scale (3.0 / nnorm / enorm / enorm / enorm / enorm / enorm);
         
         MatrixUtil.addBlock(hessian, 6, 6, block);

         block.scale (-1.0);
         MatrixUtil.addBlock(hessian, 6, 3, block);
         
         MatrixUtil.addBlock(hessian, 3, 6, block);
         
         block.scale (-1.0);
         MatrixUtil.addBlock(hessian, 3, 3, block);
      }
      
      return h;
   }
   
   public static Vector3d faceNormal(ShellElement3d ele, MatrixNd derivative, MatrixNd[] hessian) {
      if (derivative != null) {
         derivative.setZero ();
      }
      
      if (hessian != null) {
         hessian = new MatrixNd[3];
         for (int i = 0; i < 3; i++) {
            hessian[i].setZero ();
         }
      }
      
      FemNode3d[] nodes = ele.getNodes ();
      
      Point3d qi0 = nodes[0].getPosition ();
      Point3d qi1 = nodes[1].getPosition ();
      Point3d qi2 = nodes[2].getPosition ();

      Vector3d qi1_qi0 = new Vector3d().sub (qi1, qi0);
      Vector3d qi2_qi0 = new Vector3d().sub (qi2, qi0);
      Vector3d n = new Vector3d().cross (qi1_qi0, qi2_qi0);
      
      if (derivative != null) {
         Vector3d qi2_qi1 = new Vector3d().sub (qi2, qi1);
         derivative.addSubMatrix (0, 0, new MatrixNd(crossMatrix3d(qi2_qi1)));
         
         Vector3d qi0_qi2 = new Vector3d().sub (qi0, qi2);
         derivative.addSubMatrix (0, 3, new MatrixNd(crossMatrix3d(qi0_qi2)));
         
         derivative.addSubMatrix (0, 6, new MatrixNd(crossMatrix3d(qi1_qi0)));
      }
      
      if (hessian != null) {
         for (int j = 0; j < 3; j++) {
            Vector3d ej = new Vector3d();
            ej.set (j, 1.0);
            
            MatrixNd ejc = new MatrixNd(crossMatrix3d(ej));
            
            hessian[j].addScaledSubMatrix (0, 3, -1, ejc);
            hessian[j].addScaledSubMatrix (0, 6, +1, ejc);
            hessian[j].addScaledSubMatrix (3, 6, -1, ejc);
            hessian[j].addScaledSubMatrix (3, 0, +1, ejc);
            hessian[j].addScaledSubMatrix (6, 0, -1, ejc);
            hessian[j].addScaledSubMatrix (6, 3, +1, ejc);
         }
      }
      
      return n;
   }
   
   public static Matrix3d crossMatrix3d(Vector3d v) {
      return new Matrix3d(
         0, -v.z, v.y,
         v.z, 0, -v.x, 
         -v.y, v.x, 0
      );
   }
   
   public static double angle(Vector3d v, Vector3d w, Vector3d axis, MatrixNd derivative, MatrixNd hessian) {
      Vector3d vw = new Vector3d().cross (v, w);
      
      double theta = 2.0 * Math.atan2 ((vw.dot (axis) / axis.norm()), v.dot (w) + v.norm() * w.norm());
   
      if (derivative != null) {
         Vector3d axisV = new Vector3d().cross (axis, v);
         
         MatrixNd seg = new MatrixNd(1,3);
         seg.set (axisV);
         seg.scale (-1.0 / v.normSquared () / axis.norm ());
         MatrixUtil.addBlock (derivative, 0, 0, seg);
         
         seg.scale (-1.0);
         MatrixUtil.addBlock (derivative, 0, 3, seg);
         
         seg.setZero ();
         derivative.setSubMatrix (0, 6, seg);
      }
      
      if (hessian != null) {
         hessian.setZero ();
         
         Vector3d axisV = new Vector3d().cross (axis, v);
         Vector3d axisW = new Vector3d().cross (axis, w);
         
         Matrix3d block = new Matrix3d();
         block.outerProduct (axisV, v);
         block.scale (2.0 / v.normSquared () / v.normSquared () / axis.norm ());
         MatrixUtil.add3x3Block (hessian, 0, 0, block);
         
         block.outerProduct (axisW, w);
         block.scale (-2.0 / w.normSquared() / w.normSquared() / axis.norm ());
         MatrixUtil.add3x3Block (hessian, 3, 3, block);
         
         block = MatrixUtil.crossMatrix (axis);
         block.scale (-1.0 / v.normSquared () / axis.norm ());
         MatrixUtil.add3x3Block (hessian, 0, 0, block);
         
         block = MatrixUtil.crossMatrix (axis);
         block.scale (1 / w.normSquared () / axis.norm ());
         MatrixUtil.add3x3Block (hessian, 3, 3, block);
         
         // dahat
         
         Matrix3d dahat = new Matrix3d();
         dahat.scaledAdd (1.0 / axis.norm (), Matrix3d.IDENTITY);
         dahat.addScaledOuterProduct(-1.0 / axis.norm () / axis.norm () / axis.norm (), axis, axis);
         
         // Last 2 blocks
         
         block.mul (MatrixUtil.crossMatrix (v), dahat);
         block.scale (1.0 / v.normSquared ());
         MatrixUtil.add3x3Block (hessian, 0, 6, block);
         
         block.mul (MatrixUtil.crossMatrix (w), dahat);
         block.scale (-1.0 / w.normSquared ());
         MatrixUtil.add3x3Block (hessian, 3, 6, block);
      }
      
      return theta;
   }
}
