package artisynth.demos.growth.models.ts.chen;

import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.ShellElement3d;
import maspack.geometry.Face;
import maspack.matrix.Matrix2d;
import maspack.matrix.Matrix3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;

/**
 * GeometryDerivatives.cpp 
 */
public class GeometryDerivative {
   
   /**
    * Calculate the first fundamental form for the given face.
    * 
    * Verified.
    * 
    * @param derivative M(4,9)
    * @param hessian M(9,9)[4]
    * @return
    */
   public static Matrix2d firstFundamentalForm(
      MeshConnectivity MC, 
      FemModel3d model,
      int f,
      MatrixNd derivative, 
      MatrixNd[] hessian
   ) {
      // Nodal positions.
      Vector3d[] q = new Vector3d[3];
      for (int i = 0; i < 3; i++) {
         q[i] = model.getNode (MC.F[f][i]).getPosition ();
      }
      
      Vector3d q1_q0 = new Vector3d().sub (q[1], q[0]);
      Vector3d q2_q0 = new Vector3d().sub (q[2], q[0]);
      
      Matrix2d result = new Matrix2d(
         q1_q0.dot (q1_q0), q1_q0.dot (q2_q0), 
         q2_q0.dot (q1_q0), q2_q0.dot (q2_q0)
      );
      
      if (derivative != null) {
         derivative.setZero ();
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
      }
      
      if (hessian != null) {
         if (hessian.length != 4) {
            throw new RuntimeException("Unexpected length");
         }
         
         for (int i = 0; i < 4; i++) {
            hessian[i] = new MatrixNd(9, 9);
         }
         
         MatrixNd I = new MatrixNd(Matrix3d.IDENTITY);
         MatrixNd I2 = new MatrixNd(I);
         I2.scale (2);
         MatrixNd negI = new MatrixNd(I);
         negI.scale (-1);
         MatrixNd negI2 = new MatrixNd(I);
         negI2.scale (-2);

         hessian[0].addSubMatrix (0, 0, I2);
         hessian[0].addSubMatrix (3, 3, I2);
         hessian[0].addSubMatrix (0, 3, negI2);
         hessian[0].addSubMatrix (3, 0, negI2);
         
         hessian[1].addSubMatrix (3, 6, I);
         hessian[1].addSubMatrix (6, 3, I);
         hessian[1].addSubMatrix (0, 3, negI);
         hessian[1].addSubMatrix (0, 6, negI);
         hessian[1].addSubMatrix (3, 0, negI);
         hessian[1].addSubMatrix (6, 0, negI);
         hessian[1].addSubMatrix (0, 0, I2);
         
         hessian[2].addSubMatrix (3, 6, I);
         hessian[2].addSubMatrix (6, 3, I);
         hessian[2].addSubMatrix (0, 3, negI);
         hessian[2].addSubMatrix (0, 6, negI);
         hessian[2].addSubMatrix (3, 0, negI);
         hessian[2].addSubMatrix (6, 0, negI);
         hessian[2].addSubMatrix (0, 0, I2);
         
         hessian[3].addSubMatrix (0, 0, I2);
         hessian[3].addSubMatrix (6, 6, I2);
         hessian[3].addSubMatrix (0, 6, negI2);
         hessian[3].addSubMatrix (6, 0, negI2);
      }
      
      return result;
   }
   
   /**
    * Calculate the triangle altitude.
    * 
    * Verified2.
    * 
    * @param ele
    * @param edgeIdx 
    * One of the three edges of the given element. 0,1,2. 
    * i.e. local edgeIdx relative to element.
    * 
    * @param derivative 1x9
    * @param hessian 9x9
    * @return
    */
   public static double triangleAltitude(
      MeshConnectivity MC, FemModel3d model, Face face, int edgeIdx, MatrixNd derivative, 
      MatrixNd hessian) 
   {
      if (derivative != null) {
         derivative.setZero ();
      }
      
      if (hessian != null) {
         hessian.setZero ();
      }
      
      // Face normal.
      
//      if (edgeIdx == 2) {
//         System.out.println ("here");
//      }
      
      MatrixNd nderiv = new MatrixNd(3, 9);
      MatrixNd[] nhess = new MatrixNd[3];
      for (int i = 0; i < 3; i++) {
         nhess[i] = new MatrixNd(9, 9);
      }
      Vector3d n = faceNormal(MC, model, face, edgeIdx, nderiv, nhess);
      
      int v1 = (edgeIdx + 1) % 3;
      int v2 = (edgeIdx + 2) % 3;
      
      Vector3d q1 = model.getNode (MC.F[face.idx][v1]).getPosition ();
      Vector3d q2 = model.getNode (MC.F[face.idx][v2]).getPosition ();
      
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
         
         MatrixNd eT = new MatrixNd(1,3);
         eT.setRow (0, e);  
         derivative.addScaledSubMatrix (0, 6, -nnorm / enorm / enorm / enorm, eT);
         derivative.addScaledSubMatrix (0, 3, nnorm / enorm / enorm / enorm, eT);
      }
      
//      System.out.println (nderiv.toString ("%.5f"));
//      System.out.println (nhess[0].toString ("%.5f"));
//      System.out.println (nhess[1].toString ("%.5f"));
//      System.out.println (nhess[2].toString ("%.5f"));
//      System.out.println (n);
//      System.out.println (v1);
//      System.out.println (v2);
//      System.out.println (q1);
//      System.out.println (q2);
//      System.out.println (e);
//      System.out.println (nnorm);
//      System.out.println (enorm);
      
      if (hessian != null) {
         for (int i = 0; i < 3; i++) {
            hessian.scaledAdd (n.get (i) / nnorm / enorm, nhess[i]);
         }
         
         // P
         
         MatrixNd P = new MatrixNd(3,3);
         P.setIdentity ();
         P.scale (1/nnorm );
         
         Matrix3d _n_nT = new Matrix3d();
         _n_nT.outerProduct(n, n);   // n.nT is the outer product of n and n.
         _n_nT.scale (1/nnorm/nnorm/nnorm);
         
         P.sub (new MatrixNd(_n_nT));
         
         //
         
         MatrixNd hessian_operand = new MatrixNd(9, 9);
         hessian_operand.mulTransposeLeft (nderiv, P);
         hessian_operand.mul (nderiv);
         hessian.scaledAdd (1/enorm, hessian_operand);
         
//         System.out.println (hessian.toString ("%.5f"));
         
         // Hessian blocks
         
         Matrix3d e_nT = new Matrix3d();
         e_nT.outerProduct (e, n);
 
         MatrixNd block = new MatrixNd(3,9);
         block.mul (e_nT, nderiv);
         hessian.addScaledSubMatrix(6, 0, -1*nnorm/enorm/enorm/enorm, block);
//         System.out.println (hessian.toString ("%.5f"));
         hessian.addScaledSubMatrix(3, 0, +1/nnorm/enorm/enorm/enorm, block);
//         System.out.println (hessian.toString ("%.5f"));
         
         Matrix3d n_eT = new Matrix3d();
         n_eT.outerProduct (n, e);   // n.eT is the outer product of n and e.
         MatrixNd nderivT = new MatrixNd(9, 3);
         nderivT.transpose(nderiv);
         block.mul (nderivT, n_eT);  // 3.9
         
         System.out.println (n_eT.toString ("%.5f"));
         System.out.println (block.toString ("%.5f"));
         nderivT.scale (-1);
         block.mul (nderivT, n_eT);  // 3.9
         System.out.println (block.toString ("%.5f"));
         
         
         
         hessian.addScaledSubMatrix(0, 6, -1.0 / nnorm / enorm / enorm / enorm, block);
//         System.out.println (hessian.toString ("%.5f"));
         hessian.addScaledSubMatrix(0, 3, +1.0 / nnorm / enorm / enorm / enorm, block);
//         System.out.println (hessian.toString ("%.5f"));
         
         block.set (Matrix3d.IDENTITY);
         hessian.addScaledSubMatrix (6, 6, -1.0 * nnorm / enorm / enorm / enorm, block);
//         System.out.println (hessian.toString ("%.5f"));
         hessian.addScaledSubMatrix (6, 3, +1.0 * nnorm / enorm / enorm / enorm, block);
         hessian.addScaledSubMatrix (3, 6, +1.0 * nnorm / enorm / enorm / enorm, block);
         hessian.addScaledSubMatrix (3, 3, -1.0 * nnorm / enorm / enorm / enorm, block);
         
//         System.out.println (hessian.toString ("%.5f"));
         
         
         // outer
         
         Matrix3d outer = new Matrix3d();
         outer.outerProduct (e, e);
         outer.scale (3.0 * nnorm / enorm / enorm / enorm / enorm / enorm);
         MatrixNd outerNd = new MatrixNd(outer);
         
         hessian.addScaledSubMatrix (6, 6, 1.0, outerNd);
         hessian.addScaledSubMatrix (6, 3, -1.0, outerNd);
         hessian.addScaledSubMatrix (3, 6, -1.0, outerNd);
         hessian.addScaledSubMatrix (3, 3, 1.0, outerNd);
         
//         System.out.println (hessian.toString ("%.5f"));
//         System.out.println ("here");
         
      }
      
      return h;
   }
   
   /**
    * Calculate the normal for the given face.
    * 
    * @param ele
    * @param derivative
    * @param hessian
    * @return
    */
   public static Vector3d faceNormal(
      MeshConnectivity MC, 
      FemModel3d model, 
      Face face,
      int edgeIdx,
      MatrixNd derivative,   // 3x9
      MatrixNd[] hessian     // 9x9
   ) {
      if (derivative != null) {
         derivative.setZero ();
      }
      
      if (hessian != null) {
         for (int i = 0; i < 3; i++) {
            hessian[i].setZero ();
         }
      }
      
      int v0 = edgeIdx % 3;
      int v1 = (edgeIdx + 1) % 3;
      int v2 = (edgeIdx + 2) % 3;
      
      Point3d qi0 = model.getNode (MC.F[face.idx][v0]).getPosition ();
      Point3d qi1 = model.getNode (MC.F[face.idx][v1]).getPosition ();
      Point3d qi2 = model.getNode (MC.F[face.idx][v2]).getPosition ();

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
          0,  -v.z, v.y,
          v.z, 0,  -v.x, 
         -v.y, v.x, 0
      );
   }
   
   /**
    * Compute the angle between two 3D vectors.
    * 
    * Verified
    * 
    * @param v
    * @param w
    * @param axis 
    * Rotational axis shared by v and w. (e.g. edge shared by two faces)
    * 
    * @param derivative  // 1x9
    * @param hessian     // 9x9
    * @return
    */
   public static double angle(
      Vector3d v, 
      Vector3d w, 
      Vector3d axis,
      MatrixNd derivative, 
      MatrixNd hessian
   ) {
      Vector3d vw = new Vector3d().cross (v, w);
      
      double theta = 2.0 * Math.atan2 ((vw.dot (axis) / axis.norm()), v.dot (w) + v.norm() * w.norm());
   
      Vector3d axisV = new Vector3d().cross (axis, v);
      Vector3d axisW = new Vector3d().cross (axis, w);
      
      if (derivative != null) {
         MatrixNd axisV_1x3 = new MatrixNd(1, 3);
         axisV_1x3.setRow (0, axisV);
         
         MatrixNd axisW_1x3 = new MatrixNd(1, 3);
         axisW_1x3.setRow (0, axisW);
         
         derivative.addScaledSubMatrix (0, 0, -1.0 / v.normSquared () / axis.norm (), axisV_1x3);
         derivative.addScaledSubMatrix (0, 3, +1.0 / w.normSquared () / axis.norm (), axisW_1x3);
         
         MatrixNd zero1x3 = new MatrixNd(1,3);
         derivative.setSubMatrix (0, 6, zero1x3);
      }
      
      if (hessian != null) {
         hessian.setZero ();

         MatrixNd block = new MatrixNd();
         Matrix3d op = new Matrix3d();  // Outer Product
         
         op.outerProduct (axisV, v);
         block.set (op);
         hessian.addScaledSubMatrix(0, 0, 
            2.0 / v.normSquared () / v.normSquared () / axis.norm (), block);
         
         op.outerProduct (axisW, w);
         block.set (op);
         hessian.addScaledSubMatrix(3, 3, 
            -2.0 / w.normSquared() / w.normSquared() / axis.norm (), block);
         
         block = MatrixUtil.crossMatrixNd (axis);
         hessian.addScaledSubMatrix(0, 0, 
            -1.0 / v.normSquared () / axis.norm (), block);
         
         hessian.addScaledSubMatrix(3, 3, 
            1.0 / w.normSquared () / axis.norm (), block);
         
         // dahat
         
         Matrix3d dahat = new Matrix3d();
         dahat.scaledAdd (1.0 / axis.norm (), Matrix3d.IDENTITY);
         dahat.addScaledOuterProduct(-1.0 / axis.norm () / axis.norm () / axis.norm (), axis, axis);
         
         // Last 2 blocks
         
         block.mul (MatrixUtil.crossMatrix (v), dahat);
         hessian.addScaledSubMatrix(0, 6, 
            1.0 / v.normSquared (), block);
         
         block.mul (MatrixUtil.crossMatrix (w), dahat);
         hessian.addScaledSubMatrix(3, 6, 
            -1.0 / w.normSquared (), block);
      }
      
      return theta;
   }
}
