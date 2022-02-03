package artisynth.demos.growth.models.ts.chen;

import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.ShellElement3d;
import artisynth.demos.growth.util.MeshUtil;
import maspack.geometry.Face;
import maspack.geometry.HalfEdge;
import maspack.geometry.Vertex3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Vector3d;

import static java.lang.Math.cos;
import static java.lang.Math.tan;

public class MidedgeAngleTanFormulation {
   
   /**
    * 
    * @param f
    * @param derivative 3x21
    * @param hessian 21x21
    */
   public static Vector3d secondFundamentalFormEntries(
      FemModel3d model, ShellElement3d ele, Face face, MatrixNd derivative, MatrixNd[] hessian) 
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
         
         HalfEdge edge = face.getEdge (i);
         MatrixNd thetaderiv = new MatrixNd(1, 12);
         MatrixNd thetahess = new MatrixNd(12, 12);
         double theta = edgeTheta(model, edge, thetaderiv, thetahess);
         
         // DANTODO: Needs review, including edgeThetas.
         double orient = (MeshUtil.isHalfEdgeWithMinHeadIdx (edge)) ? 1.0 : -1.0;
         double alpha = 0.5 * theta;  // + orient * edgeThetas[edge]
         II.set (i, 2.0 * altitude * tan(alpha));
         
         if (derivative != null) {
            int hv0 = i;
            int hv1 = (i + 1) % 3;
            int hv2 = (i + 2) % 3;
            
            MatrixNd block = new MatrixNd(1,3);
            hderiv.getSubMatrix (0, 0, block);
            derivative.addScaledSubMatrix (i, 3 * hv0, 2.0 * tan(alpha), block);
            
            hderiv.getSubMatrix (0, 3, block);
            derivative.addScaledSubMatrix (i, 3 * hv1, 2.0 * tan(alpha), block);
            
            hderiv.getSubMatrix (0, 6, block);
            block.scale (2.0 * tan(alpha));
            derivative.addScaledSubMatrix (i, 3 * hv2, 2.0 * tan(alpha), block);
            
            int av0 = 0;
            int av1 = 0;
            int av2 = 0;
            int av3 = 0;
            
            if (MeshUtil.isHalfEdgeWithMinHeadIdx (edge)) {
               av0 = (i + 1) % 3;
               av1 = (i + 2) % 3;
               av2 = i;
               av3 = 3 + i;
            } else {
              av0 = (i + 2) % 3;
              av1 = (i + 1) % 3;
              av2 = 3 + i;
              av3 = i;
            }
            
            thetaderiv.getSubMatrix (0, 0, block);
            derivative.addScaledSubMatrix (i, 3 * av0, altitude / cos(alpha) / cos(alpha), block);
            
            thetaderiv.getSubMatrix (0, 3, block);
            derivative.addScaledSubMatrix (i, 3 * av1, altitude / cos(alpha) / cos(alpha), block);
            
            thetaderiv.getSubMatrix (0, 6, block);
            derivative.addScaledSubMatrix (i, 3 * av2, altitude / cos(alpha) / cos(alpha), block);
            
            thetaderiv.getSubMatrix (0, 9, block);
            derivative.addScaledSubMatrix (i, 3 * av3, altitude / cos(alpha) / cos(alpha), block);
            
            derivative.add (i, 18+i, 2.0 * altitude / cos(alpha) / cos(alpha) * orient);
         }
         
         if (hessian != null) {
            int hv[] = new int[3];
            hv[0] = i;
            hv[1] = (i + 1) % 3;
            hv[2] = (i + 2) % 3;

            MatrixNd block = new MatrixNd(3,3);
            
            for (int j = 0; j < 3; j++) {
               for (int k = 0; k < 3; k++) {
                  hhess.getSubMatrix (3*j, 3*k, block);
                  hessian[i].addScaledSubMatrix (3 * hv[j], 3 * hv[k], 2.0 * tan(alpha), block);
               }
            }
            
            int av[] = new int[4];
            if (MeshUtil.isHalfEdgeWithMinHeadIdx (edge)) {
               av[0] = (i + 1) % 3;
               av[1] = (i + 2) % 3;
               av[2] = i;
               av[3] = 3 + i;
            } else {
                av[0] = (i + 2) % 3;
                av[1] = (i + 1) % 3;
                av[2] = 3 + i;
                av[3] = i;
            }
            
            MatrixNd thetaderiv_block = new MatrixNd(1,3);
            MatrixNd hderiv_block = new MatrixNd(1,3);
            
            for (int k = 0; k < 3; k++) {
               hderiv.getSubMatrix (0, 3*k, hderiv_block);
               
                for (int j = 0; j < 4; j++) {
                   thetaderiv.getSubMatrix (0, 3 * j, thetaderiv_block);

                   block.mulTransposeLeft (thetaderiv_block, hderiv_block);
                   hessian[i].addScaledSubMatrix (3 * av[j], 3 * hv[k], 1.0 / cos(alpha) / cos(alpha), block);
                   
                   block.mulTransposeLeft (hderiv_block, thetaderiv_block);
                   hessian[i].addScaledSubMatrix (3 * hv[k], 3 * av[j], 1.0 / cos(alpha) / cos(alpha), block);
                }
                
                hessian[i].addScaledSubMatrix (18 + i, 3 * hv[k], 2.0 / cos(alpha) / cos(alpha) * orient, hderiv_block);
                
                block = new MatrixNd(hderiv_block);
                block.transpose ();
                hessian[i].addScaledSubMatrix (3 * hv[k], 18 + i, 2.0 / cos(alpha) / cos(alpha) * orient, block);
            }
            
            MatrixNd thetahess_block = new MatrixNd(3,3);
            
            for (int k = 0; k < 4; k++) {
                thetaderiv.getSubMatrix (0, 3*k, hderiv_block);
               
                for (int j = 0; j < 4; j++) {
                   thetahess.getSubMatrix (3*j, 3*k, thetahess_block);
                   hessian[i].addScaledSubMatrix (3 * av[j], 3 * av[k], altitude / cos(alpha) / cos(alpha), thetahess_block);
                   
                   thetaderiv_block.setSize (1, 3);
                   thetaderiv.getSubMatrix (0, 3*j, thetaderiv_block);
                   thetaderiv_block.transpose ();
                   hessian[i].addScaledSubMatrix (3 * av[j], 3 * av[k], altitude * tan(alpha) / cos(alpha) / cos(alpha), thetaderiv);
                }
                
                thetaderiv_block.setSize (1, 3);
                thetaderiv.getSubMatrix (0, 3 * k, thetaderiv_block);
                hessian[i].addScaledSubMatrix (18 + i, 3 * av[k], 
                   2.0 * altitude * tan(alpha) / cos(alpha) / cos(alpha) * orient, thetaderiv_block);
                
                thetaderiv_block.transpose ();
                hessian[i].addScaledSubMatrix (3 + av[k], 18 * i, 
                   2.0 * altitude * tan(alpha) / cos(alpha) / cos(alpha) * orient, thetaderiv_block);
            }
            
            hessian[i].add (18 + i, 18 + i, 4.0 * altitude * tan(alpha) / cos(alpha) / cos(alpha));
         }
      }
     
      return II;
   }
   
   public static MatrixNd secondFundamentalForm(
      FemModel3d model, 
      ShellElement3d ele, 
      Face face, 
      MatrixNd derivative, 
      MatrixNd[] hessian
   ) {
      if (derivative != null) {
         derivative.setZero ();
      }
      
      if (hessian != null) {
         if (hessian.length != 4) {
            throw new RuntimeException("Unexpected length");
         }
         
         for (int i = 0; i < hessian.length; i++) {
            hessian[i].setZero ();
         }
      }
      
      MatrixNd IIderiv = new MatrixNd(3, 21);
      MatrixNd IIhess[] = new MatrixNd[3];
      
      Vector3d II = secondFundamentalFormEntries(
         model, ele, face, 
         derivative != null ? IIderiv : null, hessian != null ? IIhess : null);
      
      MatrixNd result = new MatrixNd(new double[][] {
         {II.x + II.y}, {II.x},
         {II.x},        {II.x + II.z}
      });
      
      if (derivative != null) {
         MatrixNd IIderiv_row0 = new MatrixNd(1, 21);
         MatrixNd IIderiv_row1 = new MatrixNd(1, 21);
         MatrixNd IIderiv_row2 = new MatrixNd(1, 21);
         
         IIderiv.getSubMatrix (0, 0, IIderiv_row0);     
         IIderiv.getSubMatrix (1, 0, IIderiv_row1);     
         IIderiv.getSubMatrix (2, 0, IIderiv_row2);     
         
         derivative.addSubMatrix (0, 0, IIderiv_row0);   
         derivative.addSubMatrix (0, 0, IIderiv_row1);  
         
         derivative.addSubMatrix (1, 0, IIderiv_row0);   
         derivative.addSubMatrix (2, 0, IIderiv_row0);
         
         derivative.addSubMatrix (3, 0, IIderiv_row0);   
         derivative.addSubMatrix (3, 0, IIderiv_row2);
      }
      
      if (hessian != null) {
         hessian[0].add(IIhess[0]);
         hessian[0].add(IIhess[1]);
         
         hessian[1].add(IIhess[0]);
         hessian[2].add(IIhess[0]);
         
         hessian[3].add(IIhess[0]);
         hessian[3].add(IIhess[2]);
      }
      
      return result;
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
         
         block.mul (angderiv_row, MatrixUtil.crossMatrixNd (q2_q1));
         derivative.addSubMatrix (0, 0, block);
         
         block.mul (angderiv_row, MatrixUtil.crossMatrixNd (q0_q2));
         derivative.addSubMatrix (0, 3, block);
         
         block.mul (angderiv_row, MatrixUtil.crossMatrixNd (q1_q0));
         derivative.addSubMatrix (0, 6, block);
         
         //
         
         block.mul (angderiv_row, MatrixUtil.crossMatrixNd (q1_q3));
         derivative.addSubMatrix (0, 0, block);
         
         block.mul (angderiv_row, MatrixUtil.crossMatrixNd (q3_q0));
         derivative.addSubMatrix (0, 3, block);
         
         block.mul (angderiv_row, MatrixUtil.crossMatrixNd (q0_q1));
         derivative.addSubMatrix (0, 9, block);
      }
      
      if (hessian != null) {
         MatrixNd[] vqm = new MatrixNd[3];
         vqm[0] = MatrixUtil.crossMatrixNd (q0_q2);
         vqm[1] = MatrixUtil.crossMatrixNd (q1_q0);
         vqm[2] = MatrixUtil.crossMatrixNd (q2_q1);
         
         MatrixNd[] wqm = new MatrixNd[3];
         wqm[0] = MatrixUtil.crossMatrixNd (q0_q1);
         wqm[1] = MatrixUtil.crossMatrixNd (q1_q3);
         wqm[2] = MatrixUtil.crossMatrixNd (q3_q0);
         
         int vindices[] = {3, 6, 0};
         int windices[] = {9, 0, 3};
         
         MatrixNd anghess_block_00 = new MatrixNd(3,3);
         anghess.getSubMatrix (0, 0, anghess_block_00);
         
         MatrixNd anghess_block_03 = new MatrixNd(3,3);
         anghess.getSubMatrix (0, 3, anghess_block_03);
         
         MatrixNd anghess_block_30 = new MatrixNd(3,3);
         anghess.getSubMatrix (3, 0, anghess_block_30);

         MatrixNd anghess_block_33 = new MatrixNd(3,3);
         anghess.getSubMatrix (3, 3, anghess_block_33);
         
         //
         
         MatrixNd anghess_block_06 = new MatrixNd(3,3);
         anghess.getSubMatrix (0, 6, anghess_block_06);
         
         MatrixNd anghess_block_60 = new MatrixNd(3,3);
         anghess.getSubMatrix (6, 0, anghess_block_60);
         
         MatrixNd anghess_block_36 = new MatrixNd(3,3);
         anghess.getSubMatrix (3, 6, anghess_block_36);
         
         MatrixNd anghess_block_63 = new MatrixNd(3,3);
         anghess.getSubMatrix (6, 3, anghess_block_63);
         
         //
         
         MatrixNd block = new MatrixNd(3,3);
         
         for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
              block.mulTransposeLeft (vqm[i], anghess_block_00);
              block.mul (vqm[j]);
              hessian.addSubMatrix(vindices[i], vindices[j], block);
              
              block.mulTransposeLeft (vqm[i], anghess_block_03);
              block.mul (wqm[j]);
              hessian.addSubMatrix(vindices[i], windices[j], block);
              
              block.mulTransposeLeft (wqm[i], anghess_block_30);
              block.mul (vqm[j]);
              hessian.addSubMatrix(windices[i], vindices[j], block);
              
              block.mulTransposeLeft (wqm[i], anghess_block_33);
              block.mul (wqm[j]);
              hessian.addSubMatrix(windices[i], vindices[j], block);
            }
            
            block.mulTransposeLeft (vqm[i], anghess_block_06);
            hessian.addSubMatrix(vindices[i], 3, block);
            
            block.mul(anghess_block_60, vqm[i]);
            hessian.addSubMatrix(3, vindices[i], block);
            
            block.mulTransposeLeft (vqm[i], anghess_block_06);
            block.scale (-1.0);
            hessian.addSubMatrix(vindices[i], 0, block);
            
            block.mul (anghess_block_60, vqm[i]);
            block.scale (-1.0);
            hessian.addSubMatrix(0, vindices[i], block);
            
            //
            
            block.mulTransposeLeft (wqm[i], anghess_block_36);
            hessian.addSubMatrix(windices[i], 3, block);
            
            block.mul(anghess_block_63, wqm[i]);
            hessian.addSubMatrix(3, windices[i], block);
            
            block.mulTransposeLeft (wqm[i], anghess_block_36);
            block.scale (-1.0);
            hessian.addSubMatrix(windices[i], 0, block);
            
            block.mul (anghess_block_63, wqm[i]);
            block.scale (-1.0);
            hessian.addSubMatrix(0, windices[i], block);
         }
        
         MatrixNd dang1M = new MatrixNd(1, 3);
         angderiv.getSubMatrix (0, 0, dang1M);
         Vector3d dang1 = new Vector3d(
            dang1M.get (0, 0), dang1M.get (0, 1), dang1M.get (0, 2));
         
         MatrixNd dang2M = new MatrixNd(1, 3);
         angderiv.getSubMatrix (0, 3, dang2M);
         Vector3d dang2 = new Vector3d(
            dang2M.get (0, 0), dang2M.get (0, 1), dang2M.get (0, 2));
         
         MatrixNd dang1mat = MatrixUtil.crossMatrixNd (dang1);
         MatrixNd dang2mat = MatrixUtil.crossMatrixNd (dang2);
         
         MatrixNd neg_dang1mat = new MatrixNd(dang1mat);
         neg_dang1mat.scale (-1.0);
         
         MatrixNd neg_dang2mat = new MatrixNd(dang2mat);
         neg_dang2mat.scale (-1.0);
         
         //
         
         hessian.addSubMatrix (6, 3, dang1mat);
         hessian.addSubMatrix (0, 3, neg_dang1mat);
         hessian.addSubMatrix (0, 6, dang1mat);
         hessian.addSubMatrix (3, 0, dang1mat);
         hessian.addSubMatrix (3, 6, neg_dang1mat);
         hessian.addSubMatrix (6, 0, neg_dang1mat);
         
         hessian.addSubMatrix (9, 0, dang2mat);
         hessian.addSubMatrix (3, 0, neg_dang2mat);
         hessian.addSubMatrix (3, 9, dang2mat);
         hessian.addSubMatrix (0, 3, dang2mat);
         hessian.addSubMatrix (0, 9, neg_dang2mat);
         hessian.addSubMatrix (9, 3, neg_dang2mat);
      }
      
      return theta;
   }
}
