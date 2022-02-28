package artisynth.demos.growth.models.ts.chen;

import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.ShellElement3d;
import maspack.geometry.Face;
import maspack.matrix.Matrix2d;
import maspack.matrix.MatrixNd;
import maspack.matrix.VectorNd;

public class StVKMaterial extends DiscreteShellMaterial {
   
   public double lameAlpha_;
   public double lameBeta_;
   
   public StVKMaterial(double poissons) {
      // main.cpp::lameParameters
      double young = 1.0; // "doesn't matter for static solves"
      this.lameAlpha_ = young * poissons / (1.0 - poissons * poissons);
      this.lameBeta_ = young / 2.0 / (1.0 + poissons);
   }
   
   /**
    * Calculate the stretching energy for the given element.
    * 
    * Called by DiscreteShell.elasticEnergy().
    * Verified.
    * 
    * @param ele 
    * @param rs 
    * @param f
    * Array index of the element. 
    */
   public double stretchingEnergy(
      MeshConnectivity MC, 
      FemModel3d model,
      RestState rs, 
      int f,
      VectorNd derivative, // 1x9
      MatrixNd hessian     // 9x9
   ) {
      MonolayerRestState biRs = (MonolayerRestState)rs;
      
      double coeff = biRs.thicknesses.get (f) / 4.0;
      
      Matrix2d abar = biRs.abars.get (f);
      Matrix2d abarinv = new Matrix2d(abar);
      boolean isInvert = abarinv.invert ();
      if (!isInvert) {
         throw new AssertionError("Failed to invert");
      }
      
      //
      
      MatrixNd aderiv = new MatrixNd(4, 9);
      MatrixNd[] ahess = new MatrixNd[4];
      Matrix2d a = GeometryDerivative.firstFundamentalForm (
         MC, model, f,
         (derivative != null) ? aderiv : null, 
         (hessian != null) ? ahess : null);
      
      //
      
      Matrix2d M = new Matrix2d();
      M.sub (a, abar);
      M.mul (abarinv, M);
      double dA = 0.5 *  sqrt(abar.determinant ());
      
      // 
      
      Matrix2d MM = new Matrix2d();
      MM.mul (M, M);
      double StVK = 0.5 * lameAlpha_ * pow(M.trace (), 2) + lameBeta_ * MM.trace ();
      double result = coeff * dA * StVK;
      
      // Derivative

      if (derivative != null) {
         Matrix2d M_abarinv = new Matrix2d();
         M_abarinv.mul (M, abarinv);
         
         Matrix2d temp = new Matrix2d();
         temp.scaledAdd (lameAlpha_ * M.trace (), abarinv);
         temp.scaledAdd (2 * lameBeta_, M_abarinv);
         
         VectorNd temp_vec4 = MatrixUtil.m2x2_to_vec4_colMaj (temp);
         derivative.mulTranspose (aderiv, temp_vec4);
         derivative.scale (coeff * dA);
      }
      
      // Hessian
      
      if (hessian != null) {
         MatrixNd abarinv_4x1 = MatrixUtil.m2x2_to_mat4x1_colMaj (abarinv);
         MatrixNd inner = new MatrixNd();
         inner.mulTransposeLeft (aderiv, abarinv_4x1);
         inner.transpose ();  // 9x1 to 1x9

         MatrixNd hessian_operand = new MatrixNd(9, 9);
         hessian_operand.mulTransposeLeft (inner, inner);
         hessian_operand.scale (lameAlpha_);
         hessian.add (hessian_operand);

         Matrix2d Mainv = new Matrix2d();
         Mainv.mul (M, abarinv);
         
         // "iterate over Mainv and abarinv as if they were vectors"
         
         VectorNd abarinv_vec = MatrixUtil.m2x2_to_vec4_colMaj (abarinv);
         VectorNd Mainv_vec = MatrixUtil.m2x2_to_vec4_colMaj (Mainv);
         
         for (int i = 0; i < 4; i++) {
            hessian.scaledAdd (
               lameAlpha_ * M.trace() * abarinv_vec.get (i) +
               2 * lameBeta_ * Mainv_vec.get(i), 
               ahess[i]);
         }
         
         VectorNd aderiv_row0 = new VectorNd(9);
         VectorNd aderiv_row1 = new VectorNd(9);
         VectorNd aderiv_row2 = new VectorNd(9);
         VectorNd aderiv_row3 = new VectorNd(9);
         
         aderiv.getRow (0, aderiv_row0);
         aderiv.getRow (1, aderiv_row1);
         aderiv.getRow (2, aderiv_row2);
         aderiv.getRow (3, aderiv_row3);

         // 
         
         VectorNd inner00_vec = new VectorNd(9);
         inner00_vec.scaledAdd (abarinv.get (0, 0), aderiv_row0);
         inner00_vec.scaledAdd (abarinv.get (0, 1), aderiv_row2);
         
         VectorNd inner01_vec = new VectorNd(9);
         inner01_vec.scaledAdd (abarinv.get (0, 0), aderiv_row1);
         inner01_vec.scaledAdd (abarinv.get (0, 1), aderiv_row3);
         
         VectorNd inner10_vec = new VectorNd(9);
         inner10_vec.scaledAdd (abarinv.get (1, 0), aderiv_row0);
         inner10_vec.scaledAdd (abarinv.get (1, 1), aderiv_row2);
         
         VectorNd inner11_vec = new VectorNd(9);
         inner11_vec.scaledAdd (abarinv.get (1, 0), aderiv_row1);
         inner11_vec.scaledAdd (abarinv.get (1, 1), aderiv_row3);
         
         MatrixNd inner00 = new MatrixNd(1, 9);
         inner00.setRow (0, inner00_vec);
         
         MatrixNd inner01 = new MatrixNd(1, 9);
         inner01.setRow (0, inner01_vec);
         
         MatrixNd inner10 = new MatrixNd(1, 9);
         inner10.setRow (0, inner10_vec);
         
         MatrixNd inner11 = new MatrixNd(1, 9);
         inner11.setRow (0, inner11_vec);
         
         hessian_operand.mulTransposeLeft (inner00, inner00);
         hessian_operand.scale (2 * lameBeta_);
         hessian.add (hessian_operand);
         
         MatrixNd tmp = new MatrixNd(1, 9);
         tmp.mulTransposeLeft (inner01, inner10);
         hessian_operand.mulTransposeLeft (inner10, inner01);
         hessian_operand.add (tmp);
         hessian_operand.scale (2 * lameBeta_);
         hessian.add (hessian_operand);
         
         hessian_operand.mulTransposeLeft (inner11, inner11);
         hessian_operand.scale (2 * lameBeta_);
         hessian.add (hessian_operand);
         
         hessian.scale (coeff * dA);
      }

      return result;
   }
   
   /**
    * Calculate the bending energy for the given face.
    * 
    * Verified.
    * 
    * @param derivative [18+3*numExtraDOFs]
    * @param hessian    [18+3*numExtraDOFs, 18+3*numExtraDOFs]
    */
   public double bendingEnergy(
      FemModel3d model,
      VectorNd extraDOFs,
      MeshConnectivity MC,
      RestState rs, 
      Face face,
      int f,
      int numExtraDOFs,
      VectorNd derivative,   
      MatrixNd hessian        
   ) {
      MonolayerRestState mrs = (MonolayerRestState) rs;
      
      double coeff = pow(mrs.thicknesses.get (f), 3) / 12;
      int nedgedofs = numExtraDOFs;
      Matrix2d abarinv = new Matrix2d();
      boolean isInverted = abarinv.invert(mrs.abars.get (f));
      if (!isInverted) {
         throw new RuntimeException("Failed to invert");
      }
      MatrixNd bderiv = new MatrixNd(4, 18+3*nedgedofs);
      MatrixNd[] bhess = new MatrixNd[4];
      
      Matrix2d b = MidedgeAngleTanFormulation.secondFundamentalForm (
         model, 
         extraDOFs,
         MC,
         face, 
         (derivative != null) ? bderiv : null, 
         (hessian != null) ? bhess : null
      );
      
//      System.out.println (bhess[0].toString ("%.5f"));
//      System.out.println (bhess[1].toString ("%.5f"));
//      System.out.println (bhess[2].toString ("%.5f"));
//      System.out.println (bhess[3].toString ("%.5f"));
      
      Matrix2d M = new Matrix2d();
      M.sub (b, mrs.bbars.get (f));
      M.mul(abarinv, M);
      double dA = 0.5 * sqrt(mrs.abars.get (f).determinant ());
      
      Matrix2d M2 = new Matrix2d();
      M2.mul (M, M);
      
      double StVK = 0.5 * this.lameAlpha_ * pow(M.trace (), 2) + 
         this.lameBeta_ * M2.trace();
      double result = coeff * dA * StVK;
      
      if (derivative != null) {
         Matrix2d temp = new Matrix2d();
         temp.mul(M, abarinv);
         temp.scale (2 * lameBeta_);
         temp.scaledAdd (lameAlpha_ * M.trace(), abarinv);
         
         VectorNd temp_vec = MatrixUtil.m2x2_to_vec4_colMaj (temp);
         derivative.mulTranspose (bderiv, temp_vec);
         derivative.scale (coeff * dA);
      }
      
      if (hessian != null) {
         MatrixNd abarinv_vec = MatrixUtil.m2x2_to_mat4x1_colMaj (abarinv);
         
         MatrixNd inner = new MatrixNd();
         inner.mulTransposeLeft (bderiv, abarinv_vec);
         inner.transpose ();  // [1, 18 + 3 * nedgedofs]
                  
         hessian.mulTransposeLeft (inner, inner);
         hessian.scale (lameAlpha_);
         
         Matrix2d Mainv = new Matrix2d();
         Mainv.mul (M, abarinv);
         for (int i = 0; i < 4; i++) {
            // iterate over Mainv and abarinv as if they were vectors
            hessian.scaledAdd (
               lameAlpha_ * M.trace() * abarinv.get (i/2, i%2) + 
               2 * lameBeta_ * Mainv.get (i/2, i%2), 
               bhess[i]);
         }
                  
         MatrixNd[] bderiv_rows = new MatrixNd[4];
         for (int i = 0; i < 4; i++) {
            bderiv_rows[i] = new MatrixNd(1, 18 + 3 * nedgedofs);
            bderiv.getSubMatrix (i, 0, bderiv_rows[i]);
         }
         
         MatrixNd inner00 = new MatrixNd(1, 18 + 3 * nedgedofs);
         inner00.scaledAdd (abarinv.get (0, 0), bderiv_rows[0]);
         inner00.scaledAdd (abarinv.get (0, 1), bderiv_rows[2]);
         
         MatrixNd inner01 = new MatrixNd(1, 18 + 3 * nedgedofs);
         inner01.scaledAdd (abarinv.get (0, 0), bderiv_rows[1]);
         inner01.scaledAdd (abarinv.get (0, 1), bderiv_rows[3]);
         
         MatrixNd inner10 = new MatrixNd(1, 18 + 3 * nedgedofs);
         inner10.scaledAdd (abarinv.get (1, 0), bderiv_rows[0]);
         inner10.scaledAdd (abarinv.get (1, 1), bderiv_rows[2]);
         
         MatrixNd inner11 = new MatrixNd(1, 18 + 3 * nedgedofs);
         inner11.scaledAdd (abarinv.get (1, 0), bderiv_rows[1]);
         inner11.scaledAdd (abarinv.get (1, 1), bderiv_rows[3]);
         
         MatrixNd hessian_operand = new MatrixNd();
         hessian_operand.mulTransposeLeft (inner00, inner00);
         hessian.scaledAdd (2 * lameBeta_, hessian_operand);

         hessian_operand.mulTransposeLeft (inner01, inner10);
         hessian_operand.mulTransposeLeftAdd (inner10, inner01);
         hessian.scaledAdd (2 * lameBeta_, hessian_operand);
         
         hessian_operand.mulTransposeLeft (inner11, inner11);
         hessian.scaledAdd (2 * lameBeta_, hessian_operand);
         
         hessian.scale (coeff * dA);
      }
      
      return result;
   }
}
