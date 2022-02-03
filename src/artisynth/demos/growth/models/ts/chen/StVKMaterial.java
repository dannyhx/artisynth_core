package artisynth.demos.growth.models.ts.chen;

import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.ShellElement3d;
import artisynth.demos.growth.models.ts.chen.GeometryDerivative.FirstFundamentalFormRv;
import maspack.geometry.Face;
import maspack.matrix.Matrix2d;
import maspack.matrix.MatrixNd;
import maspack.matrix.VectorNd;

public class StVKMaterial extends DiscreteShellMaterial {
   
   public double lameAlpha_;
   public double lameBeta_;
   
   /**
    * Calculate the stretching energy for the given element.
    * 
    * @param ele 
    * @param rs 
    * @param f
    * Array index of the element. 
    */
   public StretchingEnergyRv stretchingEnergy(
      ShellElement3d ele, 
      RestState rs, 
      int f,
      VectorNd derivative, 
      MatrixNd hessian
   ) {
      MonolayerRestState biRs = (MonolayerRestState)rs;
      
      //
      
      MatrixNd aderiv = new MatrixNd(4, 9);
      MatrixNd[] ahess = new MatrixNd[0];
      FirstFundamentalFormRv ffrv = GeometryDerivative.firstFundamentalForm (
         ele, 
         (derivative != null) ? aderiv : null, 
         (hessian != null) ? ahess : null);
      Matrix2d a = ffrv.Result;
      aderiv = ffrv.Derivative;
      ahess = ffrv.Hessian;
      
      //
      
      double coeff = biRs.thicknesses.get (f) / 4.0;
      
      Matrix2d abar = biRs.abars.get (f);
      Matrix2d abarinv = new Matrix2d(abar);
      boolean isInvert = abarinv.invert ();
      if (!isInvert) {
         throw new AssertionError("Failed to invert");
      }
      
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
//         derivative.mulTransposeLeft (aderiv, temp_4x1);
         derivative.scale (coeff * dA);
      }
      
      // Hessian
      
      if (hessian != null) {
         MatrixNd abarinv_4x1 = MatrixUtil.m2x2_to_mat4x1_colMaj (abarinv);
         MatrixNd inner = new MatrixNd(1, 9);
         inner.mulTransposeLeft (aderiv, abarinv_4x1);

         MatrixNd hessian_operand = new MatrixNd(9, 9);
         hessian_operand.mulTransposeLeft (inner, inner);
         hessian_operand.scale (lameAlpha_);
         hessian.add (hessian_operand);

         Matrix2d Mainv = new Matrix2d();
         Mainv.mul (M, abarinv);
         
         // Iterate over Mainv and abarinv as if they were vectors.
         
         VectorNd abarinv_vec = MatrixUtil.m2x2_to_vec4_colMaj (abarinv);
         VectorNd Mainv_vec = MatrixUtil.m2x2_to_vec4_colMaj (Mainv);
         
         for (int i = 0; i < 4; i++) {
            hessian.scaledAdd (
               lameAlpha_ * M.trace() * abarinv_vec.get (i) +
               2 * lameBeta_ * Mainv_vec.get(i), ahess[i]);
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

      StretchingEnergyRv rv = new StretchingEnergyRv();
      rv.Result = result;
      rv.Derivative = derivative;
      rv.Hessian = hessian;
      
      return rv;
   }
   
   public BendingEnergyRv bendingEnergy(
      FemModel3d model,
      ShellElement3d ele, 
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
      abarinv.invert(mrs.abars.get (f));
      MatrixNd bderiv = new MatrixNd(4, 18+3*nedgedofs);
      MatrixNd[] bhess = new MatrixNd[4];
      
      MidedgeAngleTanFormulation.secondFundamentalForm (
         model, 
         ele, 
         face, 
         (derivative != null) ? bderiv : null, 
         (hessian != null) ? bhess : null
      );
      
      
      
      
      
      
      
      
      
      
      BendingEnergyRv rv = new BendingEnergyRv();
      return rv;
   }
}
