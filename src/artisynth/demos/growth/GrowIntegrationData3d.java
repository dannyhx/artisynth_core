package artisynth.demos.growth;

import artisynth.core.femmodels.IntegrationData3d;
import maspack.matrix.Matrix3d;

/** 
 * Extension of IntegrationData3d to account for growth. 
 * 
 * IntegrationData3d contains non-constant variables of its corresponding
 * IntegrationPoint3d.
 */
public class GrowIntegrationData3d extends IntegrationData3d {

   /** Plastic deformation gradient. For linear material, this is
    *  `I + plastic strain`. */
   protected Matrix3d Fp = new Matrix3d(Matrix3d.IDENTITY);
   
   /**
    * Get the plastic deformation gradient. 
    * 
    * By default, it is null or an identity matrix, indicating that no 
    * plastic strain is present.
    */
   public Matrix3d getFp() {
      return Fp;
   }
   
   /**
    * Set the plastic deformation gradient.
    * 
    * By default, preF is null or an identity matrix, indicating that no 
    * plastic strain is present.
    */
   public void setFp(Matrix3d Fp) {
      this.Fp = Fp;
   }
   
   /**
    * Increment the plastic deformation gradient, element-wise.
    * 
    * For example, to double the element size along its x-axis, `additionalFp`
    * should be set to [-0.5 0 0] where `additionalFp` is an diagonal matrix.
    * 
    * The negative sign in -0.5 is intended, which will cause the 
    * model to believe that is it "shrunk" and should expand to retrieve the
    * strain.
    */
   public void addFp(Matrix3d additionalFp) {
      this.Fp.add (additionalFp);
   }

}
