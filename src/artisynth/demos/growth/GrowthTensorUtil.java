package artisynth.demos.growth;

import maspack.matrix.Matrix3d;

public class GrowthTensorUtil {
   
   /** Set to 2 if growth tensor is symmetrical. Otherwise, set to 1. */
   protected static int SYM_COUNT = 2;
   
   /** Number of unique components in the growth tensor. */
   public static int numStrainComp() {
      return (SYM_COUNT == 2) ? 6 : 9;
   }
   
   public static Matrix3d vecToMtx3d(double[] V) {
      if (SYM_COUNT == 2) {
         return new Matrix3d(
            V[0],           V[5]/SYM_COUNT, V[4]/SYM_COUNT,
            V[5]/SYM_COUNT, V[1],           V[3]/SYM_COUNT,
            V[4]/SYM_COUNT, V[3]/SYM_COUNT, V[2]); 
      }
      
      return new Matrix3d(
         V[0], V[5], V[4],
         V[8], V[1], V[3],
         V[7], V[6], V[2]); 
   }
   
   public static double[] mtx3dToVec(Matrix3d mtx) {
      if (SYM_COUNT == 2) {
         return new double[] {
             mtx.m00, mtx.m11, mtx.m22, 
             mtx.m12*SYM_COUNT, 
             mtx.m20*SYM_COUNT, 
             mtx.m01*SYM_COUNT};
      }
      
      return new double[] {
          mtx.m00, mtx.m11, mtx.m22,
          mtx.m12, mtx.m02, mtx.m01, 
          mtx.m21, mtx.m20, mtx.m10
      };
   }
}
