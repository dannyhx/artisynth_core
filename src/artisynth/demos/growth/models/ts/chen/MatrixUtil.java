package artisynth.demos.growth.models.ts.chen;

import maspack.matrix.Matrix2d;
import maspack.matrix.Matrix3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Vector3d;
import maspack.matrix.Vector4d;
import maspack.matrix.VectorNd;

public class MatrixUtil {
   /**
    * For a specified 1x3 block within the given matrix, increment the block
    * using a Vec3.
    * 
    * This is equivalent to Eigen's "M<1,3>(i,j) += operand" operation.
    * 
    * @param M
    * @param i
    * @param j
    * @param operand
    */
   public static void add1x3Block(MatrixNd M, int i, int j, Vector3d operand) {
      // Number of columns; row width.
      int nCols = M.colSize ();
      int nRows = M.rowSize ();
      
      double[] buf = M.getBuffer ();
      
      // Sanity check
      if (buf.length != nCols * nRows) {
         throw new ArithmeticException("Matrix buffer size is unexpected.");
      }
      
      buf[i*nCols + j + 0] += operand.get (0);
      buf[i*nCols + j + 1] += operand.get (1);
      buf[i*nCols + j + 2] += operand.get (2);
   }
   
   /**
    * Map the given Matrix2D into a vector, in col-major order.
    * 
    * Example:
    *   [1 2] -> [1 3 2 4]
    *   [3 4] 
    */
   public static MatrixNd m2x2_to_mat4x1_colMaj(Matrix2d M) {
      MatrixNd rv = new MatrixNd(4, 1);
      rv.set (0, 0, M.m00);
      rv.set (1, 0, M.m10);
      rv.set (2, 0, M.m01);
      rv.set (3, 0, M.m11);
      return rv;
   }
   
   public static VectorNd m2x2_to_vec4_colMaj(Matrix2d M) {
      return new VectorNd(M.m00, M.m10, M.m01, M.m11);
   }
   
   public static Matrix3d crossMatrix(Vector3d v) {
      return new Matrix3d(
           0, -v.z,  v.y, 
         v.z,    0, -v.x, 
        -v.y,  v.x,  0);
   }
   
   public static MatrixNd crossMatrixNd(Vector3d v) {
      MatrixNd M = new MatrixNd(3, 3);
      M.set(new double[] {
           0, -v.z,  v.y, 
         v.z,    0, -v.x, 
        -v.y,  v.x,  0});
      return M; 
   }
}
