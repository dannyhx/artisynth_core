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
   
   public static void add3x3Block(MatrixNd M, int i, int j, Matrix3d operand) {
      // Number of columns; row width.
      int nCols = M.colSize ();
      int nRows = M.rowSize ();
      
      double[] buf = M.getBuffer ();
      
      // Sanity check
      if (buf.length != nCols * nRows) {
         throw new ArithmeticException("Matrix buffer size is unexpected.");
      }
      
      buf[(i+0)*nCols + j + 0] += operand.m00;
      buf[(i+0)*nCols + j + 1] += operand.m01;
      buf[(i+0)*nCols + j + 2] += operand.m02;
      
      buf[(i+1)*nCols + j + 0] += operand.m10;
      buf[(i+1)*nCols + j + 1] += operand.m11;
      buf[(i+1)*nCols + j + 2] += operand.m12;
      
      buf[(i+2)*nCols + j + 0] += operand.m20;
      buf[(i+2)*nCols + j + 1] += operand.m21;
      buf[(i+2)*nCols + j + 2] += operand.m22;
   }
   
   public static void addBlock(MatrixNd M, int i, int j, MatrixNd block) {
      // Number of columns; row width.
      int mNumCols = M.colSize ();
      int mNumRows = M.rowSize ();
      
      double[] mBuf = M.getBuffer ();
      
      // Sanity check
      if (mBuf.length != mNumCols * mNumRows) {
         throw new ArithmeticException("Matrix buffer size is unexpected.");
      }
      
      ////
      
      int bNumCols = block.colSize ();
      int bNumRows = block.rowSize ();
      
      double[] bBuf = block.getBuffer ();
      
      // Sanity check
      if (bBuf.length != bNumCols * bNumRows) {
         throw new ArithmeticException("Matrix buffer size is unexpected.");
      }
      
      ////
      
      // For each row of the block.
      for (int br = 0; br < bNumRows; br++) {
         // For each col of the block relative to M.
         for (int bc = 0; bc < bNumCols; bc++) {
            mBuf[(i+br)*mNumCols + (j+bc)*mNumRows] += bBuf[br*bNumCols + bc];
         }
      }
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
   
   public static Vector4d m2x2_to_vec4_colMaj(Matrix2d M) {
      return new Vector4d(M.m00, M.m10, M.m01, M.m11);
   }
   
   public static Matrix3d crossMatrix(Vector3d v) {
      return new Matrix3d(
           0, -v.z,  v.y, 
         v.z,    0, -v.x, 
        -v.y,  v.x,  0);
   }
}
