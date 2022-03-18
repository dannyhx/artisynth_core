package artisynth.demos.growth.models.ts.evouga;

import java.util.ArrayList;

import maspack.matrix.MatrixNd;
import maspack.matrix.SparseMatrixNd;

public class MatrixCell {
   public int i;
   public int j;
   public double val;
   
   public MatrixCell(int i, int j, double val) {
      this.i = i;
      this.j = j;
      this.val = val;
   }
   
   public static MatrixNd BuildMatrixNd(int nRows, int nCols, ArrayList<MatrixCell> cells) {
      MatrixNd M = new MatrixNd(nRows, nCols);
      for (MatrixCell cell : cells) {
         double existingVal = M.get (cell.i, cell.j);
         M.set (cell.i, cell.j, cell.val + existingVal);
      }
      return M;
   }
   
   public static SparseMatrixNd BuildSparseMatrixNd(int nRows, int nCols, ArrayList<MatrixCell> cells) {
      SparseMatrixNd M = new SparseMatrixNd(nRows, nCols);
      for (MatrixCell cell : cells) {
         double existingVal = M.get (cell.i, cell.j);
         M.set (cell.i, cell.j, cell.val + existingVal);
      }
      return M;
   }
}
