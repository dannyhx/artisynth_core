package artisynth.demos.growth.util;

import maspack.matrix.DenseMatrixBase;
import maspack.matrix.MatrixNd;
import maspack.matrix.Vector;
import maspack.matrix.VectorNd;

/*
Copyright ©2013 The Regents of the University of California
(Regents). All Rights Reserved. Permission to use, copy, modify, and
distribute this software and its documentation for educational,
research, and not-for-profit purposes, without fee and without a
signed licensing agreement, is hereby granted, provided that the
above copyright notice, this paragraph and the following two
paragraphs appear in all copies, modifications, and
distributions. Contact The Office of Technology Licensing, UC
Berkeley, 2150 Shattuck Avenue, Suite 510, Berkeley, CA 94720-1620,
(510) 643-7201, for commercial licensing opportunities.

IN NO EVENT SHALL REGENTS BE LIABLE TO ANY PARTY FOR DIRECT,
INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING
LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
DOCUMENTATION, EVEN IF REGENTS HAS BEEN ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.

REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE. THE SOFTWARE AND ACCOMPANYING
DOCUMENTATION, IF ANY, PROVIDED HEREUNDER IS PROVIDED "AS
IS". REGENTS HAS NO OBLIGATION TO PROVIDE MAINTENANCE, SUPPORT,
UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
*/

public class SolverUtil {
   /**
    * vectors.cpp::solve_llsq
    * 
    * @param A
    * @param b
    * @return
    */
   public static VectorNd solve_llsq(DenseMatrixBase A, VectorNd b) {
      int m = A.rowSize ();
      int n = A.colSize ();
      
      if (b.size () != m) {
         throw new RuntimeException("b vector does not match row size of A.");
      }
      
      MatrixNd M = new MatrixNd(n, n);
      VectorNd y = new VectorNd(n);
      
      for (int i = 0; i < n; i++) {
         VectorNd iCol = new VectorNd(m);
         A.getColumn (i, iCol);
         y.set (i, b.dot (iCol));
         
         for (int j = 0; j < n; j++) {
            VectorNd jCol = new VectorNd(m);
            A.getColumn (j, jCol);
            M.set (i, j, iCol.dot (jCol));
         }
      }
      
      if (M.frobeniusNorm () == 0) {
         throw new RuntimeException("Detected zero frobenius norm matrix.");
      }
      
      if (n == 3) {
         return solve_symmetric_3x3(M, y);
      }
      
      throw new RuntimeException("Unsupported matrix size for linear solver.");
   }
   
   /**
    * vectors.cpp::solve_symmetric
    * 
    * @param M
    * @param y
    * @return
    */
   protected static VectorNd solve_symmetric_3x3(DenseMatrixBase A, Vector b) {
      if (A.rowSize () != 3 || A.colSize () != 3) {
         throw new RuntimeException("Expected 3x3 matrix.");
      }
      
      double t13 = A.get(1,2)*A.get(1,2);
      double t14 = A.get(0,2)*A.get(0,2);
      double t15 = A.get(0,0)*t13;
      double t16 = A.get(1,1)*t14;
      double t17 = A.get(0,1)*A.get(0,1);
      double t18 = A.get(2,2)*t17;
      double t21 = A.get(0,1)*A.get(0,2)*A.get(1,2)*2.0;
      double t22 = A.get(0,0)*A.get(1,1)*A.get(2,2);
      double t19 = t15+t16+t18-t21-t22;
      
      if (Math.abs (t19) == 0) {
         throw new RuntimeException("Detected singular matrix.");
      }
      
      double t20 = 1/t19;
      
      return new VectorNd(
         t20*(t13*b.get(0)+A.get(0,2)*
            (A.get(1,1)*b.get(2)-A.get(1,2)*b.get(1))-A.get(0,1)*
            (A.get(1,2)*b.get(2)-A.get(2,2)*b.get(1))-A.get(1,1)*
            A.get(2,2)*b.get(0)),
         
         t20*(t14*b.get(1)+A.get(1,2)*
         (A.get(0,0)*b.get(2)-A.get(0,2)*b.get(0))
         -A.get(0,1)*(A.get(0,2)*b.get(2)-A.get(2,2)*
         b.get(0))-A.get(0,0)*A.get(2,2)*b.get(1)),
         
         t20*(t17*b.get(2)+A.get(1,2)*(A.get(0,0)*b.get(1)-A.get(0,1)*b.get(0))
         -A.get(0,2)*(A.get(0,1)*b.get(1)-A.get(1,1)*b.get(0))
         -A.get(0,0)*A.get(1,1)*b.get(2))
      );
   }
}
