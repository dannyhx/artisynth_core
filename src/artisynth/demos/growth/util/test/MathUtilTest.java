package artisynth.demos.growth.util.test;

import artisynth.demos.growth.util.MathUtil;
import maspack.matrix.Point2d;
import maspack.matrix.Point3d;

public class MathUtilTest {
   
   protected void test_trianglePointTo2D() {
      Point3d[] tri = new Point3d[] {
          new Point3d(1,1,7),
          new Point3d(4,1,7), 
          new Point3d(1,4,7)};
                                  
      Point2d p2d = MathUtil.trianglePointTo2D (tri, new Point3d(2,1,7));
      
      System.out.println (p2d);
   }
   
   
   public static void main(String[] args) {
      MathUtilTest t = new MathUtilTest();
      t.test_trianglePointTo2D ();
   }
}
