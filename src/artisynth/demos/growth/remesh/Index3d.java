package artisynth.demos.growth.remesh;

import maspack.matrix.Point3d;

public class Index3d extends Point3d {

   private static final long serialVersionUID = 1L;
   public int idx = -1;
   
   public Index3d(Point3d coord, int idx) {
      this.x = coord.x; 
      this.y = coord.y; 
      this.z = coord.z;
      this.idx = idx;
   }
}
