package _custom.cont;

import maspack.matrix.Vector2d;

/**
 * Contains computed collision info.
 */
public class CCRV {
   /** Time when collision occurred. Between t=0 and t=1. 
    *  -1 indicates no collision. */
   public double hitTime;
   
   /* --- Vertex-triangle collision --- */
   
   /** Barycentric coordinates of triangle, indicating where the collision
     * occurred on the triangle surface. */ 
   public Vector2d bary;
   
   /* --- Edge-Edge collision --- */
   
   /** Minimum distance between two edges at the time of collision. */
   public double minDist;
   
   /** Fraction along line segment from tail to head of edge0 where collision occurred. */
   public double r;
   
   /** Fraction along line segment from tail to head of edge1 where collision occurred. */
   public double s;
   
   public CCRV() {
      this.hitTime = -1;
   }
   
   public CCRV(double hitTime) {
      this.hitTime = hitTime;
   }
}
