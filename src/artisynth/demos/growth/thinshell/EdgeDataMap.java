package artisynth.demos.growth.thinshell;

import java.util.HashMap;

import artisynth.demos.growth.util.MathUtil;
import maspack.geometry.Face;
import maspack.geometry.HalfEdge;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Point3d;

/**
 * Provides way to associate data to mesh edges. 
 */
public class EdgeDataMap {
   
   public class EdgeData {
      public double mRestTheta;
   }
   
   protected HashMap<HalfEdge,EdgeData> mMap;
   
   public EdgeDataMap() {
      this.mMap = new HashMap<HalfEdge,EdgeData>();
   }
   
   public static EdgeDataMap createFromMesh(PolygonalMesh mesh) {
      EdgeDataMap edm = new EdgeDataMap();
      
      for (Face face : mesh.getFaces ()) {
         for (int e = 0; e < 3; e++) {
            HalfEdge edge = face.getEdge (e);
            
            // Its real counter-part will create the EdgeData instead.
            if (!isRealHalfEdge(edge) || edge.opposite == null) {
               continue;
            }
            
            EdgeData data = edm.create (edge);
            
            Face edgeFace = edge.getFace ();
            Face edgeFaceOpp = edge.getOppositeFace ();
            Point3d x0 = edge.head.getPosition ();
            Point3d x1 = edge.tail.getPosition ();
            
            data.mRestTheta = MathUtil.dihedralAngle (
               x0, x1, edgeFace.getNormal (), edgeFaceOpp.getNormal ());
            
//            data.mRestTheta = 1.00;
         }
      }
      
      return edm;
   }
   
   public EdgeData create(HalfEdge edge) {
      edge = getRealHalfEdge(edge);
      
//      System.out.printf ("Edge: [%s,%s]\n", edge.head.getIndex (), edge.tail.getIndex ());
      
      EdgeData data = mMap.get (edge);
      if (data != null) {
         throw new RuntimeException("Entry already exists.");
      }
      
      data = new EdgeData();
      mMap.put (edge, data);
      
      return data; 
   }
   
   public EdgeData remove(HalfEdge edge) {
      edge = getRealHalfEdge(edge);
      EdgeData data = mMap.remove (edge);
      if (data == null) {
         throw new RuntimeException("Entry does not exist for deletion.");
      }
      
      return data;
   }
   
   public EdgeData get(HalfEdge edge) {
      edge = getRealHalfEdge(edge);
      return mMap.get (edge); 
   }
   
   //
   
   public static boolean isRealHalfEdge(HalfEdge edge) {
      return (edge.head.getIndex () < edge.tail.getIndex ()); 
   }
   
   public static HalfEdge getRealHalfEdge(HalfEdge edge) {
      if (edge.opposite != null && 
          edge.head.getIndex () > edge.tail.getIndex ()) 
      {
         return edge.opposite;
      }
      
      return edge; 
   }
   
}

