package artisynth.demos.growth.thinshell;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.Map;

import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.ShellElement3d;
import artisynth.demos.growth.GrowTriElement;
import artisynth.demos.growth.util.MathUtil;
import artisynth.demos.growth.util.ShellUtil;
import maspack.geometry.Face;
import maspack.geometry.HalfEdge;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Point3d;

/**
 * Provides way to associate data to mesh edges. 
 */
public class EdgeDataMap {
   
   public class EdgeData {
      public double mAngStrain;
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
            
            edm.create (edge);
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
   
   
   /* --- Utils --- */
   
   
   public void useResidualPlasticStrain(FemModel3d model) {
      for (Map.Entry<HalfEdge, EdgeData> entry : this.mMap.entrySet()) {
         HalfEdge edge = entry.getKey();
         EdgeData edgeData = entry.getValue();
         
         double ang = ShellUtil.getDihedralAngle (model, edge, false);
         double angRest = ShellUtil.getDihedralAngle (model, edge, true);
         
         // Amount of deformation actually occurred, between t0 and t1.
         double angOcc = ang - angRest;
         
         // Remaining deformation to expect.
         edgeData.mAngStrain -= angOcc;
     }
   }
}

