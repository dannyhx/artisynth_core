package artisynth.demos.growth.thinshell;

import java.util.HashMap;
import java.util.Map;

import artisynth.core.femmodels.FemModel3d;
import artisynth.demos.growth.util.ShellUtil;
import maspack.geometry.Face;
import maspack.geometry.HalfEdge;
import maspack.geometry.PolygonalMesh;

/**
 * Provides way to associate data to mesh edges. 
 */
public class EdgeDataMap {
   
   public class EdgeData {
      public double mAngStrain;
   }
   
   public HashMap<HalfEdge,EdgeData> mMap;
   protected FemModel3d mModel;
   
   public EdgeDataMap(FemModel3d model, PolygonalMesh mesh) {
      this.mMap = new HashMap<HalfEdge,EdgeData>();
      this.mModel = model;
      
      //
      
      for (Face face : mesh.getFaces ()) {
         for (int e = 0; e < 3; e++) {
            HalfEdge edge = face.getEdge (e);
            
            // Its real counter-part will create the EdgeData instead.
            if (!isRealHalfEdge(edge) || edge.opposite == null) {
               continue;
            }
            
            this.create (edge);
         }
      }
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
   
   
   public void useResidualPlasticStrain() {
      for (Map.Entry<HalfEdge, EdgeData> entry : this.mMap.entrySet()) {
         HalfEdge edge = entry.getKey();
         EdgeData edgeData = entry.getValue();
         
         double ang = ShellUtil.getDihedralAngle (mModel, edge, false);
         double angRest = ShellUtil.getDihedralAngle (mModel, edge, true);
         
         // Amount of deformation actually occurred, between t0 and t1.
         double angOcc = ang - angRest;
         
         // Remaining deformation to expect.
         edgeData.mAngStrain -= angOcc;
     }
   }
   
}

