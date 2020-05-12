package artisynth.demos.growth.models.paper;

import java.util.ArrayList;

import artisynth.demos.growth.util.MathUtil;
import maspack.geometry.Face;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point2d;
import maspack.matrix.Point3d;

public class _Exp_Toolbox {
   public static void trimStrip(PolygonalMesh mesh, Point2d stripStart2d,
      Point2d stripEnd2d, double stripRad, double minZ) {
      // Remove faces
      
      ArrayList<Face> facesToRemove = new ArrayList<Face>();
      for (Face face : mesh.getFaces ()) {
         for (Vertex3d fVtx : face.getVertices ()) {
            Point3d fVtxPos = fVtx.getPosition ();
            Point2d fVtxPos2d = MathUtil.to2d (fVtxPos);
            
            Point2d closetStripPt = MathUtil.getClosestPtToLineSegment2d (
               fVtxPos2d, stripStart2d, stripEnd2d);
            
            double dist2strip = fVtxPos2d.distance (closetStripPt);
            
            if (dist2strip < stripRad && fVtxPos.z > minZ) {
               facesToRemove.add (face);
               break;
            }
         }
      }
      
      mesh.removeFaces (facesToRemove);
      mesh.removeDisconnectedVertices ();
   }
   
   public static void trimVertically(PolygonalMesh mesh, double minNodeZ, double maxNodeZ) {
      // Remove faces
      
      ArrayList<Face> facesToRemove = new ArrayList<Face>();
      for (Face face : mesh.getFaces ()) {
         Point3d faceBotVtxPos = null;
         for (Vertex3d fVtx : face.getVertices ()) {
            if (faceBotVtxPos == null || fVtx.getPosition ().z < faceBotVtxPos.z) {
               faceBotVtxPos = fVtx.getPosition ();
            }
         }
         
         if (faceBotVtxPos.z < minNodeZ || faceBotVtxPos.z > maxNodeZ)
            facesToRemove.add (face);
      }
      
      mesh.removeFaces (facesToRemove);
      mesh.removeDisconnectedVertices ();
   }
   
   public static void drillDown(PolygonalMesh mesh, double drillRadius, double drillBotZ, double drillTopZ) {
      ArrayList<Face> facesToRemove = new ArrayList<Face>();
      for (Face face : mesh.getFaces ()) {
         for (Vertex3d fVtx : face.getVertices ()) {
            Point3d fVtxPos = fVtx.getPosition ();
            
            // Is within drill radius?
            boolean isWithinRadius = 
               new Point2d(fVtxPos.x, fVtxPos.y).norm () < drillRadius;
            
            boolean isWithinDrillLength =
               fVtxPos.z > drillBotZ && fVtxPos.z < drillTopZ;
               
            if (isWithinRadius && isWithinDrillLength) {
               facesToRemove.add (face);
               break;
            }
         }
      }
      
      mesh.removeFaces (facesToRemove);
      mesh.removeDisconnectedVertices ();
   }
   
   public static void trimRadius(PolygonalMesh mesh, double minRadius, double maxRadius) {

      ArrayList<Face> facesToRemove = new ArrayList<Face>();
      for (Face face : mesh.getFaces ()) {
         Point3d faceBotVtxPos = null;
         for (Vertex3d fVtx : face.getVertices ()) {
            if (faceBotVtxPos == null || fVtx.getPosition ().z < faceBotVtxPos.z) {
               faceBotVtxPos = fVtx.getPosition ();
            }
         }
           
         Point2d faceBotVtxPos2d = new Point2d(faceBotVtxPos.x, faceBotVtxPos.y);
         
         if (faceBotVtxPos2d.norm () < minRadius || faceBotVtxPos2d.norm () > maxRadius)
            facesToRemove.add (face);
      }
      
      mesh.removeFaces (facesToRemove);
      mesh.removeDisconnectedVertices ();
   }
   
   public static void trimRangeY(PolygonalMesh mesh, double minNodeY, double maxNodeY) {
      ArrayList<Face> facesToRemove = new ArrayList<Face>();
      for (Face face : mesh.getFaces ()) {
         for (Vertex3d vtx : face.getVertices ()) {
            Point3d pnt = vtx.getPosition ();
            if (pnt.y < minNodeY || pnt.y > maxNodeY) {
               facesToRemove.add (face);
               break;
            }
         }
      }
      
      mesh.removeFaces (facesToRemove);
      mesh.removeDisconnectedVertices ();
   }
   
   public static void trimHorizontallyX(PolygonalMesh mesh, double minNodeX, double maxNodeX) {
      ArrayList<Face> facesToRemove = new ArrayList<Face>();
      for (Face face : mesh.getFaces ()) {
         Point3d centroid = new Point3d();
         face.computeWorldCentroid (centroid);
           
         if (centroid.x < minNodeX || centroid.x > maxNodeX)
            facesToRemove.add (face);
      }
      
      mesh.removeFaces (facesToRemove);
      mesh.removeDisconnectedVertices ();
   }
   
   public static void confineInSphere(PolygonalMesh mesh, double confineRad) {
      ArrayList<Face> facesToRemove = new ArrayList<Face>();
      for (Face face : mesh.getFaces ()) {
         Point3d centroid = new Point3d();
         face.computeWorldCentroid (centroid);
           
         if (centroid.norm () > confineRad)
            facesToRemove.add (face);
      }
      
      mesh.removeFaces (facesToRemove);
      mesh.removeDisconnectedVertices ();
   }
   
   public static Point3d getTopPoint(PolygonalMesh mesh) {
      Point3d pnt = null;
      
      for (Vertex3d vtx : mesh.getVertices ()) {
         if (pnt == null || vtx.getPosition ().z > pnt.z) {
            pnt = new Point3d(vtx.getPosition ());
         }
      }
      
      return pnt;
   }
   
   public static Point3d getBotPoint(PolygonalMesh mesh) {
      Point3d botPnt = null;
      
      for (Vertex3d vtx : mesh.getVertices ()) {
         if (botPnt == null || vtx.getPosition ().z < botPnt.z) {
            botPnt = new Point3d(vtx.getPosition ());
         }
      }
      
      return botPnt;
   }
   
   
}
