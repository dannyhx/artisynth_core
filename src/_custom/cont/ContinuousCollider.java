package _custom.cont;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.Set;

import maspack.collision.ContactInfo;
import maspack.collision.EdgeEdgeContact;
import maspack.collision.PenetratingPoint;
import maspack.geometry.BVNode;
import maspack.geometry.Boundable;
import maspack.geometry.Face;
import maspack.geometry.HalfEdge;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;
import maspack.util.Pair;

public class ContinuousCollider {

   public static boolean myDebug = true;
   
   protected ContinuousCollisions myContCol;
   
   public static double myTimeElipson = 1e-4;
   public static double mySpaceElipson = 1e-6;
   
   public static double myDistScale = 1.0;
   
   public ContinuousCollider() {
      myContCol = new ContinuousCollisions();
   }
   
   int m = 0;
   
   /* --- Primary Methods (public) --- */
   
   public ContactInfo getContacts (SweptMeshInfo smi0, SweptMeshInfo smi1, 
   boolean isDynamic0, boolean isDynamic1) {
      
      if (! isDynamic0 && ! isDynamic1) 
         return null;
      
      ContactInfo cinfo = new ContactInfo(smi0.myMesh, smi1.myMesh);
      
      System.out.println ("Computing penetration points for m=0");
      m = 0;
      ArrayList<PenetratingPoint> pentPts0 = findPenetratingPoints(smi0, smi1);
      
      System.out.println ("Computing penetration points for m=1");
      m = 1;
      ArrayList<PenetratingPoint> pentPts1 = (smi0 != smi1) ? 
         findPenetratingPoints(smi1, smi0) : new ArrayList<PenetratingPoint>();
       
//      pentPts0 = new ArrayList<PenetratingPoint>();
//      pentPts1 = new ArrayList<PenetratingPoint>();
         
      cinfo.setPenetratingPoints (pentPts0, 0);
      cinfo.setPenetratingPoints (pentPts1, 1);    

      System.out.println ("Computing edge-edge contacts for m=0 vs m=1");
      ArrayList<EdgeEdgeContact> eeCts = findEdgeEdgeContacts (smi0, smi1);
//      eeCts = new ArrayList<EdgeEdgeContact>();
      cinfo.setEdgeEdgeContacts (eeCts);

      if (cinfo.getPenetratingPoints(0).isEmpty () &&
          cinfo.getPenetratingPoints(1).isEmpty () &&
          cinfo.getEdgeEdgeContacts().isEmpty ())
         return null;
      
      removeRedundantCollisions(cinfo);
      
      if (myDebug)
         printCollisions(cinfo);
         
      return cinfo;
   }
   
   /* --- Secondary Methods --- (protected) */
   
   
   /**
    * Among all the detected vertex-triangle and edge-edge collisions,
    * filter-out the following collisions:
    * 
    *   - Edge-Edge collision where an edge's vertex collided at an earlier 
    *     hitTime.
    *   - Vertex-Tri collision where a connected edge of the vertex has an 
    *     earlier hitTime.
    */
   protected void removeRedundantCollisions(ContactInfo cinfo) {
      // Don't include edge-edge collision if an edge's vertices caused a
      // vertex-triangle at an earlier hitTime.
      cinfo.getEdgeEdgeContacts ().removeIf (eeCt -> 
         isExistEarlierVtxTriContact (
            cinfo.getPenetratingPoints (0), eeCt.edge0, eeCt.hitTime) || 
         isExistEarlierVtxTriContact (
            cinfo.getPenetratingPoints (1), eeCt.edge1, eeCt.hitTime)
      );
  
      // Don't include vertex-triangle collision if an edge, which is connected
      // to the vertex, has an earlier collide time.
      for (int m=0; m<2; m++) {
         final int fm = m;
         cinfo.getPenetratingPoints (fm).removeIf (pentPt -> 
            isExistEarlierEdgeEdgeContact (
               fm, cinfo.getEdgeEdgeContacts (), pentPt.vertex, pentPt.hitTime)
         );
      }
      
      // Handle situations where 2 redundant edge-edge contacts exist:
      //   [10,20]-[30,40] and [30,40]-[10,20] 
      cinfo.getEdgeEdgeContacts ().removeIf (eeCt -> 
         isExistingInverseEdgeEdgeContact (eeCt, cinfo.getEdgeEdgeContacts ()) &&
         isEdge0HeadGreaterThanEdge1Head (eeCt)  // Prevent removing both
      );
      
      
      // Find earliest hitTime
      
//      double minHitTime = Double.POSITIVE_INFINITY;
//      
//      for (EdgeEdgeContact eeCt : cinfo.getEdgeEdgeContacts ())
//         minHitTime = Math.min (eeCt.hitTime, minHitTime);
//      
//      for (int m=0; m<2; m++)
//         for (PenetratingPoint pentPt : cinfo.getPenetratingPoints (m)) 
//            minHitTime = Math.min (pentPt.hitTime, minHitTime);
//            
//      // Remove collisions that have a longer hitTime than the minimum
//      
//      ArrayList<EdgeEdgeContact> eeCts_toRemove = new ArrayList<EdgeEdgeContact>();
//      for (EdgeEdgeContact eeCt : cinfo.getEdgeEdgeContacts ())
//         if (eeCt.hitTime > minHitTime + myTimeElipson) {
////            System.out.printf ("Skipping [%d-%d][%d-%d] collision because earlier collision exists.\n",
////               eeCt.edge0.head.getIndex (), eeCt.edge0.tail.getIndex (), 
////               eeCt.edge1.head.getIndex (), eeCt.edge1.tail.getIndex ());
//            eeCts_toRemove.add (eeCt);
//         }
//      cinfo.getEdgeEdgeContacts ().removeAll (eeCts_toRemove);
//      
//      for (int m=0; m<2; m++) {
//         ArrayList<PenetratingPoint> pentPts_toRemove = new ArrayList<PenetratingPoint>();
//         for (PenetratingPoint pentPt : cinfo.getPenetratingPoints (m)) 
//            if (pentPt.hitTime > minHitTime + myTimeElipson) {
////               System.out.printf ("Skipping [%d]%s collision because earlier collision exists.\n",
////                  pentPt.vertex.getIndex (), pentPt.face.vertexStr ());
//               pentPts_toRemove.add (pentPt);
//            }
//         cinfo.getPenetratingPoints (m).removeAll (pentPts_toRemove);
//      }
      
   }
   
   protected ArrayList<PenetratingPoint> findPenetratingPoints(
   SweptMeshInfo smi0, SweptMeshInfo smi1) {
      
      // For each vertex, find set of colliding triangles (broad-phase).
      
      ArrayList<BVNode> ixtVertexNodes = new ArrayList<BVNode>();
      ArrayList<BVNode> ixtFaceNodes = new ArrayList<BVNode>();
      
      smi0.myVertexTree.setBvhToWorld ( smi0.myMesh.getMeshToWorld () );
      smi1.myTriangleTree.setBvhToWorld ( smi1.myMesh.getMeshToWorld () );

      smi0.myVertexTree.intersectTree (ixtVertexNodes, ixtFaceNodes, 
         smi1.myTriangleTree);
      
      Boundable2BoundableCollisions vtx2triCsns = 
         new Boundable2BoundableCollisions();
      
      // For each vertex-triangle pair, check if they're actually colliding
      // (narrow-phase).
      for (int i = 0; i < ixtVertexNodes.size (); i++) {
         SweptVertex sv = (SweptVertex)
            ixtVertexNodes.get (i).getElements ()[0];
         SweptTriangle st = (SweptTriangle)
            ixtFaceNodes.get (i).getElements ()[0];
         
         // Confirm collision (narrow-phase)
         CCRV ccrv = isVertexTriangleCollision (sv, st, smi0, smi1);
         if (ccrv.hitTime < 0) {   // No collision
            continue;
         }
        
         // Case 1: This is the vertex's first collision
         if (vtx2triCsns.isEmpty (sv)) {
            // Append triangle to vertex's list of collisions
            vtx2triCsns.addCollision (sv, st, ccrv);
         }
         else { 
            // Case 2: Vertex has existing collisions.  
            
            // Get one of the existing collisions. Note that all existing
            // collisions have equal hitTime.
            Pair<Boundable,CCRV> cloesetCsn = vtx2triCsns.getFirstCollision (sv);
            double closest_hitTime = cloesetCsn.second.hitTime;
            
            // If this new collisions has an equal hitTime (i.e. on-par), 
            // include it.
            if (Math.abs (ccrv.hitTime-closest_hitTime) < myTimeElipson) {
               vtx2triCsns.addCollision (sv, st, ccrv);
            }
            // Else if the new collision has a significantly earlier hitTime, 
            // clear the vertex's existing collisions, and set this 
            // new collision as the best contending collision.
            else if (ccrv.hitTime < closest_hitTime) {
               vtx2triCsns.getCollisions (sv).clear ();
               vtx2triCsns.addCollision (sv, st, ccrv);
            }
         }
      }
      
      // Create a penetrating point for each vertex-triangle collision.
      
      ArrayList<PenetratingPoint> pentPts = new ArrayList<PenetratingPoint>();

      // For each vertex
      for (Boundable sv : vtx2triCsns.getKeys ()) {
         // For each triangle collision
         for (Pair<Boundable,CCRV> csn : vtx2triCsns.getCollisions (sv)) {
            SweptTriangle st = (SweptTriangle)csn.first;
            CCRV ccrv = csn.second;
            
            PenetratingPoint pentPt = createPenetratingPoint(
               (SweptVertex)sv, st, ccrv, smi0, smi1);
            pentPts.add (pentPt);
         }
      }
      
      // Not necessary, but helps with debugging.
      Collections.sort(pentPts, new Comparator<PenetratingPoint>() {
         @Override
         public int compare(PenetratingPoint o1, PenetratingPoint o2) {
            return o1.vertex.getIndex() - o2.vertex.getIndex ();
         }
      });
      
      return pentPts;
   }
   
   protected ArrayList<EdgeEdgeContact> findEdgeEdgeContacts(
   SweptMeshInfo smi0, SweptMeshInfo smi1) {
      
      // For each edge, find all edge-edge collisions (broad-phase).
      
      ArrayList<BVNode> ixtEdgeNodes0 = new ArrayList<BVNode>();
      ArrayList<BVNode> ixtEdgeNodes1 = new ArrayList<BVNode>();
      
      smi0.myEdgeTree.setBvhToWorld ( smi0.myMesh.getMeshToWorld () );
      smi1.myEdgeTree.setBvhToWorld ( smi1.myMesh.getMeshToWorld () );
      
      smi0.myEdgeTree.intersectTree (ixtEdgeNodes0, ixtEdgeNodes1, 
         smi1.myEdgeTree);
      
      Boundable2BoundableCollisions e2eCsns = new Boundable2BoundableCollisions();
      
      // For each edge collision
      for (int i = 0; i < ixtEdgeNodes0.size(); i++) {

         SweptEdge se0 = (SweptEdge)ixtEdgeNodes0.get (i).getElements ()[0];
         SweptEdge se1 = (SweptEdge)ixtEdgeNodes1.get (i).getElements ()[0];
         
         // Confirm collision (narrow-phase)
         CCRV ccrv = isEdgeEdgeCollision(se0, se1, smi0, smi1);
         if (ccrv.hitTime < 0)
            continue;

         // Store the confirmed collision.
         
         // If this is the first confirmed collision
         if (e2eCsns.isEmpty (se0)) {
            e2eCsns.addCollision (se0, se1, ccrv);
         }
         // Otherwise, edge0 has existing collisions.
         else {   
            // Get an existing edge0 collision. Note that all of edge0's 
            // existing collisions have equal hitTime.
            Pair<Boundable,CCRV> cloesetCsn = e2eCsns.getFirstCollision (se0);
            double closest_hitTime = cloesetCsn.second.hitTime;
            
            // If this new collision has similar hitTime, include it as well.
            if (Math.abs(ccrv.hitTime-closest_hitTime) < myTimeElipson) {  
               e2eCsns.addCollision (se0, se1, ccrv);
            }
            // If this new collision has an earlier hitTime, set as replacement.
            else if (ccrv.hitTime < closest_hitTime) {
               e2eCsns.getCollisions (se0).clear ();
               e2eCsns.addCollision (se0, se1, ccrv);
            }
         }
      }
      
      // If an edge of the 2nd mesh has multiple collisions, only 
      // include the earliest collisions.
      
//      Boundable2BoundableCollisions inv_e2eCsns = e2eCsns.createInverseMapping ();
//      
//      for (Boundable se0i : inv_e2eCsns.getKeys ()) {
//         // For given edge of opposite mesh (i.e. mesh1) (se0i), find minimum collision time. 
//         double minHitTime = Double.POSITIVE_INFINITY;
//         for (Pair<Boundable,CCRV> p_se1i : inv_e2eCsns.getCollisions (se0i)) {
//            minHitTime = Math.min (minHitTime, p_se1i.second.hitTime);
//         }
//         
//         // Remove collisions in original e2eCsns involving se0i that have a greater hitTime.
//         
//         for (Pair<Boundable,CCRV> p_se1i : inv_e2eCsns.getCollisions (se0i)) {
//            Boundable se0 = p_se1i.first;
//            
//            final double final_minHitTime = minHitTime;
//            e2eCsns.getCollisions (se0).removeIf (p_se1 -> 
//               p_se1.second.hitTime > final_minHitTime + myTimeElipson
//            );
//         }
//      }
      
      // Create edge-edge contact for each collision 
      
      ArrayList<EdgeEdgeContact> eeCts = new ArrayList<EdgeEdgeContact>();
      
      // For each edge
      for (Boundable se0 : e2eCsns.getKeys ()) {
         // For each edge-edge collision
         for (Pair<Boundable,CCRV> csn : e2eCsns.getCollisions (se0)) {
            SweptEdge se1 = (SweptEdge)csn.first;
            CCRV ccrv = csn.second;
            
            EdgeEdgeContact eeCt = createEdgeEdgeContact(
               (SweptEdge)se0, se1, ccrv, smi0, smi1);
            eeCts.add (eeCt);
         }
      }
      
      return eeCts;
   }
   
   
   /* --- Helper Methods --- */
   
   
   public CCRV isVertexTriangleCollision(
   SweptVertex sv, SweptTriangle st, SweptMeshInfo sv_smi, SweptMeshInfo st_smi) {
      
      // Ignore collision checking if vertex is connected to the triangle.
      if (MeshUtil.isHasVertex (sv.myVertex, st.myFace)) {
         return new CCRV();
      }
      
      // Transform the vertex and triangle points into world-space
      
      Point3d[] svPtsW = sv.createTransformedPoints (
         sv_smi.myMesh.getMeshToWorld ());
      Point3d[] stPtsW = st.createTransformedPoints (
         st_smi.myMesh.getMeshToWorld ());
      
      // Execute narrow-phase collision detection algorithm
      
      CCRV ccrv = myContCol.collideVertexTrianglePnts (
         svPtsW[1], svPtsW[0],   // Vertex 
         stPtsW[3], stPtsW[0],   // Triangle vertex #0
         stPtsW[4], stPtsW[1],   // Triangle vertex #1
         stPtsW[5], stPtsW[2],   // Triangle vertex #2
         mySpaceElipson);

      return ccrv;
   }
   
   protected CCRV isEdgeEdgeCollision(
   SweptEdge se0, SweptEdge se1, SweptMeshInfo se0_smi, SweptMeshInfo se1_smi) {
      
      HalfEdge he0 = se0.myEdge;
      HalfEdge he1 = se1.myEdge;
      
      // Ignore collision checking both edges are the same or adjacent
      if (MeshUtil.isShareVertex (he0, he1)) {
         return new CCRV();
      }
      
      // Make a copy of the swept edge points before transforming them to
      // world-space.
      Point3d[] se0PtsW = se0.createTransformedPoints (
         se0_smi.myMesh.getMeshToWorld ());
      Point3d[] se1PtsW = se1.createTransformedPoints (
         se1_smi.myMesh.getMeshToWorld ());
      
      // Execute narrow-phase collision detection algorithm 
      return myContCol.collideEdgeEdgePnts (
         se0PtsW[2], se0PtsW[0],   // Edge0 head
         se0PtsW[3], se0PtsW[1],   // Edge0 tail
         se1PtsW[2], se1PtsW[0],   // Edge1 head
         se1PtsW[3], se1PtsW[1],   // Edge1 tail
         mySpaceElipson);
   }
   
   protected PenetratingPoint createPenetratingPoint(
   SweptVertex sv, SweptTriangle st, CCRV ccrv, SweptMeshInfo sv_smi, 
   SweptMeshInfo st_smi) {
      
      Point3d v_cur = new Point3d(sv.getPoint (0));
      Point3d v_prv = new Point3d(sv.getPoint (1));
      
      // Transform to world
      sv_smi.myMesh.transformToWorld (v_cur);
      sv_smi.myMesh.transformToWorld (v_prv);
      
      // Penetration vector (vector to undo the vertex penetration).
      // `v_cur to facePt_cur`
      
      st.X = st_smi.myMesh.getMeshToWorld ();
      Point3d[] facePts = st.computeInstanteousTriangle (1);
      Point3d facePt_cur = baryInto3d (facePts[0], facePts[1], facePts[2], ccrv.bary);
      
      Vector3d pentVec = new Vector3d();
      pentVec.sub (facePt_cur, v_cur);
      
      // Compute collision point
      
      Point3d csnPt = sv.computeInstanteousPoint (ccrv.hitTime);
      sv_smi.myMesh.transformToWorld (csnPt);;
      
      // Create PenetratingPoint
      
      PenetratingPoint pentPt = new PenetratingPoint(
         sv.myVertex, st.myFace, ccrv.bary, csnPt, 
         pentVec, null);
      
      pentPt.distance = pentVec.norm () * myDistScale;
      pentPt.hitTime = ccrv.hitTime;
      
      pentPt.normal = new Vector3d(st.myFace.getWorldNormal ());
      pentPt.normal.normalize ();
      
      // However, this assumes that pentVec direction is correct.
      if (pentPt.normal.dot (pentVec) < 0) {
         pentPt.normal.negate ();
         System.out.println ("Negated!");
      }
      
      // Print
      
      Vertex3d[] faceVtxs = st.myFace.getTriVertices ();
      String faceVtxStr = String.format ("[%d-%d-%d]", 
         faceVtxs[0].getIndex (), 
         faceVtxs[1].getIndex (),
         faceVtxs[2].getIndex ()
      );
      
      if (myDebug) {
         System.out.println ("Penetrating Point: " + 
            " V: " + sv.myVertex.getIndex () + 
            " T: " + st.myFace.vertexStr () + 
            " T (manual): " + faceVtxStr +
            " PentVec: " + pentVec.toString ("%.5f") + 
            " Normal: " + pentPt.normal.toString ("%.2f") + 
            " Dist: " + pentPt.distance + 
            " HitTime: " + ccrv.hitTime + 
            " FaceNrm: " + st.myFace.getWorldNormal () +  
            "\n"
         );
      }
      
//      Vector3d disp = new Vector3d().sub (cur, prv);
//      
//      Vector3d prv2Face = new Vector3d();
//      prv2Face.interpolate (ccrv.hitTime, disp);
//      
//      // Calculate `Point on face`
//      Point3d nearestFacePoint = new Point3d();
//      nearestFacePoint.add (prv, prv2Face);
//      
//      // Compute `Cur to Face`
//      Vector3d disp2nearestFacePoint = new Vector3d();  
      
//      // If vertex is idle, need to rely on face's displacement to find
//      // penetration distance.
//      if (sv.isSweepIdle (mySpaceElipson)) {
//         Point3d curFacePnt = baryInto3d (st.myPnts[0], st.myPnts[1], st.myPnts[2], ccrv.bary);
//         disp2nearestFacePoint.sub (curFacePnt, /*idle vertex=*/cur);
//      }
//      else {
//         disp2nearestFacePoint.sub(nearestFacePoint, cur);
//      }
//      
//        
//      PenetratingPoint pentPt = new PenetratingPoint(
//        sv.myVertex, st.myFace, ccrv.bary, nearestFacePoint, 
//        disp2nearestFacePoint, null);
//
//      pentPt.hitTime = ccrv.hitTime;
//      
//      // Compute normal
//      
//      pentPt.normal = st.computeInstanteousNormal (ccrv.hitTime);
//      st_smi.myMesh.transformToWorld (pentPt.normal);
//      
//      boolean isVtxHitFaceFront = (pentPt.normal.dot (disp2nearestFacePoint) > 0);
//      if (! isVtxHitFaceFront) {
//         pentPt.normal.negate ();
//      }
      
      return pentPt;
   }
   
   int neg = 0;
   protected EdgeEdgeContact createEdgeEdgeContact(SweptEdge se0, SweptEdge se1, 
      CCRV ccrv, SweptMeshInfo se0_smi, SweptMeshInfo se1_smi) {
      
      EdgeEdgeContact eeCt = new EdgeEdgeContact();
      eeCt.edge0 = se0.myEdge;
      eeCt.edge1 = se1.myEdge;
      
      // Compute penetration depth (i.e. vector to undo e0's intersection).

      se0.X = se0_smi.myMesh.getMeshToWorld ();
      se1.X = se1_smi.myMesh.getMeshToWorld ();
      
      Point3d e0_curPt = se0.computeInstanteousEdgePoint (1, ccrv.r);
      Point3d e1_curPt = se1.computeInstanteousEdgePoint (1, ccrv.s);
      
      Vector3d pentVec = new Vector3d();
      pentVec.sub (e1_curPt, e0_curPt);   // 1/2 to undo penetration
      
      // Compute cross product between the 2 edges.

      Vector3d e01cross = new Vector3d();
      e01cross.cross (
         se0.computeInstanteousEdgeVec (0), 
         se1.computeInstanteousEdgeVec (0));
      e01cross.normalize ();
      
      // Cross product (e0 csn resolution) must be same direction as pentVec
      if (e01cross.dot (pentVec) < 0)
         e01cross.negate ();
      
      eeCt.point0 = se0.computeInstanteousEdgePoint (ccrv.hitTime, ccrv.r);
      eeCt.point1 = se1.computeInstanteousEdgePoint (ccrv.hitTime, ccrv.s);
      
      eeCt.displacement = pentVec.norm () * myDistScale;
      eeCt.point1ToPoint0Normal = e01cross;
      
      if (myDebug)
         System.out.println ("Edge-Edge Contact: " + 
            " E0: " + eeCt.edge0.vertexStr () + 
            " E1: " + eeCt.edge1.vertexStr () + 
            " PentVec: " + pentVec.toString ("%.4f") + 
            " e01cross: " + e01cross.toString ("%.4f") + 
            " disp: " + pentVec.norm () + 
            "\n"
         );     

//      // Compute cross product between the 2 edges 
//
//      Vector3d e01cross = new Vector3d();
//      e01cross.cross (
//         se0.computeInstanteousEdgeVec (ccrv.hitTime), 
//         se1.computeInstanteousEdgeVec (ccrv.hitTime));
//      e01cross.normalize ();
//      
//      // Direction of cross product is unreliable when edges are intersecting.
//      // So, rely on the velocity vector of the two moving edges.
//  
//      Vector3d e0velo = se0.computeEdgeVelocity (ccrv.r);
//      Vector3d e1velo = se1.computeEdgeVelocity (ccrv.s);
//      
//      boolean is_e01velo_sameDir = (e0velo.dot (e1velo) > 0);
//      
//      // Case 1: se1 is idle
//      if (se1.isSweepIdle (mySpaceElipson)) {
//         // Collision response direction of e0 should be opposite of its velocity.
//         if (e01cross.dot (e0velo) > 0)
//            e01cross.negate ();
//      }
//      // Case 2: se0 and se1 have opposite velocity directions, or se0 idle
//      else if (se0.isSweepIdle (mySpaceElipson) || ! is_e01velo_sameDir) {
//         // Collision response direction of e0 should be same as e1's velocity
//         if (e01cross.dot (e1velo) < 0) 
//            e01cross.negate ();
//      }
//      // Case 3: se0 and se1 have same velocity directions
//      else { // (is_e01velo_sameDir) 
//         // Collision response direction of e0 should be same as e0/e1's velocity
//         if (e01cross.dot (e0velo) < 0) 
//            e01cross.negate ();
//      }
      
      
      
      
      // DANCOLEDIT
      // TODO: If compute collision points at t=hitTime, the distance between 
      // them will be very close to 0. Consequently, vector between the 2 points
      // may be zero as a result, which makes it difficult to find the 
      // displacement between the two collision points.
      // Try t=0 or t=1 instead of t=hitTime if the collision response direction
      // looks wrong.
      
//    boolean is_se1_idle = se1.isSweepIdle (mySpaceElipson);
//    
//    double pnt0_hitTime = (is_se1_idle) ? 1 : ccrv.hitTime; 
//    double pnt1_hitTime = (is_se1_idle) ? ccrv.hitTime : 1;    
//    
//    Vector3d edge0_dir_th = new Vector3d();
//    Vector3d edge1_dir_th = new Vector3d();
//    
//    eeCt.point0 = computeClosestEdgePoint(se0, pnt0_hitTime, ccrv.r,
//       se0_smi.myMesh.getMeshToWorld (), /*out=*/edge0_dir_th);
//    eeCt.point1 = computeClosestEdgePoint(se1, pnt1_hitTime, ccrv.s, 
//       se1_smi.myMesh.getMeshToWorld (), /*out=*/edge1_dir_th);
//    
//      // Cross-product of edge's direction vectors.
//      eeCt.point1ToPoint0Normal.cross( edge0_dir_th, edge1_dir_th );
//      eeCt.point1ToPoint0Normal.normalize ();
//      
//      // Incident vector.
//      eeCt.w.sub( eeCt.point0, eeCt.point1 );
//      
//      // This will be very small (less than 1e-6).
//      eeCt.displacement = eeCt.w.norm ();
//      
//      // Supposedly prevent precision problems when point0 and point1 are close,
//      // according to EdgeEdgeContact.calculate().
//      if (eeCt.w.dot (eeCt.point1ToPoint0Normal) < 0) {
//         eeCt.point1ToPoint0Normal.negate();
//      }
//      
//      eeCt.point1ToPoint0Normal.negate ();
      
      eeCt.s0 = ccrv.r;
      eeCt.s1 = ccrv.s;
      
      eeCt.hitTime = ccrv.hitTime;
      
      return eeCt;
   }
   
   /**
    * Compute point on given edge that's closest to the other edge at t=hitTime.
    * 
    * @param se
    * Given edge. Contains info about edge at t=0 and t=1.
    * 
    * @param hitTime
    * Time of collision. Anywhere from t=0 to t=1.
    * 
    * @param s
    * Between tail to head of given edge, which point is closest to the other
    * edge at t=hitTime? Anywhere from 0 (i.e. tail) to 1 (i.e. head).
    * 
    * @param worldX
    * Matrix to transform edge into world-space.
    * 
    * @param out_edgeDir_th 
    * This vector will be populated. Contains edge direction at t=hitTime.
    */
   protected Point3d computeClosestEdgePoint(SweptEdge se, double hitTime, 
   double s, RigidTransform3d worldX, Vector3d out_edgeDir_th) {
      
      // Transform swept edge points into world-space
      Point3d[] sePtsW = se.createTransformedPoints (worldX);

      Point3d head_t1 = sePtsW[0];
      Point3d tail_t1 = sePtsW[1];
      Point3d head_t0 = sePtsW[2];
      Point3d tail_t0 = sePtsW[3];
      
      // Point of head at t=hitTime 
      Point3d head_th = new Point3d();
      head_th.interpolate (head_t0, hitTime, head_t1);
      
      // Point of tail at t=hitTime 
      Point3d tail_th = new Point3d();
      tail_th.interpolate (tail_t0, hitTime, tail_t1);
      
      // Edge as vector at t=hitTime
      Vector3d vec_th = new Vector3d().sub (head_th, tail_th);
      
      Point3d closestPt_th = new Point3d();
      closestPt_th.interpolate (s, vec_th);
      closestPt_th.add (tail_th);
      
      out_edgeDir_th.set(vec_th);
      return closestPt_th;
   }
   
   /**
    * Print out all the qualified collisions.
    */
   protected void printCollisions(ContactInfo cinfo) {
      for (EdgeEdgeContact eeCt : cinfo.getEdgeEdgeContacts ())
         System.out.printf ("  Edge-Edge Collision: %s-%s. Disp: %.4f, "+
           "HitTime: %.4f\n", eeCt.edge0.vertexStr (), eeCt.edge1.vertexStr (),
           eeCt.displacement, eeCt.hitTime);
      
      for (int m=0; m<2; m++)
         for (PenetratingPoint pentPt : cinfo.getPenetratingPoints (m)) 
            System.out.printf ("  Vtx-Tri Collision: [ %d ]-%s. Disp: %.4f, "+
              "HitTime: %.4f\n", pentPt.vertex.getIndex (),
              pentPt.face.vertexStr (), pentPt.distance, pentPt.hitTime);
   }
   
   /**
    * Given a colliding edge, does one of its vertex have an earlier
    * collision time?
    * 
    * @param pentPts
    * List of all the vertices that collided.
    * 
    * @param edge
    * Colliding edge.
    * 
    * @param edge_hitTime
    * Timestep fraction when the edge collided.
    */
   protected boolean isExistEarlierVtxTriContact(
   ArrayList<PenetratingPoint> pentPts, HalfEdge edge, double edge_hitTime) {
      for (PenetratingPoint pentPt : pentPts) {
         boolean isEdgeVtxConn = MeshUtil.isEdgeAndVtxConnected (
            edge, pentPt.vertex);
         double vtxTri_hitTime = pentPt.hitTime;
         
         if (isEdgeVtxConn && vtxTri_hitTime < edge_hitTime - myTimeElipson) {
            if (myDebug)
               System.out.printf ("Skipping edge [%d-%d] b/c pentPt [%d] has "+
                 "earlier hitTime.\n", edge.head.getIndex (),
                 edge.tail.getIndex (), pentPt.vertex.getIndex ());
            
            return true;
         }
      }
      
      return false;
   }
   
   /**
    * Given a colliding vertex, does one of its connected edges have an
    * earlier collision time?
    * 
    * @param m
    * Mesh0 or Mesh1?
    * 
    * @param eeCts
    * List of all edge-edge collisions.
    * 
    * @param pentVtx
    * Colliding vertex.
    * 
    * @param pentVtx_hitTime
    * Timestep fraction when the vertex collided.
    */
   protected boolean isExistEarlierEdgeEdgeContact(int m,
   ArrayList<EdgeEdgeContact> eeCts, Vertex3d pentVtx, double pentVtx_hitTime) {
      for (EdgeEdgeContact eeCt : eeCts) {
         boolean isEdgeVtxConn = (m == 0) ?
            MeshUtil.isEdgeAndVtxConnected (eeCt.edge0, pentVtx) :
            MeshUtil.isEdgeAndVtxConnected (eeCt.edge1, pentVtx);

         double edgeEdge_hitTime = eeCt.hitTime;
         
         if (isEdgeVtxConn && 
             edgeEdge_hitTime < pentVtx_hitTime - myTimeElipson) 
         { 
            if (myDebug)
               System.out.printf ("Skipping pentPt [%d] b/c edge-edge "+
                 "[%d-%d][%d-%d] has earlier hitTime.\n",
                 pentVtx.getIndex (), 
                 eeCt.edge0.head.getIndex (), eeCt.edge0.tail.getIndex (), 
                 eeCt.edge1.head.getIndex (), eeCt.edge1.tail.getIndex ());
            
            return true;
         }
      }
      
      return false;
   }
   
   /**
    * Does an inverse exist for the given edge-edge-contact?
    */
   protected boolean isExistingInverseEdgeEdgeContact(
   EdgeEdgeContact eeCt_A, ArrayList<EdgeEdgeContact> eeCts_B) {
      for (EdgeEdgeContact eeCt_B : eeCts_B) {
         if (eeCt_A.edge0 == eeCt_B.edge1 && eeCt_A.edge1 == eeCt_B.edge0) 
         {
            return true;
         }
      }
      return false;
   }
   
   protected boolean isEdge0HeadGreaterThanEdge1Head(EdgeEdgeContact eeCt) {
      return (eeCt.edge0.head.getIndex () > eeCt.edge1.head.getIndex ());
   }
   
   protected Point3d baryInto3d(Point3d a, Point3d b, Point3d c, Vector2d bary) {
      Point3d pnt3d = new Point3d();
      
      pnt3d.scaledAdd (1-bary.x-bary.y, a);
      pnt3d.scaledAdd (bary.x,          b);
      pnt3d.scaledAdd (bary.y,          c);
      
      return pnt3d;
   }
   
   
   /* --- Data Classes --- */
   
   /**
    * For each vertex or edge, keep an organized list of its collisions 
    * (e.g. vertex-triangle or edge-edge).
    */
   public class Boundable2BoundableCollisions {
      public LinkedHashMap<Boundable, ArrayList<Pair<Boundable,CCRV>>> 
      myBnd2BndCsns;
     
      public Boundable2BoundableCollisions() {
         myBnd2BndCsns = 
            new LinkedHashMap<Boundable, ArrayList<Pair<Boundable,CCRV>>>();
      }
      
      public void addCollision(Boundable b0, Boundable b1, CCRV ccrv) {
         // Create collision entry
         Pair<Boundable, CCRV> csn = new Pair<Boundable, CCRV>(b1, ccrv);
         
         // Get list of existing collision entries
         ArrayList<Pair<Boundable,CCRV>> csns = myBnd2BndCsns.get (b0);
         
         // If no existing collisions, create a new array of collisions
         // for the vertex.
         if (csns == null) {
            csns = new ArrayList<Pair<Boundable,CCRV>>();
            myBnd2BndCsns.put (b0, csns);
         }
         csns.add (csn);
      }
      
      public ArrayList<Pair<Boundable,CCRV>> getCollisions(Boundable b0) {
         ArrayList<Pair<Boundable,CCRV>> csns = myBnd2BndCsns.get (b0);
         if (csns == null) {
            // Create new list of existing collisions entries, which will 
            // be empty.
            csns = new ArrayList<Pair<Boundable,CCRV>>();
            myBnd2BndCsns.put (b0, csns);
         }
         return csns;
      }
      
      public Pair<Boundable,CCRV> getFirstCollision(Boundable b0) {
         ArrayList<Pair<Boundable,CCRV>> csns = myBnd2BndCsns.get (b0);
         if (csns == null || csns.isEmpty()) {
            return null;
         }
            
         return csns.get (0);
      }
      
      public boolean isEmpty(Boundable b0) {
         ArrayList<Pair<Boundable,CCRV>> csns = myBnd2BndCsns.get (b0);
         return (csns == null || csns.isEmpty ());
      }
      
      public Set<Boundable> getKeys() {
         return myBnd2BndCsns.keySet ();
      }
      
      public Collection<ArrayList<Pair<Boundable,CCRV>>> getValues() {
         return myBnd2BndCsns.values ();
      }
      
      public Boundable2BoundableCollisions createInverseMapping() {
         Boundable2BoundableCollisions invB2BCsns = 
            new Boundable2BoundableCollisions ();
         
         for (Boundable se0 : myBnd2BndCsns.keySet ()) {
            ArrayList<Pair<Boundable, CCRV>> se1s = myBnd2BndCsns.get (se0);
            
            for (Pair<Boundable, CCRV> se1 : se1s) {
               invB2BCsns.addCollision (se1.first, se0, se1.second);
            }
         }
         
         return invB2BCsns;
      }
   }
}
