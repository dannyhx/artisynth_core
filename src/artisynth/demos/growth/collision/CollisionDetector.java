package artisynth.demos.growth.collision;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.Set;

import artisynth.core.femmodels.BackNode3d;
import artisynth.core.femmodels.FemMeshComp;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.mechmodels.CollidableBody;
import artisynth.core.mechmodels.RigidBody;
import artisynth.demos.growth.util.MathUtil;
import artisynth.demos.growth.util.MeshUtil;
import maspack.collision.ContactInfo;
import maspack.collision.EdgeEdgeContact;
import maspack.collision.PenetratingPoint;
import maspack.geometry.BVNode;
import maspack.geometry.Boundable;
import maspack.geometry.HalfEdge;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;
import maspack.util.Pair;

/**
 * Detects nearby (discrete) and colliding (continuous) features. 
 * 
 * DEV NOTES:
 *    Artisynth AABB tree runs faster if bvhWorld set to Identity, which is 
 *    ok for FEM models.
 *      - Currently using intersectTreeWorldIdentity() 
 */
public class CollisionDetector {

   public static boolean myDebug = false;
   
   /** Tolerance when comparing collision times. */
   public static double myTimeElipson = 1e-8;
   
   /** Distance tolerance when determining collisions. */
   public static double mySpaceElipson = 1e-10;

   /** Minimum distance that nearby features should be repelled apart. */
   public static double myClothThickness = 1e-2;   // Should correspond to penetrationTol
   
   /** Boost repulsion of nearby features, if needed. */
   public static double myImminentImpulseScale = 1.00;   // 0.25
   
   /** Assumed time step. */
   public static double myTimestep = 0.01;
   
   /** Space-time path of each collidable body. */
   public LinkedHashMap<CollidableBody,SweptMeshInfo> myBody2SweptMeshInfo;
   
   /** Utility to perform narrow-phase continuous collision detection. */
   protected NarrowPhaseCCD myContCol;
   
   // Debugging only. The FEM models that are currently under examination.  
   protected FemMeshComp mFmc0;
   protected FemMeshComp mFmc1;

   /** Detect nearby (NEARBY) or colliding (ACTUAL) features? 
    *  (ACTUAL_ZONED) is the same as (ACTUAL) except impact zones are used. */
   public static Stage mStage = Stage.ACTUAL;
   public enum Stage {NEARBY, ACTUAL, ACTUAL_ZONED};
   
   /** Legacy. For forcing backstepping. 0.00 = turned off. Not useful. */
   public static double mHitTimeBacktrack = 0.00;

   /** Max number of iterations to perform continuous detection and resolution 
    *  in order to eliminate secondary collisions. */
   public static int myMaxNumActualIters = 10;
   
   /** Minimum distance (as a negative) that colliding features should be kept 
    *  apart during their resolution. */
   public static double myImpactZonePenetrationTol = -1e-2;
   
   /** Enable nearby feature detection? */
   public static boolean myEnableProximityDetection = true;
   
   /** Enable colliding feature detection? */
   public static boolean myEnableContinuousDetection = true;
   
   /** Enable colliding feature detection using impact zones? */
   public static boolean myEnableImpactZoneDetection = true;
   
   
   
   /**
    * Maintains a reference to the unilaterals that were used to undo
    * the penetration points. These unilaterals are adjusted during remeshing
    * before they're applied to the subsequent constraint backward Euler 
    * solve.
    */
   public static ContactConstraintAgg myCCAgg = null;
   
   /** Visualize detected contacts. Debugging purpose. */
   public static ContinuousRenderable mContRend;
   
   public CollisionDetector() {
      myBody2SweptMeshInfo = new LinkedHashMap<CollidableBody,SweptMeshInfo>();
      myContCol = new NarrowPhaseCCD();
   }
   
   
   /* --- Setters and Getters --- */
   
   /** Get a mesh's space-time path, across the latest time step. */
   public SweptMeshInfo getSweptMeshInfo(CollidableBody body) {
      return myBody2SweptMeshInfo.get (body);
   }
   
   
   /* --- Primary Functions (public) --- */
   
   /** 
    * Clear the space-time path of each mesh.
    * 
    * Call this whenever a simulation is re-initialized.
    */
   public void clearSweptMeshInfo() {
      this.myBody2SweptMeshInfo.clear ();
   }
   
   /** 
    * Update the previous vertex positions to the current vertex positions. 
    * 
    * Call this at the beginning of each physics step.
    */
   public void copyCurrent2PreviousPositions() {
      for (SweptMeshInfo smi : myBody2SweptMeshInfo.values ()) {
         smi.copyCurrent2PreviousPositions ();
      }
   }
   
   /**
    * Search for nearby or colliding features between two bodies.
    * 
    * @return
    * Info regarding each detected collision.
    */
   public ContactInfo getContacts(CollidableBody body0, CollidableBody body1) {
      for (CollidableBody body : new CollidableBody[] {body0,body1} ) {
         SweptMeshInfo smi = myBody2SweptMeshInfo.get (body);
         
         if (smi == null) {
            smi = new SweptMeshInfo(body.getCollisionMesh ());
            myBody2SweptMeshInfo.put (body, smi);
         }
         
         smi.updatePrevPositionsX0 ();
         
         if (mStage == Stage.NEARBY) {
//            smi.computeAvgVelocities (myTimestep);
            smi.saveCurrentPositions ();
            
            // Previous and current positions are set to be equal.
            // Essentially AABB will only enclose the mesh features at t=0
            // (i.e. previous time step), allowing us to search for nearby
            // features at t=0.
            smi.copyPrevious2CurrentPositions ();
         }
         
         smi.updateBVTrees ();
         
         if (body0 == body1) break;
      }
      
      SweptMeshInfo smi0 = myBody2SweptMeshInfo.get (body0);
      SweptMeshInfo smi1 = myBody2SweptMeshInfo.get (body1);
      
      mFmc0 = (body0 instanceof FemMeshComp) ? (FemMeshComp) body0 : null;
      mFmc1 = (body1 instanceof FemMeshComp) ? (FemMeshComp) body1 : null;
      
      boolean isDynamic0 = isBodyDynamic(body0);
      boolean isDynamic1 = isBodyDynamic(body1);
      
      refreshSurfaceMeshes(body0);
      refreshSurfaceMeshes(body1);
      
      if (myDebug)
         System.out.printf ("Computing contact between %s-%s\n", 
            body0.getName (), body1.getName ());
      
      ContactInfo cinfo = getContacts (smi0, smi1, isDynamic0, isDynamic1);
      
      for (SweptMeshInfo smi : new SweptMeshInfo[] {smi0,smi1} ) {
         if (mStage == Stage.NEARBY) {
            smi.loadCurrentPositions ();
         }
         
         if (smi0 == smi1) break;
      }

      return cinfo;
   }
   
   /**
    * Refresh the AABB of the mesh features for the given FEM model.
    * 
    * Call this after remeshing the FEM model.
    */
   public void rebuildSweptMeshInfo(FemMeshComp comp) {
      FemModel3d femModel = (FemModel3d)comp.getParent ().getParent (); 

      SweptMeshInfo smi = myBody2SweptMeshInfo.get (comp);      
      smi.build (femModel.getSurfaceMesh ());
      
      // Note that we have to use getSurfaceMesh() instead of 
      // comp.getCollidableMesh() b/c getSurfaceMesh() will ensure that the 
      // returned mesh is updated --- which is needed after remeshing.
   }
   
   
   /* --- Secondary Methods (protected) --- */
 
   
   protected boolean isBodyDynamic(CollidableBody body) {
      if (body instanceof RigidBody) {
         RigidBody rb = (RigidBody)body;
         return rb.isDynamic ();
      }
      else if (body instanceof FemMeshComp) {
         FemMeshComp femMeshComp = (FemMeshComp)body; 
         FemModel3d femModel = (FemModel3d)femMeshComp.getParent ().getParent (); 

         return femModel.getDynamicsEnabled ();
      }
      else {
         throw new UnsupportedOperationException("Unsupported body instance.");
      }
   }
   
   /**
    * Ensure surface mesh is synced with the nodal positions.
    */
   protected void refreshSurfaceMeshes(CollidableBody body) {
      if (body instanceof FemMeshComp) {
         FemMeshComp femMeshComp = (FemMeshComp)body; 
         FemModel3d femModel = (FemModel3d)femMeshComp.getParent ().getParent (); 
         
         femModel.updatePosState ();
      }
   }
   
   /**
    * Perform collision detection between two given meshes. 
    * 
    * Whether nearby or colliding collisions features are detected depends on 
    * {@link #mStage}.
    * 
    * @param smi0
    * Space-time path of mesh0. 
    * 
    * @param smi1
    * Space-time path of mesh1.
    * 
    * @param isDynamic0
    * Is mesh0 dynamic?
    * 
    * @param isDynamic1
    * Is mesh1 dynamic?
    * 
    * @return
    * Info regarding any collisions detected.
    */
   protected ContactInfo getContacts (SweptMeshInfo smi0, SweptMeshInfo smi1, 
   boolean isDynamic0, boolean isDynamic1) {
      
      if (! isDynamic0 && ! isDynamic1) 
         return null;
      
      ContactInfo cinfo = new ContactInfo(smi0.myMesh, smi1.myMesh);
      
      if (myDebug)
         System.out.println ("Computing penetration points for m=0"); 
      ArrayList<PenetratingPoint> pentPts0 = findPenetratingPoints(smi0, smi1);
      
      if (myDebug)
         System.out.println ("Computing penetration points for m=1"); 
      ArrayList<PenetratingPoint> pentPts1 = (smi0 != smi1) ? 
         findPenetratingPoints(smi1, smi0) : new ArrayList<PenetratingPoint>();
       
      cinfo.setPenetratingPoints (pentPts0, 0);
      cinfo.setPenetratingPoints (pentPts1, 1);    

      if (myDebug)
         System.out.println ("Computing edge-edge contacts for m=0 vs m=1");
      ArrayList<EdgeEdgeContact> eeCts = findEdgeEdgeContacts (smi0, smi1);
      cinfo.setEdgeEdgeContacts (eeCts);

      if (cinfo.getPenetratingPoints(0).isEmpty () &&
          cinfo.getPenetratingPoints(1).isEmpty () &&
          cinfo.getEdgeEdgeContacts().isEmpty ())
      {
         return null;
      }
         
      // This step is important for imminent collisions in particular, to 
      // severely reduce the number of constraints.
      removeRedundantCollisions(cinfo);
      
      if (myDebug) {
         printCollisions(cinfo);
         mContRend = new ContinuousRenderable (cinfo);
      }
         
      return cinfo;
   }
   
   /* --- Tertiary Methods --- (protected) */
   
   
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
      // e.g.  [10,20]-[30,40] and [30,40]-[10,20], where the numbers correspond
      //       to vertex indices.
      cinfo.getEdgeEdgeContacts ().removeIf (eeCt -> 
         isExistingInverseEdgeEdgeContact (eeCt, cinfo.getEdgeEdgeContacts ()) &&
         isEdge0HeadGreaterThanEdge1Head (eeCt)  // Prevent removing both
      );
   }
   
   /** Search for nearby or colliding vertex-triangle between two meshes. 
    * 
    * @param smi0
    * Space-time path of mesh0. 
    * 
    * @param smi1
    * Space-time path of mesh1. 
    * 
    * @return 
    * Detected vertex-triangle collisions.
    */
   protected ArrayList<PenetratingPoint> findPenetratingPoints(
   SweptMeshInfo smi0, SweptMeshInfo smi1) {
      
      // For each vertex, find set of colliding triangles (broad-phase).
      
      ArrayList<BVNode> ixtVertexNodes = new ArrayList<BVNode>();
      ArrayList<BVNode> ixtFaceNodes = new ArrayList<BVNode>();
      
      smi0.myVertexTree.setBvhToWorld ( smi0.myMesh.getMeshToWorld () );
      smi1.myTriangleTree.setBvhToWorld ( smi1.myMesh.getMeshToWorld () );

      smi0.myVertexTree.intersectTreeWorldIdentity (ixtVertexNodes, ixtFaceNodes, 
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
         sv.X = smi0.myMesh.getMeshToWorld ();
         st.X = smi1.myMesh.getMeshToWorld ();
         CCRV ccrv = isCollision (sv, st);
         if (ccrv.hitTime < 0) {   // No collision
            continue;
         }
        
         if (mStage == Stage.NEARBY) {
            vtx2triCsns.addCollision (sv, st, ccrv);
         }
         // Case 1: This is the vertex's first collision
         else if (vtx2triCsns.isEmpty (sv)) {
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
            
            PenetratingPoint pentPt = (mStage == Stage.NEARBY) ?
               createImminentPenetratingPoint((SweptVertex)sv, st, ccrv) :
               createPenetratingPoint((SweptVertex)sv, st, ccrv);
            
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
   
   /** Search for nearby or colliding edge-edge between two meshes.
    * 
    * @param smi0
    * Space-time path of mesh0. 
    * 
    * @param smi1
    * Space-time path of mesh1. 
    * 
    * @return 
    * Detected edge-edge collisions.
    */
   protected ArrayList<EdgeEdgeContact> findEdgeEdgeContacts(
   SweptMeshInfo smi0, SweptMeshInfo smi1) {
      
      // For each edge, find all edge-edge collisions (broad-phase).
      
      ArrayList<BVNode> ixtEdgeNodes0 = new ArrayList<BVNode>();
      ArrayList<BVNode> ixtEdgeNodes1 = new ArrayList<BVNode>();
      
      smi0.myEdgeTree.setBvhToWorld ( smi0.myMesh.getMeshToWorld () );
      smi1.myEdgeTree.setBvhToWorld ( smi1.myMesh.getMeshToWorld () );
      
      double startMs = System.currentTimeMillis ();
      if (myDebug)
         System.out.printf ("Finding potential edge-edge intersections...");
      smi0.myEdgeTree.intersectTreeWorldIdentity (ixtEdgeNodes0, ixtEdgeNodes1, 
         smi1.myEdgeTree);
      double endMs = System.currentTimeMillis ();
      if (myDebug)
         System.out.printf ("Count: [%d], Time: [%.2f sec]\n", 
            ixtEdgeNodes0.size (), (endMs-startMs)/1000);
      
      Boundable2BoundableCollisions e2eCsns = new Boundable2BoundableCollisions();
      
      // For each edge collision
      for (int i = 0; i < ixtEdgeNodes0.size(); i++) {

         SweptEdge se0 = (SweptEdge)ixtEdgeNodes0.get (i).getElements ()[0];
         SweptEdge se1 = (SweptEdge)ixtEdgeNodes1.get (i).getElements ()[0];
         
         // Confirm collision (narrow-phase)
         se0.X = smi0.myMesh.getMeshToWorld ();
         se1.X = smi1.myMesh.getMeshToWorld ();
         CCRV ccrv = isCollision(se0, se1);
         if (ccrv.hitTime < 0)
            continue;

         // Store the confirmed collision.
         
         if (mStage == Stage.NEARBY) {
            e2eCsns.addCollision (se0, se1, ccrv);
         }
         // If this is the first confirmed collision
         else if (e2eCsns.isEmpty (se0)) {
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
      
      // Create edge-edge contact for each collision 
      
      ArrayList<EdgeEdgeContact> eeCts = new ArrayList<EdgeEdgeContact>();
      
      // For each edge
      for (Boundable se0 : e2eCsns.getKeys ()) {
         // For each edge-edge collision
         for (Pair<Boundable,CCRV> csn : e2eCsns.getCollisions (se0)) {
            SweptEdge se1 = (SweptEdge)csn.first;
            CCRV ccrv = csn.second;
            
            EdgeEdgeContact eeCt = (mStage == Stage.NEARBY) ?
               createImminentEdgeEdgeContact((SweptEdge)se0, se1, ccrv) : 
               createEdgeEdgeContact((SweptEdge)se0, se1, ccrv);
            eeCts.add (eeCt);
         }
      }
      
      return eeCts;
   }
   
   
   /* --- Helper Methods --- */
   
   /** 
    * Measure to see if a vertex and triangle are nearby or colliding.
    *
    * @param sv 
    * Space-time path of the vertex. 
    * 
    * @param st 
    * Space-time path of the triangle. 
    * 
    * @return 
    * Details regarding the collision, if any.
    */
   public CCRV isCollision(SweptVertex sv, SweptTriangle st) {
      
      // Ignore collision checking if vertex is connected to the triangle.
      if (MeshUtil.isHasVertex (sv.myVertex, st.myFace)) {
         return new CCRV();
      }
      
      // Execute narrow-phase collision detection algorithm
   
      CCRV ccrv = null;
      
      if (mStage == Stage.NEARBY) {
         ccrv = isCollisionImminent(sv, st);
      }
      else {
         // Transform the vertex and triangle points into world-space
         Point3d[] svPtsW = sv.createTransformedPoints ();
         Point3d[] stPtsW = st.createTransformedPoints ();
         
         ccrv = myContCol.collideVertexTrianglePnts (
            svPtsW[1], svPtsW[0],   // Vertex 
            stPtsW[3], stPtsW[0],   // Triangle vertex #0
            stPtsW[4], stPtsW[1],   // Triangle vertex #1
            stPtsW[5], stPtsW[2],   // Triangle vertex #2
            mySpaceElipson);
      }

      return ccrv;
   }
   
   /** 
    * Measure to see if two edges are nearby or colliding.
    *
    * @param se0
    * Space-time path of edge0.
    * 
    * @param se1
    * Space-time path of edge1.
    * 
    * @return 
    * Details regarding the collision, if any.
    */
   protected CCRV isCollision(SweptEdge se0, SweptEdge se1) {
      
      HalfEdge he0 = se0.myEdge;
      HalfEdge he1 = se1.myEdge;
      
      // Ignore collision checking both edges are the same or adjacent
      if (MeshUtil.isShareVertex (he0, he1)) {
         return new CCRV();
      }
      
      // Make a copy of the swept edge points before transforming them to
      // world-space.
      Point3d[] se0PtsW = se0.createTransformedPoints ();
      Point3d[] se1PtsW = se1.createTransformedPoints ();
      
      CCRV rv = null;
      
      if (mStage == Stage.NEARBY) {
         rv = isCollisionImminent(se0, se1);
      }
      else {
         // Execute narrow-phase collision detection algorithm 
         rv = myContCol.collideEdgeEdgePnts (
            se0PtsW[se0.H0], se0PtsW[se0.H1],   // Edge0 head
            se0PtsW[se0.T0], se0PtsW[se0.T1],   // Edge0 tail
            se1PtsW[se1.H0], se1PtsW[se1.H1],   // Edge1 head
            se1PtsW[se1.T0], se1PtsW[se1.T1],   // Edge1 tail
            mySpaceElipson);
      }
      
      return rv;
   }
   
   /** 
    * Refine a colliding vertex-triangle info into Artisynth's 
    * PenetratingPoint class in order to hook up with its LCP solver.
    */
   protected PenetratingPoint createPenetratingPoint(
   SweptVertex sv, SweptTriangle st, CCRV ccrv) {
      
      // Compute vertex instantaneous positions
      
      Point3d v_prv = sv.computeInstanteousPoint (0);
      Point3d v_hit = sv.computeInstanteousPoint (ccrv.hitTime);
      Point3d v_cur = sv.computeInstanteousPoint (1);

      // Compute face point instantaneous positions

      Point3d[] fPts = st.computeInstanteousTriangle (0);
      Point3d fPt_prv = MathUtil.projectPoint2plane (v_prv, fPts[0], fPts[1], fPts[2]);
      
      fPts = st.computeInstanteousTriangle (1);
      Point3d fPt_cur = MathUtil.projectPoint2plane (v_cur, fPts[0], fPts[1], fPts[2]);
      
      // Create PenetratingPoint
      
      Vector3d v2f_cur = new Vector3d().sub (fPt_cur, v_cur);
      
      PenetratingPoint pentPt = new PenetratingPoint(
         sv.myVertex, st.myFace, ccrv.bary, v_hit, v2f_cur, null);
      
      pentPt.distance = v2f_cur.norm ();
      
      // Compute normal (direction that vertex should bounce from face).
      
      Vector3d f2v_prv = new Vector3d().sub (v_prv, fPt_prv);
      Vector3d f2v_cur = new Vector3d().sub (v_cur, fPt_cur);
      
      st.myFace.computeNormal ();
      pentPt.normal = new Vector3d(st.myFace.getWorldNormal ());
      
      double nrm_dot_f2v_prv = pentPt.normal.dot (f2v_prv);
      double nrm_dot_f2v_cur = pentPt.normal.dot (f2v_cur);
      
      if (nrm_dot_f2v_prv < nrm_dot_f2v_cur) {
         pentPt.normal.negate ();
      }
      
      // NEW: Edit 0 to minHitTimeBacktrack is 2nd arg
      double hitTime = Math.max (ccrv.hitTime-mHitTimeBacktrack, 0);
      pentPt.hitTime = hitTime;
      
      pentPt.vPnt_justBefore_hitTime = sv.computeInstanteousPoint (hitTime);
      pentPt.tPnts_justBefore_hitTime = st.computeInstanteousTriangle (hitTime);
      
      // EXP

//      computeVertexTriangleCollision_normalAndDist (sv, st, 
//         ccrv, pentPt.normal);
      
      if (myDebug) {
         System.out.println ("Penetrating Point: " + 
            " V: " + sv.myVertex.getIndex () + 
            " T: " + st.myFace.vertexStr () + 
            " FaceNrm: " + st.myFace.getWorldNormal ().toString ("%.2f") + 
            " Normal: " + pentPt.normal.toString ("%.2f") + 
            " Dist: " + String.format ("%.4f", pentPt.distance) + 
            " HitTime: " + String.format ("%.4f", pentPt.hitTime) + 
            " \n  " +
            " f2v_prv: " + f2v_prv.toString ("%.8f") + 
            " f2v_cur: " + f2v_cur.toString ("%.8f") + 
            " nrm_dot_f2v_prv: " + String.format ("%.8f", nrm_dot_f2v_prv) + 
            " nrm_dot_f2v_cur: " + String.format ("%.8f", nrm_dot_f2v_cur) + 
            " \n "
         );
      }

      return pentPt;
   }
   
   /** 
    * Refine a colliding edge-edge info into Artisynth's 
    * EdgeEdgeContact class in order to hook up with its LCP solver.
    */
   protected EdgeEdgeContact createEdgeEdgeContact(SweptEdge se0, SweptEdge se1, 
      CCRV ccrv) {
      
      EdgeEdgeContact eeCt = new EdgeEdgeContact();
      eeCt.edge0 = se0.myEdge;
      eeCt.edge1 = se1.myEdge;
      
      // Compute penetration depth
      
      Point3d e0_pt_prv = new Point3d();
      Point3d e1_pt_prv = new Point3d();
      se0.computeClosestEdgePairPoints (se1, 0, e0_pt_prv, e1_pt_prv, null, mySpaceElipson);
      
      Point3d e0_pt_cur = new Point3d();
      Point3d e1_pt_cur = new Point3d();
      se0.computeClosestEdgePairPoints (se1, 1, e0_pt_cur, e1_pt_cur, null, mySpaceElipson);
      
      Vector3d e10_pt_prv = new Vector3d();
      e10_pt_prv.sub (e1_pt_prv, e0_pt_prv); 
      
      Vector3d e10_pt_cur = new Vector3d();
      e10_pt_cur.sub (e1_pt_cur, e0_pt_cur);  
      
      // Compute cross product between the 2 edges.

      Vector3d e01cross = new Vector3d();
      e01cross.cross (
         se0.computeInstanteousEdgeVec (ccrv.hitTime), 
         se1.computeInstanteousEdgeVec (ccrv.hitTime));
      e01cross.normalize ();
      
      double cross_dot_e10_prv = e01cross.dot (e10_pt_prv);
      double cross_dot_e10_cur = e01cross.dot (e10_pt_cur);
      
      // Rule
      // TODO Might need to modify to be similar to VT rule 
      // if problems occur.
      if (cross_dot_e10_prv > 0)
         e01cross.negate ();
      
      Vector3d e01crossbp = new Vector3d(e01cross);
      
      eeCt.point0 = se0.computeInstanteousEdgePoint (ccrv.hitTime, ccrv.r);
      eeCt.point1 = se1.computeInstanteousEdgePoint (ccrv.hitTime, ccrv.s);
      
      eeCt.displacement = e10_pt_cur.norm ();
      eeCt.point1ToPoint0Normal = e01cross;
      
      // NEW: Edit 0 to minHitTimeBacktrack is 2nd arg
      double hitTime = Math.max (ccrv.hitTime-mHitTimeBacktrack, 0);
      
      eeCt.e0Pnts_justBefore_hitTime = se0.computeInstanteousEdgePoints (hitTime);
      eeCt.e1Pnts_justBefore_hitTime = se1.computeInstanteousEdgePoints (hitTime);
      
      // EXP 
      
//      computeEdgeEdgeCollision_normalAndDist (se0, se1,
//         ccrv, eeCt.point1ToPoint0Normal);
      
//      Vector3d e01 = new Vector3d();
//      e01.sub (e0_pt_prv, e0_pt_cur);
//      eeCt.point1ToPoint0Normal = MathUtil.vectorProjection (e01, e01cross);
//      eeCt.point1ToPoint0Normal.normalize ();
      
      
      if (myDebug)
         System.out.println ("Edge-Edge Contact: " + 
            " E0: " + eeCt.edge0.vertexStr () + 
            " E1: " + eeCt.edge1.vertexStr () + 
            " e10_pt_prv: " + e10_pt_prv.toString ("%.4f") + 
            " dot: " + cross_dot_e10_prv + 
            " e10_pt_cur: " + e10_pt_cur.toString ("%.4f") + 
            " dot: " + cross_dot_e10_cur + 
            " e01crossbp: " + e01crossbp.toString ("%.4f") + 
            " e01cross: " + e01cross.toString ("%.4f") + 
            " disp: " + eeCt.displacement + 
            "\n"
         );     
      
      eeCt.s0 = ccrv.r;
      eeCt.s1 = ccrv.s;
      
      eeCt.hitTime = hitTime;
      
      return eeCt;
   }
   

   
   /**
    * Print out all the qualified collisions.
    */
   protected void printCollisions(ContactInfo cinfo) {
      for (EdgeEdgeContact eeCt : cinfo.getEdgeEdgeContacts ())
         System.out.printf ("  Edge-Edge Collision: %s-%s. Disp: %.4f, "+
           "HitTime: %.4f, Normal: %s\n", eeCt.edge0.vertexStr (), eeCt.edge1.vertexStr (),
           eeCt.displacement, eeCt.hitTime, eeCt.point1ToPoint0Normal.toString ("%.2f"));
      
      for (int m=0; m<2; m++)
         for (PenetratingPoint pentPt : cinfo.getPenetratingPoints (m)) {
            FemMeshComp fmc0 = (m==0) ? mFmc0 : mFmc1;
            FemMeshComp fmc1 = (m==0) ? mFmc1 : mFmc0;
            
            int v = (fmc0 != null) ?
               fmc0.getNodeForVertex (pentPt.vertex).getNumber () :
               pentPt.vertex.getIndex ();
            
            String faceVtxStr = (fmc1 != null) ? 
               String.format ("[%d-%d-%d]",  
                  fmc1.getNodeForVertex (pentPt.face.getVertex (0)).getNumber (), 
                  fmc1.getNodeForVertex (pentPt.face.getVertex (1)).getNumber (), 
                  fmc1.getNodeForVertex (pentPt.face.getVertex (2)).getNumber ()
               ) : pentPt.face.vertexStr () ;
            
            System.out.printf ("  Vtx-Tri Collision: [ %d ]-%s. Disp: %.4f, "+
              "HitTime: %.4f, Nrm: %s\n", v,
              faceVtxStr, pentPt.distance, pentPt.hitTime, pentPt.normal.toString ("%.2f"));
         }
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

   
   /* --- Data Classes --- */
   
   /**
    * For each vertex or edge, keep an organized list of its collisions 
    * (e.g. vertex-triangle or edge-edge).
    * 
    * Used for filtering out redundant collisions. 
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
   

   
   /* --- Functions for computing imminent (i.e. nearby) contacts --- */

   /** 
    * Is the vertex and triangle nearby?
    * 
    * @param sv 
    * Vertex. Only its previous position is used.
    * 
    * @param st 
    * Triangle. Only its previous position is used.
    */
   protected CCRV isCollisionImminent(SweptVertex sv, SweptTriangle st) {

      Point3d[] svPntsW = sv.createTransformedPoints ();
      Point3d[] stPntsW = st.createTransformedPoints ();
      
      Point3d closestTriPnt = MathUtil.projectPoint2plane (
         svPntsW[sv.V0], 
         stPntsW[st.A0], stPntsW[st.B0], stPntsW[st.C0]);
      
      CCRV rv = new CCRV();
      
      // Compute normal between the vertex and triangle (i.e. what direction
      // should the vertex bounce towards, assuming imminent collision)?

      Vector3d f2v = new Vector3d().sub (svPntsW[sv.V0], closestTriPnt);
      
      st.myFace.computeNormal ();
      Vector3d nrm = new Vector3d( st.myFace.getWorldNormal ());
      double f2v_dot_nrm = nrm.dot (f2v);
      if (f2v_dot_nrm < 0)   
         nrm.negate ();
      
      // Is closestTriPt outside of triangle ?
      
      rv.bary = new Vector2d();
      myContCol.computeBarycentricCoords (rv.bary, closestTriPnt, 
         stPntsW[st.A0], stPntsW[st.B0], stPntsW[st.C0], mySpaceElipson);
      
      if (! MathUtil.isBaryCoordWithinTriangle (rv.bary, mySpaceElipson)) {
         rv.hitTime = -1;
         return rv;
      }
      
      // Is point beyond cloth thickness?
      
      Vector3d sizedNrm = MathUtil.vectorProjection (f2v, nrm);
      
//      double thicknessPenetration = myClothThickness - f2v_dot_nrm;
      double thicknessPenetration = myClothThickness - sizedNrm.norm ();
      if (thicknessPenetration < 0) {   // Negative means no penetration   
         rv.hitTime = -1;
         return rv;
      }
      
      // For imminent collisions, the hitTime doesn't actually exist.
      // We'll put the distance between the imminent features here.
      // When redundant collisions are being filtered out, collisions with the
      // earliest hitTimes are prioritized. For imminent collisions, we'll
      // prioritize imminent collisions that are closer to each other.
      rv.hitTime = sizedNrm.norm ();   
      
      rv.normal = nrm;
      rv.thicknessPenetration = thicknessPenetration;

      return rv; 
   }
   
   /** 
    * Refine a nearby vertex-triangle info into Artisynth's 
    * PenetratingPoint class in order to hook up with its LCP solver.
    */
   protected PenetratingPoint createImminentPenetratingPoint(
   SweptVertex sv, SweptTriangle st, CCRV ccrv)
   {
      // Compute their velocities
      
//      Vector3d vtxVel = sv.myAvgVels[sv.V1];
//      Vector3d triVel = MathUtil.linearInterpolation (
//         st.myAvgVels[st.A1], st.myAvgVels[st.B1], st.myAvgVels[st.C1], 
//         ccrv.bary.x, ccrv.bary.y);
//      
//      // Compute their velocities along the normal
//      
//      Vector3d vtxVelNrm = MathUtil.vectorProjection (vtxVel, ccrv.normal);
//      Vector3d triVelNrm = MathUtil.vectorProjection (triVel, ccrv.normal);
//      
//      // Compute the single 'relative velocity', relative to vertex.
//      // Points in the direction of the dominate velocity.
//      
//      Vector3d relVel = new Vector3d();
//      relVel.scaledAdd (0.5, vtxVelNrm);
//      relVel.scaledAdd (0.5, triVelNrm);
//      
//      // Make sure 'relative velocity' faces the normal direction
//      
//      if (relVel.dot (ccrv.normal) < 0)
//         relVel.negate ();
      
      // Compute impulse magnitude to undo the thickness overlap 
      
//      double overlapImpulse = myTimestep * mySpringStiffness * ccrv.thicknessPenetration;
//      
//      // Limit velocity change to 10% of thickness overlap.
//      // Mass will be factored in MLPC step.
//      overlapImpulse = Math.min(
//         overlapImpulse,
////         myAvgNodeMass * // Node mass will be accounted in the mass block matrix.
//            Math.abs (0.1 * ccrv.thicknessPenetration / myTimestep - relVel.norm ())
//      );
      
      double overlapImpulse = ccrv.thicknessPenetration;  
      
      // Create PenetratingPoint
      
      Point3d[] svPntsW = sv.createTransformedPoints ();
      Point3d[] stPntsW = st.createTransformedPoints ();
      
      Point3d closestTriPnt = MathUtil.linearInterpolation (
         stPntsW[st.A1], stPntsW[st.B1], stPntsW[st.C1],
         ccrv.bary.x, ccrv.bary.y);
      
      Vector3d v2f = new Vector3d().sub (closestTriPnt, svPntsW[sv.V0]);
      
      PenetratingPoint pentPt = new PenetratingPoint(
         sv.myVertex, st.myFace, ccrv.bary, closestTriPnt, v2f, null);
      
      pentPt.distance = overlapImpulse * myImminentImpulseScale;
      pentPt.hitTime = ccrv.hitTime;
      pentPt.normal = ccrv.normal;
      
      // NEW 
      pentPt.distance *= -1;
      
      return pentPt;
   }
   
   /** 
    * Is the edge pair nearby?
    * 
    * @param sv 
    * Edge0. Only its previous position is used.
    * 
    * @param st 
    * Edge1. Only its previous position is used.
    */
   protected CCRV isCollisionImminent(SweptEdge se0, SweptEdge se1) {
      Point3d pnt0 = new Point3d();
      Point3d pnt1 = new Point3d();
      
      Vector2d rs = new Vector2d();
      
      se0.computeClosestEdgePairPoints (se1, 0, pnt0, pnt1, rs, mySpaceElipson);
      
      CCRV rv = new CCRV();
      
      // Compute the normal between the edges.
      // Vector will coincide with direction that edge0 should bounce
      // towards.
      
      // Vector from edge1 to edge0
      Vector3d e01 = new Vector3d().sub (pnt1, pnt0);
      
      Vector3d nrm = new Vector3d();
      nrm.cross (
         se0.computeInstanteousEdgeVec (0), 
         se1.computeInstanteousEdgeVec (0));
      nrm.normalize ();
      
      // Tentative fix
      double e01_dot_nrm = e01.dot (nrm);
      if (e01_dot_nrm > 0) {
         nrm.negate ();
      }
      
      // HACK
      // Special boundary case. If two 
      if (isZeroOrOne (rs.x) || isZeroOrOne (rs.y)) {
         nrm = new Vector3d();
         nrm.sub (pnt0, pnt1);
         nrm.normalize ();
      }
      // HACK
      
      Vector3d sizedNrm = MathUtil.vectorProjection (e01, nrm);
      
//      double thicknessOverlap = myClothThickness - e01_dot_nrm;
      double thicknessPenetration = myClothThickness - sizedNrm.norm ();
      if (thicknessPenetration < 0) {
         rv.hitTime = -1;
      }
      else {
         // For imminent collisions, the hitTime doesn't actually exist.
         // We'll put the distance between the imminent features here.
         // When redundant collisions are being filtered out, collisions with the
         // earliest hitTimes are prioritized. For imminent collisions, we'll
         // prioritize imminent collisions that are closer to each other.
         rv.hitTime = sizedNrm.norm();   
         rv.normal = nrm;
         rv.r = rs.x;
         rv.s = rs.y;
         rv.thicknessPenetration = thicknessPenetration;
      }
      
      return rv;
   }
   
   /** 
    * Refine a nearby edge-edge info into Artisynth's 
    * PenetratingPoint class in order to hook up with its LCP solver.
    */
   protected EdgeEdgeContact createImminentEdgeEdgeContact(
   SweptEdge se0, SweptEdge se1, CCRV ccrv)
   {
      // Compute their velocities 
      
//      Vector3d vel0 = MathUtil.linearInterpolation (
//         se0.myAvgVels[se0.H1], se0.myAvgVels[se0.T1], ccrv.r);
//      
//      Vector3d vel1 = MathUtil.linearInterpolation (
//         se1.myAvgVels[se1.H1], se1.myAvgVels[se1.T1], ccrv.s);
//      
//      // Compute their velocities along the normal
//      
//      Vector3d velNrm0 = MathUtil.vectorProjection (vel0, ccrv.normal);
//      Vector3d velNrm1 = MathUtil.vectorProjection (vel1, ccrv.normal);
//      
//      // Compute the single 'relative velocity', relative to vertex.
//      
//      Vector3d relVel = new Vector3d();
//      relVel.scaledAdd (0.5, velNrm0);
//      relVel.scaledAdd (0.5, velNrm1);
//      
//      // Make sure 'relative velocity' faces the normal direction
//      
//      if (relVel.dot (ccrv.normal) < 0)
//         relVel.negate ();
      
      // Compute impulse magnitude to undo the thickness overlap 
      
//      double overlapImpulse = myTimestep * mySpringStiffness * ccrv.thicknessPenetration;
//      
//      // Limit velocity change to 10% of thickness overlap.
//      // Mass will be factored in MLPC step.
//      overlapImpulse = Math.min(
//         overlapImpulse,
////       myAvgNodeMass * // Node mass will be accounted in the mass block matrix.
//            Math.abs (0.1 * ccrv.thicknessPenetration / myTimestep - relVel.norm ())
//      );
      
      double overlapImpulse = ccrv.thicknessPenetration;  
      
      // Create EdgeEdgeContact
      
      EdgeEdgeContact eeCt = new EdgeEdgeContact();
      eeCt.edge0 = se0.myEdge;
      eeCt.edge1 = se1.myEdge;
      
      eeCt.displacement = overlapImpulse * myImminentImpulseScale;
      eeCt.point0 = se0.computeInstanteousEdgePoint (0, ccrv.r);
      eeCt.point1 = se1.computeInstanteousEdgePoint (0, ccrv.s);
      eeCt.point1ToPoint0Normal = ccrv.normal;
      eeCt.hitTime = ccrv.hitTime;
      eeCt.s0 = ccrv.r;
      eeCt.s1 = ccrv.s;
      
      // NEW 
      eeCt.displacement *= -1;
      
      return eeCt;
   }
   
   ////////////// Helper Functions
  
   /** Is x close to 0 or 1? */
   public boolean isZeroOrOne(double x) {
      return (Math.abs (x) < myTimeElipson || Math.abs (1-x) < myTimeElipson); 
   }
   

}
