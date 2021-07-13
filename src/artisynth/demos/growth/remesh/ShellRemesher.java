package artisynth.demos.growth.remesh;

import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.Map;

import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.ShellElement3d;
import artisynth.core.modelbase.ModelComponentBase;
import artisynth.demos.growth.thinshell.EdgeDataMap;
import artisynth.demos.growth.thinshell.EdgeDataMap.EdgeData;
import artisynth.demos.growth.util.MathUtil;
import artisynth.demos.growth.util.MeshUtil;
import artisynth.demos.growth.util.ShellUtil;
import maspack.geometry.Face;
import maspack.geometry.HalfEdge;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Matrix2d;
import maspack.matrix.Matrix2x3;
import maspack.matrix.Matrix3d;
import maspack.matrix.Matrix3x2;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;
import maspack.util.DataBuffer;
import maspack.util.Pair;

/** 
 * Remesher for a FEM-Shell model.
 * 
 * The FEM-Shell model is treated as a surface mesh where the vertices, edges,
 * and faces of the mesh corresponds with the front nodes, front edges, and 
 * front faces of the FEM-Shell model.
 */
public class ShellRemesher extends ShellRemeshOps {
   
   /** Measures the amount of refinement needed. */
   public SizingField mSizingField;
   
   protected static double mSF_magicFlipThreshold = 1e-2;
   protected static double mSF_magicFlipTriRadius = 1e-2;
   
   protected final int REMOVED_FLAG = ModelComponentBase.createTempFlag ();
   
   /** Whenever an edge is collapsed, a new node should be created that is 
    *  the average of the edge's parent nodes. */
   protected boolean isCollapseEdge_viaAvg = true;
   
   /** Force edge collapse  if it has a poor aspect ratio. */
   protected boolean allowCollapseForAlreadyPoorAspEdges = true;
   protected double edgeMetricCollapseThreshold = 0.9;
   
   /** 
    * Initialize the remesher. 
    * 
    * @param mesh
    * Represents the front surface of the FEM-Shell model. Required to 
    * traverse through the topology of the model.
    * 
    * @param femModel
    * FEM-Shell model to be remeshed. 
    * 
    * @param sizeMin
    * Minimum edge size.
    * 
    * @param sizeMax
    * Maximum edge size.
    * 
    * @param aspectMin
    * Minimum aspect ratio to target. Aspect ratio is the ratio between the 
    * smallest and largest edge size of a triangle. 0.5 is recommended.
    * 
    * @param refineAngle
    * Weighing for the curvature metrics. Higher weight -> less refinement 
    * at curved areas. 
    * 
    * @param refineComp
    * Weighing for the strain metrics. Higher weight -> less refinement 
    * at strained areas.
    * 
    * @param refineVelocity
    * Weighing for the velocity metrics. Higher weight -> less refinement 
    * at moving areas.
    */
   public ShellRemesher (PolygonalMesh mesh, FemModel3d femModel, 
   double sizeMin, double sizeMax, double aspectMin, double refineAngle, 
   double refineComp, double refineVelocity) {
      super (mesh, femModel);
      mSizingField = new SizingField(mesh, femModel, sizeMin, sizeMax, aspectMin, 
         refineAngle, refineComp, refineVelocity);
   }
   
   protected void setTarget(PolygonalMesh mesh, FemModel3d femModel) {
      super.setTarget (mesh, femModel);
      mSizingField.setTarget (mesh, femModel);
   }
   


   /* --- Primary Remeshing Function --- */
   
   /** 
    * Perform a remesh.
    * 
    * dynamicremesh.cpp :: dynamic_remesh(Mesh&...)
    */
   public void remesh() {
      // Compute the thin-shell plastic bending strain matrix for each element 
      // before remeshing. Newly created elements will have their strain matrix 
      // computed on the spot.
      if (mFemModel.myThinShellAux != null) {
         for (Face face : mMesh.getFaces ()) {
            ShellElement3d ele = mFemModel.getShellElement (face.idx);
            
            Matrix3d bendStrain = mFemModel.myThinShellAux.bendStrain_edgesToFace (face);
            ele.getPlasticBendStrain ().set (bendStrain);
         }
      }
      
      mSizingField.computeVertexSizingFields ();
      
      SF_flipEdges (mMesh.getFaces (), null);

      while (SF_splitWorstEdge(MeshUtil.getAllEdgesViaCopy (mMesh.getFaces ())));

      while (SF_improveSomeFaces(mMesh.getFaces ()) );
      
      if (isDebug) System.out.printf (
         "Num denied collapsed b/c poor faces: %d, " +
         "b/c excessive edge metric: %d\n", 
         numDeniedCollapseBcPoorFace, numDeniedCollapseBcBigEdgeMetric);
      
      // Compute the thin-shell plastic strain for each edge.
      if (mFemModel.myThinShellAux != null) {
         // Reconstruct the map to reflect the remeshed mesh.
         mFemModel.myEdgeDataMap = new EdgeDataMap(mFemModel, mMesh);
         
         for (Face face : mMesh.getFaces ()) {
            ShellElement3d ele = mFemModel.getShellElement (face.idx);
            
            Vector3d edgeStrains = mFemModel.myThinShellAux.
               bendStrain_faceToEdges (face, ele.getPlasticBendStrain ());
            
            for (int e = 0; e < 3; e++) {
               HalfEdge edge = face.getEdge (e);
               if (edge.opposite == null) {
                  continue;
               }
               
               EdgeData edgeData = mFemModel.myEdgeDataMap.get (edge);
               edgeData.mAngStrain += edgeStrains.get (e) / 2;
               // Divide by 2 because opposite face will also contribute to
               // strain.
            }
         }
         
         // Refresh node neighbors.
         mFemModel.myThinShellAux.refreshIndirectNodeNeighbors ();
      }
   }
   
   
   /* --- (1) Flipping Algorithm --- */
   
   /** 
    * Flip the necessary edges of the mesh for a few iterations. 
    * 
    * @param activeFaces
    * Faces that can be modified. Used to prevent duplicate flips.
    * 
    * @param edgesToUpdate
    * Edges that can be modified. Used to prevent duplicate flips.
    */
   protected void SF_flipEdges(ArrayList<Face> activeFaces, 
   LinkedList<HalfEdge> edgesToUpdate) {
      int N = 3*activeFaces.size ();
      for (int i = 0; i < N; i++) {
         if (isDebug) System.out.printf ("SF_flipEdges() :: "+
            "Iteration: %d/%d\n", i, N);
         
         if ( ! SF_flipSomeEdges(activeFaces, edgesToUpdate, i) ) {
            return;
         }
      }
   }
    
   /** Maximum number of edges that can be flipped in the last call to
    *  SF_flipSomeEdges().
    * 
    *  Static variable for SF_flipSomeEdges().
    */
   protected int numEdgesToUpdate_prev = 0;   
   
   /**
    * Flip the necessary edges of the mesh.
    * 
    * @param activeFaces
    * Faces that can be modified. Used to prevent duplicate flips. This is 
    * will be incremented and decrement as faces are added and removed.
    * 
    * @param edgesToUpdate
    * Edges that can be modified. Used to prevent duplicate flips. This will be
    * decremented as edges are flipped.
    * 
    * @param iter
    * Which iteration are we on? Sanity checking purposes.
    * 
    * @return
    * True if at least one edge was flipped.
    */
   protected boolean SF_flipSomeEdges(ArrayList<Face> activeFaces, 
   LinkedList<HalfEdge> edgesToUpdate, int iter) {
      LinkedList<HalfEdge> edges = SF_independentEdges( 
         SF_findEdgesToFlip(activeFaces) );

      if (isDebug) System.out.printf ("SF_flipSomeEdges() :: "+
         "Independent edges to flip: %d,  Previous: %d\n",
         edges.size (), numEdgesToUpdate_prev);
      
      if (edges.size () == numEdgesToUpdate_prev) {    // Prevent infinite loop.
         return false;                        
      }
      
      // TODO: Hack to prevent SF_flipEdges() from iterating the entirely
      // of N, which will grow very large.
      if (activeFaces == mMesh.getFaces () && numEdgesToUpdate_prev != 0 &&
          // Prevent infinite loop.    
          edges.size () >= numEdgesToUpdate_prev && iter > 0) {  
         return false;                        
      }
      
      numEdgesToUpdate_prev = edges.size ();
      
      for (HalfEdge edge : edges) {
         if (isDebug) System.out.printf ("Flip: %d-%d\n", 
            edge.head.getIndex (), edge.tail.getIndex ());
         
         edge = MeshUtil.getEdge (edge.head, edge.tail);
         
         OpRv rv = flip(edge);    
         ShellUtil.detectProblematicRestArea ( 
            mFemModel.getShellElement(rv.mAddedFaces[0].idx) );
         ShellUtil.detectProblematicRestArea ( 
            mFemModel.getShellElement(rv.mAddedFaces[1].idx) );
         
         if (activeFaces != mMesh.getFaces ()) {
            activeFaces.remove (rv.mRemovedFaces[0]);
            activeFaces.remove (rv.mRemovedFaces[1]);
            activeFaces.add (rv.mAddedFaces[0]);
            activeFaces.add (rv.mAddedFaces[1]); 
         }
         
         if (edgesToUpdate != null) {
            removeEdgeFromList(edgesToUpdate, rv.mRemovedEdge);
         }
      }
      
      return true;
   }
   
   /** 
    * Remove edges from a set of edges.
    * 
    * @param edgesToUpdate
    * Set of edges.
    * 
    * @param edgeToRemove
    * Edges to remove from the set.
    */
   protected void removeEdgeFromList(
   LinkedList<HalfEdge> edgesToUpdate, HalfEdge edgeToRemove) {
      // Don't worry about optimization. edgesToUpdate is typically
      // only size 4, generate from the faces of a flipped edge
      for (int e = 0; e < edgesToUpdate.size (); e++) {
         if (edgesToUpdate.get (e) == null) {
            continue;
         }
         
         if (MeshUtil.isShared2VtxsEdge (edgesToUpdate.get (e), edgeToRemove)) {
            edgesToUpdate.set (e, null);
            return;
         }
      }
   }
   
   /**
    * Search for edges that can be flipped.
    * 
    * @param activeFaces
    * Edges that belong to this set of faces will be searched.
    * 
    * @return
    * Flippable edges.
    */
   protected LinkedList<HalfEdge> SF_findEdgesToFlip(ArrayList<Face> activeFaces) {
      LinkedList<HalfEdge> rv = new LinkedList<HalfEdge>();
      for (HalfEdge edge : MeshUtil.getAllEdgesViaCopy (activeFaces)) {
         // TODO: Add edge preserving condition here
         if ( MeshUtil.isBoundaryEdge (edge) || ! SF_shouldFlip(edge) ) {
            continue;
         }
         rv.add (edge);
      }

      return rv;
   }
   
   // Debugging only. Counts the number of reasons why edges are rejected
   // for flipping. 
   int f1, f2, f3, f4 = 0;
   
   /**
    * Is the given edge flippable?
    * 
    * @param edge
    * Non-boundary edge
    */
   protected boolean SF_shouldFlip(HalfEdge edge) {
      double maxAngle = 40*Math.PI/180.0;    
      
      Vertex3d vtx0 = edge.head;
      Vertex3d vtx1 = edge.tail;
      
      // Corresponding nodes
      
      int n0 = vtx0.getIndex ();
      int n1 = vtx0.getIndex ();
      int opp_n0 = vtx1.getIndex ();
      int opp_n1 = vtx0.getIndex ();
      
      // XXX: Notice that opp_n1 and opp_n0 are swapped. Fixes bug
      // with flipped_n0 being negative when it shouldn't.
      
      FemNode3d node0 = mFemModel.getNode(n0);
      FemNode3d node1 = mFemModel.getNode(n1);
      FemNode3d oppNode0 = mFemModel.getNode(opp_n0);  
      FemNode3d oppNode1 = mFemModel.getNode(opp_n1);
      
      // Material space
      Point3d x = new Point3d( node0.getRestPosition () );
      Point3d z = new Point3d( node1.getRestPosition () );
      Point3d w = new Point3d( oppNode0.getRestPosition () );
      Point3d y = new Point3d( oppNode1.getRestPosition () );
      
      // TODO: Custom. If flip operation will create zero area triangle,
      // do not perform.
//      if ( MathUtil.compare (MathUtil.area (w, y, x), 0, mSF_magicMinFlipArea) == 0 ||
//           MathUtil.compare (MathUtil.area (w, y, z), 0, mSF_magicMinFlipArea) == 0 ) {
//         return false;
//      }
//      if ( MathUtil.minRadius(w,y,x) < mSF_magicFlipTriRadius ||
//           MathUtil.minRadius(w,y,z) < mSF_magicFlipTriRadius ) { 
//         return false;
//      }
      
      // Don't flip if high angles are involved
      if (abs (mSizingField.dihedralAngle (edge, false /*isPlasticSpace*/)) 
         >= maxAngle) 
      {
         f1++;
         return false;
      }
      
      // 4 sides of the 2 faces
      Vector3d y_x = new Vector3d().sub (y, x);
      Vector3d w_x = new Vector3d().sub (w, x);
      Vector3d w_z = new Vector3d().sub (w, z);
      Vector3d y_z = new Vector3d().sub (y, z);
      
      // Normals of faces if flipped occurred
      Vector3d flipped_n0 = new Vector3d().cross (y_x, w_x).normalize ();
      Vector3d flipped_n1 = new Vector3d().cross (w_z, y_z).normalize ();
      
      // This will also catch flipped edges that go outside of the perimeter
      if ( abs(mSizingField.dihedralAngle (w, y, flipped_n0, flipped_n1)) 
         >= maxAngle ) 
      {
         f2++;
         return false;
      }
      
      // Don't flip if mean normal is inverted
      
      Vector3d nrm0 = edge.getFace ().getNormal ();
      Vector3d nrm1 = edge.getOppositeFace ().getNormal ();
      
      Vector3d flipped_n0_add_n1 = new Vector3d().add (flipped_n0, flipped_n1);
      Vector3d n0_add_n1 = new Vector3d().add (nrm0, nrm1);
      
      if ( flipped_n0_add_n1.dot (n0_add_n1) <= 0 ) {    
         f3++;
         return false;
      }
      
      // Project onto a 2D plane which conserves diagonal lengths
      
      Vector3d w_y = new Vector3d().sub (w, y);
      
      Vector3d u = new Vector3d().sub (x, z).normalize ();
      Vector3d v = new Vector3d(w_y).scaledAdd (- w_y.dot (u), u).normalize ();
      
      Matrix3x2 A = new Matrix3x2();
      A.setColumn (0, u);
      A.setColumn (1, v);
      
      Matrix2x3 At = new Matrix2x3();
      At.transpose (A);
      
      Matrix3d M = new Matrix3d();
      M.add ( mSizingField.getVertexSF (n0) );
      M.add ( mSizingField.getVertexSF (n1) );
      M.add ( mSizingField.getVertexSF (opp_n0) );
      M.add ( mSizingField.getVertexSF (opp_n1) );
      M.scale (1.0 / 4.0);
      
      MatrixNd Mr;
      Mr = MathUtil.mul (At, M);
      Mr = MathUtil.mul (Mr, A);
      
      Vector2d At_x = new Vector2d();   MathUtil.mul (At, x, At_x);
      Vector2d At_y = new Vector2d();   MathUtil.mul (At, y, At_y);
      Vector2d At_z = new Vector2d();   MathUtil.mul (At, z, At_z);
      Vector2d At_w = new Vector2d();   MathUtil.mul (At, w, At_w);
      
      boolean rv = SF_shouldFlip2(At_x, At_y, At_z, At_w, new Matrix2d(Mr));
      
      if (!rv) {
         f4++;
      }
      
      return rv;
   }
   
   /** Given the 4 sides of 2 adjacent faces (projected to 2D plane), should 
    *  edge be flipped? */
   protected boolean SF_shouldFlip2(Vector2d x, Vector2d y, Vector2d z,
   Vector2d w, Matrix2d M) {
      Vector2d z_y = new Vector2d().sub (z, y);
      Vector2d x_y = new Vector2d().sub (x, y);
      
      Vector2d x_w = new Vector2d().sub (x, w);
      Vector2d z_w = new Vector2d().sub (z, w);
      
      double area0 = abs( MathUtil.wedge (z_y, x_y) );
      double area1 = abs( MathUtil.wedge (x_w, z_w) );
      
      Vector2d M_z_w = new Vector2d();
      M.mul (M_z_w, z_w);
      
      Vector2d M_x_y = new Vector2d();
      M.mul (M_x_y, x_y);
      
      return area0 * x_w.dot (M_z_w) + area1 * z_y.dot (M_x_y) < 
         -mSF_magicFlipThreshold * (area0+area1);
   }
   
   
   
   /* --- (2) Splitting Algorithm --- */
   

   /**
    * Given a set of edges, bisect the "worst" edge. 
    * 
    * @return
    * true if some "bad" edges remaining.
    */
   protected boolean SF_splitWorstEdge(LinkedList<HalfEdge> edges) {
      LinkedList<HalfEdge> badEdges = SF_findBadEdges(edges);

      for (int e = 0; e < badEdges.size (); e++) {
         HalfEdge badEdge = badEdges.get (e);
         // Some bad edges may be removed because of previous loop iteration's
         // call to SF_flipEdges.
         if (badEdge == null) {
            continue;
         }
         
         // Flipped edges from previous iteration create new faces, so need
         // to get the updated edge. When flipping and bisecting edges,
         // can assume that no nodes are being destroyed or renamed to
         // different indices, so can assure that edge to search for will
         // exist.
         badEdge = MeshUtil.getEdge (badEdge.head, badEdge.tail);
         if (badEdge == null) {
            assert(false);
            continue;
         }
         
         if (isDebug) System.out.printf ("BisectEdge: %d-%d\n", 
            badEdge.head.getIndex (), badEdge.tail.getIndex ());
         OpRv rv = bisectEdge (badEdge);   
         
         if (isDebug) System.out.println ("New bisect node: " + 
            rv.mNewVertex.getIndex ());
         
         ArrayList<Face> activeFaces = rv.getAddedFacesAsList();
         
         // Note: Any edge that is removed from a flip operation will also
         // be removed from badEdges
         SF_flipEdges (activeFaces, badEdges);
      }
      
      return false;
   }
   
   /** 
    * Search bad edges within a given set of edges. 
    * 
    * @return
    * Bad edges, sorted from largest edge "size".
    */
   protected LinkedList<HalfEdge> SF_findBadEdges(LinkedList<HalfEdge> edges) {
      LinkedList< Pair<Double,HalfEdge> > oversizedEdges = 
         new LinkedList< Pair<Double,HalfEdge> >();
      
      for (HalfEdge edge : edges) {
         double m = edgeMetric(edge);
         if (m > 1) {
            HalfEdge edgeCopy = MeshUtil.createIndependentCopy (edge);
            oversizedEdges.add ( new Pair<Double,HalfEdge>(m,edgeCopy) );
         }
      }
    
      // Sort from smallest size
      Collections.sort(oversizedEdges, new Comparator< Pair<Double,HalfEdge> >() {
         @Override
         public int compare(
         final Pair<Double,HalfEdge> o1, final Pair<Double,HalfEdge> o2) {
            return Double.compare (o1.first, o2.first);
         }
      });
      
      // Put back into regular list, sorted from largest size 
      LinkedList<HalfEdge> rv = new LinkedList<HalfEdge>();
      for (int i = oversizedEdges.size ()-1; i >= 0; i--) {
         rv.add ( oversizedEdges.get (i).second );
         if (isDebug) System.out.println ("SF_findBadEdges() :: Edge metric: " + 
            oversizedEdges.get (i).first);
      }
      
      return rv;
   }
   
   
   
   
   /* --- (3) Collapse Algorithm --- */
   
   /** Collapse edges to improve the topology of the mesh.
    * 
    * @param activeFaces 
    * Edges within this set of faces will be inspected.
    * 
    * @return 
    * true if an edge was collapsed. 
    */
   protected boolean SF_improveSomeFaces(ArrayList<Face> activeFaces) {
      for (int f = 0; f < activeFaces.size (); f++) {
         Face face = activeFaces.get (f);
         
         // Some active faces may be removed by collapse
         if (face.checkFlag (REMOVED_FLAG)) {
            continue;
         }
         
         for (int e = 0; e < 3; e++) {
            HalfEdge edge = face.getEdge (e);   
            OpRv rv = null;
            
            rv = SF_tryEdgeCollapse(edge, edge.head);
            
            if (rv.isEmpty ()) {
               rv = SF_tryEdgeCollapse(edge, edge.tail);
            }
            
            if (rv.isEmpty ()) {
               continue;
            }
            
            // Debug only.
            //return false;

            for (Face removedFace : rv.mRemovedFaces) {
               removedFace.setFlag (REMOVED_FLAG);
            }
            
            // activeFaces is already updated because from mMesh.getFaces()
            
            // Don't worry about concurrent modification to activeFaces
            // iteration, will return immediately.
            SF_flipEdges (rv.getAddedFacesAsList (), null /*edgesToUpdate*/);
            return true;
         }
      }
      
      return false;
   }
   
   /** 
    * Check if the given edge can be collapsed, and if so, collapse it.
    * 
    * @param rVtx 
    * Which vertex of the edge should be removed, if the collapse were to occur.
    * This is overridden when `isCollapseEdge_viaAvg` is true; both vertices of 
    * the edges are replaced with an average vertex.
    * 
    * @return 
    * Details regarding the collapse.
    */
   protected OpRv SF_tryEdgeCollapse(HalfEdge edge, Vertex3d rVtx) {
      // TODO: preserve logic, node or edge
      if ( MeshUtil.isBoundaryVtx (edge.head) || 
           MeshUtil.isBoundaryVtx (edge.tail) ) {
         return createOpRv();
      }

      if ( ! SF_canCollapse(edge, rVtx) ) {
         return createOpRv();
      }
      
      if (isDebug) System.out.printf (
         "CollapseEdge: %d-%d\n", edge.head.getIndex (), edge.tail.getIndex ());
      
      OpRv rv = collapseEdge (edge, rVtx, this.isCollapseEdge_viaAvg);
      return rv;
   }
   
   
   protected int numDeniedCollapseBcPoorFace = 0;
   protected int numDeniedCollapseBcBigEdgeMetric = 0;
   
   /** 
    * Given an edge, can it be collapsed?
    * 
    * @param rVtx 
    * The vertex of the edge that should be removed, if the collapse were to 
    * occur. This is overridden when `isCollapseEdge_viaAvg` is true; 
    * both vertices of the edges are replaced with an average vertex.
    */
   protected boolean SF_canCollapse(HalfEdge edge, Vertex3d rVtx) {
      
      // The 1-ring faces of this current vtx, after it's moved
      // (i.e. to midpoint), will be inspected.
      for (Vertex3d curEdgeVtx : new Vertex3d[] {edge.head, edge.tail}) {
         // If only 1 vtx will be collapsed, do not inspect the 1-ring faces of
         // vertex that will persist --- its 1-ring faces do not change.
         if (! this.isCollapseEdge_viaAvg && curEdgeVtx != rVtx) {
            continue;
         }
         
         // Get vertex opposite of edge
         Vertex3d otherEdgeVtx = MeshUtil.getOtherVertex (curEdgeVtx, edge);
         
         // Get rest position of current vertex (a)
         Vertex3d curEdgeVtxRest = new Vertex3d(); 
         curEdgeVtxRest.setPosition (
            mFemModel.getNode(curEdgeVtx.getIndex ()).getRestPosition ());
         curEdgeVtxRest.setIndex (curEdgeVtx.getIndex ());
         
         // Get rest position of other vertex (b)
         Vertex3d otherEdgeVtxRest = new Vertex3d(); 
         otherEdgeVtxRest.setPosition (
            mFemModel.getNode(otherEdgeVtx.getIndex ()).getRestPosition ());
         otherEdgeVtxRest.setIndex (otherEdgeVtx.getIndex ());
         
         // For each adjacent face surrounding current vertex
         for (Face face : MeshUtil.getOneRingFaces (curEdgeVtx)) {
            // Ignore faces that will be collapsed
            if (edge.getFace () == face || edge.getOppositeFace () == face) {
               continue;
            }
            
            Vertex3d[] triVtxs = face.getTriVertices ();
            
            // Copy triVtx but containing rest positions instead
            Vertex3d[] triVtxsRest = new Vertex3d[3];
            for (int v = 0; v < 3; v++) {
               triVtxsRest[v] = new Vertex3d();
               triVtxsRest[v].setPosition (
                  mFemModel.getNode(triVtxs[v].getIndex ()).getRestPosition ());
               triVtxsRest[v].setIndex (triVtxs[v].getIndex ());
            }

            // Compute the rest area of face
            Point3d[] triRPos = new Point3d[] {
               triVtxsRest[0].pnt, triVtxsRest[1].pnt, triVtxsRest[2].pnt};
            double a0 = MathUtil.area (triRPos[0], triRPos[1], triRPos[2]);
            double asp0 = aspect(triRPos[0], triRPos[1], triRPos[2]);
            boolean isCC0 = MeshUtil.isCounterClockwise (triVtxsRest);
            
            // Determine new position of current edge vtx.
            // Case 1: Avg 2 vtx: Move the current vtx to avg of 'cur'
            //         and 'other'.
            // Case 2: Collapse 1 vtx: Move the current vtx (i.e. rVtx) to 
            //         other vtx.
            
            Vertex3d curVtxReplacee = new Vertex3d();
            
            if (isCollapseEdge_viaAvg) {
               curVtxReplacee.pnt = MathUtil.avg (
                  curEdgeVtxRest.pnt, otherEdgeVtxRest.pnt);
            }
            else {
               curVtxReplacee = otherEdgeVtx;
            }
            
            // Collapse the edge by moving curEdgeVtx to new middle vtx
            replace(curEdgeVtxRest, curVtxReplacee, triVtxsRest);

            // Now compute the rest area of the face again
            triRPos = new Point3d[] {
               triVtxsRest[0].pnt, triVtxsRest[1].pnt, triVtxsRest[2].pnt};
            double a = MathUtil.area (triRPos[0], triRPos[1], triRPos[2]);
            double asp = aspect(triRPos[0], triRPos[1], triRPos[2]);
            boolean isCC = MeshUtil.isCounterClockwise (triVtxsRest);
            
            // FIXED BUG. Avoids inverted elements during collapse.
            if (isCC0 != isCC) {
               return false;
            }
            
            // Cannot collapse if:
            //   (Face area will shrink or face area will be smaller than
            //         0.1*sizeMin^2
            //    OR 
            //    New aspect ratio is too small)
            if ( ( a < a0 && a < 0.1*pow(mSizingField.mSizeMin,2) ) ||
                 (asp < mSizingField.mAspectMin) ) {
               
               // NEW: If asp0 was already small, permit collapse if 
               // aspect ratio will improve.
               if (allowCollapseForAlreadyPoorAspEdges && asp0 < 
                  mSizingField.mAspectMin && asp > asp0) 
               {
                  
               }
               else {
                  numDeniedCollapseBcPoorFace++;
                  return false;
               }
            }
            
            // Cannot collapse if:
            //   - Any formed edges (connecting avgVtx) exceed 0.9 size
            for (int e = 0; e < 3; e++) {
               HalfEdge faceEdge = face.getEdge (e);
               boolean hasCurEdgeVtx = 
                  MeshUtil.isHasVertex (curEdgeVtx, faceEdge);
               Vertex3d otherFaceVtx =
                  MeshUtil.getOtherVertex (curEdgeVtx, faceEdge);
               
               if (! hasCurEdgeVtx)
                  continue;
               
               if (edgeMetric_usingAvgOf2nd(
                  otherFaceVtx.getIndex (), curEdgeVtx.getIndex (),
                  otherEdgeVtx.getIndex()) > edgeMetricCollapseThreshold )
               {
                  numDeniedCollapseBcBigEdgeMetric++;
                  return false;  
               }
            }
         }
      }
      
      return true; 
   }
   
   
   
   /* --- Support Functions --- */
   
   /** 
    *  Within a given set of edges, find the largest set of edges that are not 
    *  connected to each other.
    */
   protected LinkedList<HalfEdge> SF_independentEdges(LinkedList<HalfEdge> edges) {
      LinkedList<HalfEdge> indepEdges = new LinkedList<HalfEdge>();
      
      int vtxVisitFlag = ModelComponentBase.createTempFlag ();
      
      // Greedily find the independent edges 
      for (HalfEdge edge : edges) {
         if ( ! edge.head.checkFlag (vtxVisitFlag) ||
              ! edge.tail.checkFlag (vtxVisitFlag) ) {
            indepEdges.add (edge);
         }
         edge.head.setFlag (vtxVisitFlag);
         edge.tail.setFlag (vtxVisitFlag);
      }
      
      // Remove visit flags
      for (HalfEdge edge : edges) {
         edge.head.clearFlag (vtxVisitFlag);
         edge.tail.clearFlag (vtxVisitFlag);
      }
      ModelComponentBase.removeTempFlag (vtxVisitFlag);
      
      return indepEdges;
   }
   
   /**
    * Measures the "size" of the edge. "Size" is determined by the edge nodes'
    * sizing fields.
    * 
    * @return
    * If above 1, the edge should be remeshed.
    */
   protected double edgeMetric(HalfEdge edge) {
      int n0 = edge.head.getIndex ();
      int n1 = edge.tail.getIndex ();
      
      return max( edgeMetric(n0, n1), edgeMetric(n1, n0) );
   }

   /**
    * Measures the "size" of the edge. "Size" is determined by the edge nodes'
    * sizing fields.
    * 
    * @param n0 
    * One of the edge's node.
    * 
    * @param n1
    * The other edge's node.
    * 
    * @return
    * If above 1, the edge should be remeshed.
    */
   protected double edgeMetric(int n0, int n1) {
      FemNode3d node0 = mFemModel.getNode (n0);
      FemNode3d node1 = mFemModel.getNode (n1);
      
      Vector3d du = new Vector3d().sub ( 
         node0.getRestPosition (), node1.getRestPosition () );
      
      Vector3d S0_du = new Vector3d();
      mSizingField.getVertexSF (n0).mul (S0_du, du);
      
      Vector3d S1_du = new Vector3d();
      mSizingField.getVertexSF (n1).mul (S1_du, du);
      
      return sqrt( (du.dot (S0_du) + du.dot (S1_du)) / 2.0);
   }
   
   /**
    * Measures the "size" of the edge. "Size" is determined by a node's sizing
    * field, and the average sizing field of two other nodes.
    * 
    * @param n0 
    * Use 50% of this node's sizing field.
    * 
    * @param n1_x 
    * Use 25% of this node's sizing field.
    * 
    * @param n1_y
    * Use 25% of this node's sizing field.
    * 
    * @return
    * If above 1, the edge should be remeshed.
    */
   protected double edgeMetric_usingAvgOf2nd(int n0, int n1_x, int n1_y) {
      // Square root of formula (1)
      
      FemNode3d node0 = mFemModel.getNode (n0);
      FemNode3d node1_x = mFemModel.getNode (n1_x);
      FemNode3d node1_y = mFemModel.getNode (n1_y);
      
      Point3d node1Rest = MathUtil.avg (
         node1_x.getRestPosition (), node1_y.getRestPosition ());

      Vector3d du = new Vector3d().sub ( 
         node0.getRestPosition (), node1Rest );
      
      Vector3d S0_du = new Vector3d();
      mSizingField.getVertexSF (n0).mul (S0_du, du);
      
      Vector3d S1_du = new Vector3d();
      Matrix3d S1 = MathUtil.avg (
         mSizingField.getVertexSF (n1_x), mSizingField.getVertexSF (n1_y));
      S1.mul (S1_du, du);
      
      return sqrt( (du.dot (S0_du) + du.dot (S1_du)) / 2.0);
   }
   
   
   
   /* --- Utility Functions --- */
   
   /**
    * Replace `this vertex` `with this vertex` `in this array`.
    * 
    * @param inThisArray
    * Array containing vertex indices.
    */
   protected void replace(Vertex3d thisVtx, Vertex3d withThis, Vertex3d[] inThisArray) {
      for (int i = 0; i < inThisArray.length; i++) {
         if (inThisArray[i].getIndex () == thisVtx.getIndex ()) {
            inThisArray[i] = withThis;
            return;
         }
      }
   }
   
   /** Calculate the aspect ratio of a given triangle. */
   protected double aspect (Point3d u0, Point3d u1, Point3d u2) {
      double perimeter = 
         new Vector3d().sub (u0, u1).norm () +
         new Vector3d().sub (u1, u2).norm () +
         new Vector3d().sub (u2, u0).norm ();
      
      return 
         12 * sqrt(3) * 0.5 *
         new Vector3d().cross (
            new Vector3d().sub (u1, u0), 
            new Vector3d().sub (u2, u0) ).norm () 
         / pow(perimeter, 2);
   }
   
   protected int next(int idx) {
      return (idx == 2) ? 0 : idx+1;
   }
   
   protected int prev(int idx) {
      return (idx == 0) ? 2 : idx-1;
   }
   
   
   
   /* --- Node and Shell specific Tertiary Functions --- */
   
   /** Create a new standalone node with its own sizing field. */
   protected FemNode3d createNode() {
      FemNode3d node = super.createNode ();
      mSizingField.addVertexSF (new Matrix3d());
      
      return node;
   }
   
   /** Save a node's attributes, including its sizing field. */
   protected DataBuffer saveNodeState(int n) {
      DataBuffer buf = super.saveNodeState (n);
      buf.dput (mSizingField.getVertexSF (n));
      
      return buf;
   }
   
   /** Get a node's attributes, including its sizing field. */
   protected void loadNodeState(int n , DataBuffer buf) {
      super.loadNodeState (n, buf);
      buf.dget (mSizingField.getVertexSF (n));
   }
   
   
   
}
