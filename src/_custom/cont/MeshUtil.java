package _custom.cont;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;

import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.ShellElement3d;
import artisynth.core.femmodels.ShellTriElement;
import artisynth.core.modelbase.ModelComponentBase;
import maspack.geometry.Face;
import maspack.geometry.Feature;
import maspack.geometry.HalfEdge;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.util.Pair;

public class MeshUtil {
   protected static final int SAME_FACE_FLAG = 1;
  
   
   public static String getVertexIndices(Vertex3d[] vtxs) {
      String s = "[";
      
      for (Vertex3d vtx : vtxs) {
         s += vtx.getIndex () + "-";
      }
      
      s = s.substring (0, s.length ()-1);
      s += "]";
      
      return s;
   }
   
   
   public static ArrayList<Face> getOneRingFaces(Vertex3d vtx) {
      ArrayList<Face> ring = new ArrayList<Face>();
      
      Iterator<HalfEdge> adjEdgeIter = vtx.getIncidentHalfEdges ();
      while (adjEdgeIter.hasNext ()) {
         HalfEdge adjEdge = adjEdgeIter.next ();
         ring.add ( adjEdge.getFace () );
      }
      
      return ring;
   }
   
  
   
   public static ArrayList<HalfEdge> getConnectedEdges(Vertex3d vtx) {
      ArrayList<HalfEdge> connEdges = new ArrayList<HalfEdge>();
      
      Iterator<HalfEdge> adjEdgeIter = vtx.getIncidentHalfEdges ();
      while (adjEdgeIter.hasNext ()) {
         HalfEdge adjEdge = adjEdgeIter.next ();
         
         // For a face of the center vertex
         Face face = adjEdge.getFace ();
         
         // Get the half edges that are touching the vertex.
         Iterator<HalfEdge> faceEdgeIter = face.edgeIterator ();
         while (faceEdgeIter.hasNext ()) {
            HalfEdge faceEdge = faceEdgeIter.next ();

            if (isEdgeAndVtxConnected(faceEdge, vtx) && 
                 (faceEdge.head.getIndex () < faceEdge.tail.getIndex () ||
                  isBoundaryEdge(faceEdge))) 
            {
               connEdges.add (faceEdge);
            }
         }
      }
      
      return connEdges;
   }
   

   
   public static boolean isEdgeAndVtxConnected(HalfEdge edge, Vertex3d vtx) {
      return (edge.head == vtx || edge.tail == vtx);
   }
   
   public static boolean isBoundaryEdge(HalfEdge edge) {
      return (edge.getOppositeFace () == null);
   }
   
   public static boolean isBoundaryFace(Face face) {
      for (int e = 0; e < 3; e++) {
         if (MeshUtil.isBoundaryEdge (face.getEdge (e))) {
            return true;
         }
      }
      
      return false;
   }

   
   public static Vertex3d getOtherVertex(Vertex3d skipVtx, HalfEdge edge) {
      if (edge.head == skipVtx) {
         return edge.tail;
      }
      else {
         return edge.head;
      }
   }
   
   
   public static boolean isHasVertex(Vertex3d vtx, Face face) {
      for (Vertex3d curVtx : face.getVertices ()) {
         if (vtx == curVtx) {
            return true;
         }
      }
      return false;
   }
   
   public static boolean isHasVertex(Vertex3d vtx, HalfEdge edge) {
      return (edge.head == vtx || edge.tail == vtx); 
   }
   
   public static boolean isShared2VtxsEdge(HalfEdge edge0, HalfEdge edge1) {
      return (
         (edge0.head == edge1.head && edge0.tail == edge1.tail) ||
         (edge0.head == edge1.tail && edge0.tail == edge1.head) 
      );
   }
   
   public static boolean isSameHalfEdge(HalfEdge edge0, HalfEdge edge1) {
      return (edge0.head == edge1.head && edge0.tail == edge1.tail);
   }
   
   public static HalfEdge getEdge(Vertex3d a, Vertex3d b) {
      for (HalfEdge edge : getConnectedEdges (a)) {
         if (isHasVertex(b, edge)) {
            return edge;
         }
      }
      return null;
   }
   
   public static HalfEdge getEdge(int a, int b, PolygonalMesh mesh) {
      return getEdge( mesh.getVertex (a), mesh.getVertex (b) );
   }
   

   
   public static HalfEdge getHalfEdgeWithMinHeadIdx(HalfEdge he) {
      if (he.head.getIndex () < he.tail.getIndex ()) {
         return he;
      }
      else if (he.opposite != null) {
         return he.opposite;
      }
      else {
         return he;
      }
   }
   
   public static boolean isShareVertex(HalfEdge he0, HalfEdge he1) {
      return (
         he0.head == he1.head || 
         he0.head == he1.tail || 
         he0.tail == he1.head || 
         he0.tail == he1.tail
      );
   }
   
   /**
    * Get the faces that are connected to a boundary node. Returned faces are 
    * grouped into two, separated by a specified edge.
    * 
    * @param startCutVtx
    * Vertex that's on the boundary of the mesh. Faces returned will be
    * adjacent to this vertex.
    * 
    * @param endCutVtx
    * Specified edge, which consists of startCutVtx and endCutVtx. 
    * Must be one-edge length away from startCutVtx.
    */
   public static Pair<ArrayList<Face>,ArrayList<Face>> 
   getPartitionedBoundaryNodeFaces(Vertex3d startCutVtx, Vertex3d endCutVtx) {
      Pair<ArrayList<Face>,ArrayList<Face>> faceSubsetPair = 
         new Pair<ArrayList<Face>,ArrayList<Face>>(
            new ArrayList<Face>(), new ArrayList<Face>()
         );
      
      HalfEdge cutEdge = MeshUtil.getEdge (startCutVtx, endCutVtx);
      
      // For each face subset
      for (int s = 0; s < 2; s++) {
         ArrayList<Face> faceSubset = (s == 0) ?
            faceSubsetPair.first : faceSubsetPair.second;
         
         Face prvFace = null;
         Face curFace = null; 
         
         if (cutEdge.getFace () != null) {
            prvFace = cutEdge.getOppositeFace ();
            curFace = cutEdge.getFace ();
         }
         else {
            prvFace = cutEdge.getFace();
            curFace = cutEdge.getOppositeFace ();
         }
         
         while (curFace != null) {
            // Add current face
            faceSubset.add (curFace);
            
            // Explore its edges
            boolean isFoundNewFace = false;
            for (int e = 0; e < 3; e++) {
               HalfEdge curEdge = curFace.getEdge (e);
               // If find a new adjacent face that's within the ring, jump to it
               if (curEdge.getOppositeFace () != prvFace && isHasVertex(startCutVtx, curEdge)) {
                  prvFace = curFace;
                  curFace = curEdge.getOppositeFace ();
                  // Continue search for new faces
                  isFoundNewFace = true;
                  break;
               }
            }
            
            if (! isFoundNewFace) {
               // If no new adjacent face to jump to, end search
               curFace = null;
            }
         } 
      }
      
      return faceSubsetPair;
   }
   
   protected static boolean hasEdge(Face face, HalfEdge edge) {
      for (int i = 0; i < 3; i++) {
         if (face.getEdge (i) == edge || face.getEdge (i) == edge.opposite) {
            return true;
         }
      }
      return false;
   }
   
   public static Face addFace(Vertex3d vtx0, Vertex3d vtx1, Vertex3d vtx2) {
      PolygonalMesh mesh = (PolygonalMesh)vtx0.getMesh ();
      
      if (isCounterClockwise(new Vertex3d[]{vtx0,vtx1,vtx2})) {
         return mesh.addFace (vtx0, vtx1, vtx2);
      }
      else {
//         return mMesh.addFace (vtx0, vtx2, vtx1);
         return mesh.addFace (vtx0, vtx1, vtx2);
      }
   }
   
   public static Face createFace(Vertex3d vtx0, Vertex3d vtx1, Vertex3d vtx2) {
      if (isCounterClockwise(new Vertex3d[]{vtx0,vtx1,vtx2})) {
         return Face.create (vtx0, vtx1, vtx2);
      }
      else {
//         return mMesh.addFace (vtx0, vtx2, vtx1);
         return Face.create (vtx0, vtx2, vtx1);
      }
   }
   
   public static Face createFace(Vertex3d vtx0, Vertex3d vtx1, Vertex3d vtx2, Face refFace) {
      refFace.computeNormal ();
      Vector3d refNrm = refFace.getNormal ();
      
      return createFace(vtx0, vtx1, vtx2, refNrm);
   }
   
   public static Face createFace(Vertex3d vtx0, Vertex3d vtx1, Vertex3d vtx2, Vector3d refNrm) {
      if (MeshUtil.getNormal(vtx0, vtx1, vtx2).dot (refNrm) > 0) {
         return Face.create (vtx0, vtx1, vtx2);
      }
      else {
//         return mMesh.addFace (vtx0, vtx2, vtx1);
         return Face.create (vtx0, vtx2, vtx1);
      }
   }
   
   /**
    * A set of ordered vertices is considered counter clockwise if they form 
    * a face that has a normal that faces upwards (+z).
    */
   public static boolean isCounterClockwise(Vertex3d[] v) {
      Face checkCCWFace = Face.create (v);
//      System.out.printf ("Face normal: %.2f\n", checkCCWFace.getNormal ().z);
//      System.out.println ("isCCW: " + (checkCCWFace.getNormal ().z >= 0));
      if (Double.isNaN ( checkCCWFace.getNormal ().z)) {
         return true;
      }
      
      return (checkCCWFace.getNormal ().z >= 0);
   }
  
   
   public static Vector3d getNormal(Vertex3d... v) {
      Face face = Face.create (v);
      return face.getNormal ();
   }
   
   public static Vertex3d[] getOppositeVtxs(HalfEdge he) {      
      if (he.opposite != null) {
         return new Vertex3d[] {
            he.getNext ().head,
            he.opposite.getNext ().head
         };
      }
      else {
         return new Vertex3d[] {
           he.getNext ().head
        };
      }
   }
   
   public static boolean isBoundaryVtx(ArrayList<HalfEdge> connEdges) {
      for (HalfEdge he : connEdges) {
         if (he.opposite == null) {
            return true;
         }
      }
      return false;
   }
   
   public static boolean isBoundaryVtx(Vertex3d vtx) {
      for (HalfEdge edge : MeshUtil.getConnectedEdges (vtx)) {
          if (MeshUtil.isBoundaryEdge (edge)) {
             return true;
          }
      }
      return false;
   }
   
   public static HalfEdge[] getBoundaryEdges(ArrayList<HalfEdge> edges) {
      HalfEdge[] boundaryEdges = new HalfEdge[2];
      for (HalfEdge he : edges) {
         if (he.opposite == null) {
            if (boundaryEdges[0] == null) {
               boundaryEdges[0] = he;
            }
            else {
               boundaryEdges[1] = he;
               return boundaryEdges;
            }
         }
      }
      return null;
   }
   
   public static Vertex3d getCommonVertex(HalfEdge he0, HalfEdge he1) {
      if (he0.head == he1.head || 
          he0.head == he1.tail) {
         return he0.head;
      }
      else if (he0.tail == he1.head ||
               he0.tail == he1.tail) {
         return he0.tail;
      }
      else {
         return null;
      }
   }
   
   public static Vertex3d getOtherVertex(HalfEdge knownEdge, HalfEdge connEdge) {
      Vertex3d commVtx = MeshUtil.getCommonVertex (knownEdge, connEdge);
      if (connEdge.head == commVtx) {
         return connEdge.tail;
      }
      else {
         return connEdge.head;
      }
   }
   
   public static Vertex3d getOppositeVertex(HalfEdge boundaryEdge) {
      Face boundaryFace = boundaryEdge.getFace ();
      for (int e = 0; e < 3; e++) {
         HalfEdge curEdge = boundaryFace.getEdge (e);
         if (curEdge == boundaryEdge) {
            continue;
         }
         return getOtherVertex(boundaryEdge, curEdge);
      }
      return null;
   }
   
   public static HalfEdge getOtherEdge(HalfEdge he, Face face) {
      if (he.getFace ().idx == face.idx) {
         return he.opposite;
      }
      else {
         return he;
      }
   }
   
   /**
    * Get the four vertices of the two faces that are adjacent to the given 
    * edge.
    */
   protected static Vertex3d[] getQuadrilateralVtxs(HalfEdge edge) {
      Face face0 = edge.getFace ();
      Face face1 = edge.getOppositeFace ();
      
      Vertex3d[] vtxs = new Vertex3d[4];
      
      vtxs[0] = edge.head;
      vtxs[2] = edge.tail;

      for (Vertex3d v : face0.getTriVertices ()) {
         if (v != edge.head && v != edge.tail) {
            vtxs[1] = v;
            break;
         }
      }
      
      for (Vertex3d v : face1.getTriVertices ()) {
         if (v != edge.head && v != edge.tail) {
            vtxs[3] = v;
            break;
         }
      }
      
      return vtxs;
   }
   
   public static LinkedList<HalfEdge> getAllEdgesViaCopy(ArrayList<Face> activeFaces) {
      LinkedList<HalfEdge> rvEdges = new LinkedList<HalfEdge>();
      LinkedList<HalfEdge> visitedEdges = new LinkedList<HalfEdge>();
      
      int visitedFlag = ModelComponentBase.createTempFlag ();
      
      for (Face face : activeFaces) {
         for (int e = 0; e < 3; e++) {
            HalfEdge edge = face.getEdge (e);
            edge = MeshUtil.getEdge (edge.head, edge.tail);
            if (edge == null) 
               System.out.println ("DANMDEBUG: assert");

            if (! edge.checkFlag (visitedFlag) && (edge.opposite == null || ! edge.opposite.checkFlag (visitedFlag) ))  
            {
               edge.setFlag (visitedFlag);
               visitedEdges.add (edge);
               
               HalfEdge edgeCopy = createIndependentCopy(edge);
               rvEdges.add (edgeCopy);
            }
            
            
         }
      }
      
//      // DANMDEBUG
//      for (int k = 0; k < rvEdges.size (); k++) { 
//         for (int p = 0; p < rvEdges.size (); p++) {
//            if (k==p) continue;
//            HalfEdge kEdge = rvEdges.get (k);
//            HalfEdge pEdge = rvEdges.get (p);
//            if (MeshUtil.isShared2VtxsEdge (kEdge, pEdge)) {
//               System.out.println ("DANMDEBUG: Duplicate bad edge");
//               System.out.println (kEdge.vertexStr () + " " + pEdge.vertexStr ());
//               System.out.println ("DANMDEBUG: here");
//            }
//         }
//      }
      
      MeshUtil.clearEdgeFlags (visitedEdges, visitedFlag);
      ModelComponentBase.removeTempFlag (visitedFlag);

      return rvEdges;
   }
   
   public static HalfEdge createIndependentCopy(HalfEdge edge) {
      HalfEdge heCopy = new HalfEdge();
      heCopy.head = edge.head; 
      heCopy.tail = edge.tail; 
      return heCopy;
   }
   
   public static HalfEdge getCommonEdge(Face face0, Face face1) {
      for (int e0 = 0; e0 < 3; e0++) {
         HalfEdge edge0 = face0.getEdge (e0);
         
         for (int e1 = 0; e1 < 3; e1++) {
            HalfEdge edge1 = face1.getEdge (e1);
            
            if (MeshUtil.isShared2VtxsEdge(edge0,edge1)) {
               return edge0;
            }   
         }
      }
      
      return null;
   }
   
   public static void printEdges(PolygonalMesh mesh) {
      for (Face evenFace : mesh.getFaces ()) {
         for (int h = 0; h < 3; h++) {
            HalfEdge he = MeshUtil.getHalfEdgeWithMinHeadIdx (
               evenFace.getEdge (h));
            
//            if (he.isVisited ()) {
//               continue;
//            } 
//            else {
//               he.setVisited ();
//            }
            
            System.out.printf ("Face: %d, Edge: %d->%d\n", evenFace.idx, 
               he.tail.getIndex (), he.head.getIndex ());
         }
      }
      
      clearVisitedEdgeFlags(mesh);
   }
   
   public static void clearFaceFlags(PolygonalMesh mesh, int flag) {
      for (Face face : mesh.getFaces ()) {
         face.clearFlag (flag);
      }
   }
   
   public static void clearVisitedEdgeFlags(PolygonalMesh mesh) {
      for (Face face : mesh.getFaces ()) {
         for (int h = 0; h < 3; h++) {
            face.getEdge (h).clearVisited ();
         }
      }
   }
   
   public static void clearVisitedFaceFlags(PolygonalMesh mesh) {
      for (Face face : mesh.getFaces ()) {
         face.clearVisited ();
      }
   }
   
   public static void clearVisitedVertexFlags(PolygonalMesh mesh) {
      for (Vertex3d vtx : mesh.getVertices ()) {
         vtx.clearVisited ();
      }
   }
   
   public static void clearFaceFlags(LinkedList<Face> faces, int flag) {
      for (Face face : faces) {
         face.clearFlag (flag);
      }
   }
   
   public static void clearVertexFlags(LinkedList<Vertex3d> vtxs, int flag) {
      for (Vertex3d vtx : vtxs) {
         vtx.clearFlag (flag);
      }
   }
   
   public static void clearEdgeFlags(LinkedList<HalfEdge> edges, int flag) {
      for (HalfEdge edge : edges) {
         edge.clearFlag (flag);
      }
   }
   
   public static PolygonalMesh createHexagonalTriRectangle(
   double xLen, double yLen, int xDiv, int yDiv, boolean isShaveHalfTriangles) {
      PolygonalMesh mesh = new PolygonalMesh();
      
      // Create vertices, row-by-row
      
      int numVtxsPerDiagVtxRow = xDiv;
      int numVtxsPerNonDiagVtxRow = xDiv+1;
      
      int numVtxs = ((yDiv+1)/2+1)*numVtxsPerNonDiagVtxRow + (yDiv/2)*numVtxsPerDiagVtxRow;

      Vertex3d[] vtxs = new Vertex3d[numVtxs];
      
      double z = 0;
      double xDivLen = xLen/(xDiv-1);
      double yDivLen = yLen/yDiv;
      
      int v = 0;
      for (int vr=0; vr < yDiv+1; vr++) {
         boolean isDiagVtxRow = (vr % 2 != 0);
         
         int numVtxCols =
            (isDiagVtxRow) ? numVtxsPerDiagVtxRow
                           : numVtxsPerNonDiagVtxRow;
         
         double y = vr*yDivLen;
         
         for (int vc=0; vc < numVtxCols; vc++) {
            double x = 
               (vc == 0)            ? 0 :
               (vc == numVtxCols-1) ? xLen :
               (isDiagVtxRow)       ? vc*xDivLen 
                                    : xDivLen/2 + (vc-1)*xDivLen;

            vtxs[v] = new Vertex3d(x,y,z);
            mesh.addVertex (vtxs[v]);
            v++;
         }
      }
      
      // Create faces, row-by-row
      
      Vertex3d[] faceVtxs = new Vertex3d[3];
      
      for (int fr=0; fr < yDiv; fr++) {
         boolean is_bv_diagVtxRow = (fr%2 != 0);
         
         int bv = ((fr+1)/2)*numVtxsPerNonDiagVtxRow + ((fr)/2)*numVtxsPerDiagVtxRow;
         int tv = ((fr+2)/2)*numVtxsPerNonDiagVtxRow + ((fr+1)/2)*numVtxsPerDiagVtxRow;
       
         boolean isTriUpwards = !is_bv_diagVtxRow;
         
         int lastFace = xDiv*2-1;

         if (isShaveHalfTriangles)
            lastFace  -= 1;
         
         for (int f=0; f < lastFace; f++) {
            if (isTriUpwards) {
               if (! (isShaveHalfTriangles && f==0)) {
                  faceVtxs[0] = vtxs[tv];
                  faceVtxs[1] = vtxs[bv];
                  faceVtxs[2] = vtxs[bv+1];
               }

               bv++;
            }
            else {   // tri downwards
               if (! (isShaveHalfTriangles && f==0)) {
                  faceVtxs[0] = vtxs[tv];
                  faceVtxs[1] = vtxs[bv];
                  faceVtxs[2] = vtxs[tv+1];
               }
               
               tv++;
            }
            
            if (! (isShaveHalfTriangles && f==0))
               mesh.addFace( faceVtxs );
            
            isTriUpwards = !isTriUpwards;
         }
      }
      
      // Remove disconnected vertices
      if (isShaveHalfTriangles) {
         mesh.removeDisconnectedVertices ();
      }
         
      return mesh;
   }

   

}
