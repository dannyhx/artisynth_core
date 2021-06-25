package artisynth.demos.growth.util;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;

import artisynth.core.modelbase.ModelComponentBase;
import maspack.geometry.Face;
import maspack.geometry.HalfEdge;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Vector3d;

/**
 * Collection of standalone methods regarding mesh connectivity, traversal, and
 * others. 
 */
public class MeshUtil {

   public static String getVertexIndices(Vertex3d[] vtxs) {
      String s = "[";
      
      for (Vertex3d vtx : vtxs) {
         s += vtx.getIndex () + "-";
      }
      
      s = s.substring (0, s.length ()-1);
      s += "]";
      
      return s;
   }
   
   /**
    * Get faces that are connected to given vertex.
    */
   public static ArrayList<Face> getOneRingFaces(Vertex3d vtx) {
      ArrayList<Face> ring = new ArrayList<Face>();
      
      Iterator<HalfEdge> adjEdgeIter = vtx.getIncidentHalfEdges ();
      while (adjEdgeIter.hasNext ()) {
         HalfEdge adjEdge = adjEdgeIter.next ();
         ring.add ( adjEdge.getFace () );
      }
      
      return ring;
   }
   
   /**
    * Get edges that are connected to given vertex. 
    */
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
   
   /**
    * Get vertices that are connected to the given vertex. 
    */
   public static ArrayList<Vertex3d> getConnectedVertices(Vertex3d vtx) {
      ArrayList<Vertex3d> connVtxs = new ArrayList<Vertex3d>();

      for (HalfEdge edge : getConnectedEdges(vtx)) {
         connVtxs.add( MeshUtil.getOtherVertex (vtx, edge) );
      }
      
      return connVtxs;
   }
   
   /**
    * Do the two given vertices share a common edge? 
    */
   public static boolean isConnected(Vertex3d A, Vertex3d B) {
      ArrayList<Vertex3d> adjVtxsToA = getConnectedVertices(A);
      for (Vertex3d adjVtx : adjVtxsToA) {
         if (adjVtx == B) 
            return true;
      }
      return false;
   }
   
   /**
    * Is the given edge and vertex connected? 
    */
   public static boolean isEdgeAndVtxConnected(HalfEdge edge, Vertex3d vtx) {
      return (edge.head == vtx || edge.tail == vtx);
   }
   
   /**
    * Is only one side of the edge connected to a face?  
    */
   public static boolean isBoundaryEdge(HalfEdge edge) {
      return (edge.getOppositeFace () == null);
   }
   
   /**
    * Does face have a boundary edge?
    */
   public static boolean isBoundaryFace(Face face) {
      for (int e = 0; e < 3; e++) {
         if (MeshUtil.isBoundaryEdge (face.getEdge (e))) {
            return true;
         }
      }
      
      return false;
   }

   /**
    * Given an edge and one of its vertex, get the edge's other vertex. 
    */
   public static Vertex3d getOtherVertex(Vertex3d skipVtx, HalfEdge edge) {
      if (edge.head == skipVtx) {
         return edge.tail;
      }
      else {
         return edge.head;
      }
   }
   
   /**
    * Is the vertex connected to the face? 
    */
   public static boolean isHasVertex(Vertex3d vtx, Face face) {
      for (Vertex3d curVtx : face.getVertices ()) {
         if (vtx == curVtx) {
            return true;
         }
      }
      return false;
   }
   
   /**
    * Is vertex connected to edge? 
    */
   public static boolean isHasVertex(Vertex3d vtx, HalfEdge edge) {
      return (edge.head == vtx || edge.tail == vtx); 
   }
   
   /**
    * Are the two half edges part of the same edge?
    */
   public static boolean isShared2VtxsEdge(HalfEdge edge0, HalfEdge edge1) {
      return (
         (edge0.head == edge1.head && edge0.tail == edge1.tail) ||
         (edge0.head == edge1.tail && edge0.tail == edge1.head) 
      );
   }
   
   /**
    * Are the two half edges equal in terms of vertices and direction? 
    */
   public static boolean isSameHalfEdge(HalfEdge edge0, HalfEdge edge1) {
      return (edge0.head == edge1.head && edge0.tail == edge1.tail);
   }
   
   /**
    * Get the edge that is connected to the two given vertices.
    * 
    * @return null if edge does not exist.
    */
   public static HalfEdge getEdge(Vertex3d a, Vertex3d b) {
      for (HalfEdge edge : getConnectedEdges (a)) {
         if (isHasVertex(b, edge)) {
            return edge;
         }
      }
      return null;
   }
   
   /**
    * Get the edge that is connected to the two given vertices.
    * 
    * Vertices are specified by their indices.
    * 
    * @return null if edge does not exist.
    */
   public static HalfEdge getEdge(int a, int b, PolygonalMesh mesh) {
      return getEdge( mesh.getVertex (a), mesh.getVertex (b) );
   }
   
   /**
    * Given a half edge, return either the same half edge or its sibling 
    * half edge. The half edge that is returned is the one with the 
    * smallest head vertex index.  
    * 
    * Function is handy when searching across the edges of a mesh without
    * needing to inspect two half edges per edge. 
    */
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
   
   /**
    * Are the two given half edges share a common vertex? 
    */
   public static boolean isShareVertex(HalfEdge he0, HalfEdge he1) {
      return (
         he0.head == he1.head || 
         he0.head == he1.tail || 
         he0.tail == he1.head || 
         he0.tail == he1.tail
      );
   }
   
   /**
    * Is the edge part of the given face?
    * 
    * Edge is specified using any of its two half edges. 
    */
   protected static boolean hasEdge(Face face, HalfEdge edge) {
      for (int i = 0; i < 3; i++) {
         if (face.getEdge (i) == edge || face.getEdge (i) == edge.opposite) {
            return true;
         }
      }
      return false;
   }
   
   /**
    * Add a new face to the mesh by specifying 3 vertices of the mesh.
    * 
    * @return The added face.
    */
   public static Face addFace(Vertex3d vtx0, Vertex3d vtx1, Vertex3d vtx2) {
      PolygonalMesh mesh = (PolygonalMesh)vtx0.getMesh ();
      
      if (isCounterClockwise(new Vertex3d[]{vtx0,vtx1,vtx2})) {
         return mesh.addFace (vtx0, vtx1, vtx2);
      }
      else {
         return mesh.addFace (vtx0, vtx1, vtx2);
      }
   }
   
   /**
    * Create a face by specifying 3 vertices from the mesh.
    * 
    * Created face is NOT added to the mesh; it is only returned.
    */
   public static Face createFace(Vertex3d vtx0, Vertex3d vtx1, Vertex3d vtx2) {
      if (isCounterClockwise(new Vertex3d[]{vtx0,vtx1,vtx2})) {
         return Face.create (vtx0, vtx1, vtx2);
      }
      else {
         return Face.create (vtx0, vtx2, vtx1);
      }
   }
   
   /**
    * Create a face by specifying 3 vertices from the mesh.
    * 
    * @param refFace
    * Vertex ordering of the face will be adjusted such that the new face's
    * normal is pointing at the same plane of the refFace.
    */
   public static Face createFace(Vertex3d vtx0, Vertex3d vtx1, Vertex3d vtx2, Face refFace) {
      refFace.computeNormal ();
      Vector3d refNrm = refFace.getNormal ();
      
      return createFace(vtx0, vtx1, vtx2, refNrm);
   }
   
   /**
    * Create a face by specifying 3 vertices from the mesh.
    * 
    * @param refNrm
    * Vertex ordering of the face will be adjusted such that the new face's
    * normal and refNrm are pointing at the same plane..
    */
   public static Face createFace(Vertex3d vtx0, Vertex3d vtx1, Vertex3d vtx2, Vector3d refNrm) {
      if (MeshUtil.getNormal(vtx0, vtx1, vtx2).dot (refNrm) > 0) {
         return Face.create (vtx0, vtx1, vtx2);
      }
      else {
         return Face.create (vtx0, vtx2, vtx1);
      }
   }
   
   /**
    * A set of ordered vertices is considered counter clockwise if they form 
    * a face that has a normal that faces upwards (+z).
    */
   public static boolean isCounterClockwise(Vertex3d[] v) {
      Face checkCCWFace = Face.create (v);

      if (Double.isNaN ( checkCCWFace.getNormal ().z)) {
         return true;
      }
      
      return (checkCCWFace.getNormal ().z >= 0);
   }
  
   /**
    * Get the normal of the given face, which is specified using vertices.
    */
   public static Vector3d getNormal(Vertex3d... v) {
      Face face = Face.create (v);
      return face.getNormal ();
   }
   
   /**
    * Given an edge that is shared by two faces, get the other 2 vertices of the 
    * faces that aren't connected to the edge. 
    * 
    * If edge is only part of a single face, return the vertex of the face that 
    * isn't connected to the edge.
    */
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
   
   /**
    * Is the vertex on the boundary of the mesh?
    */
   public static boolean isBoundaryVtx(Vertex3d vtx) {
      for (HalfEdge edge : MeshUtil.getConnectedEdges (vtx)) {
          if (MeshUtil.isBoundaryEdge (edge)) {
             return true;
          }
      }
      return false;
   }

   public static boolean isBoundaryVtx(ArrayList<HalfEdge> connEdges) {
      for (HalfEdge he : connEdges) {
         if (he.opposite == null) {
            return true;
         }
      }
      return false;
   }
   
   /**
    * Given a set of edges, return the edges that are on the boundary of the 
    * mesh.
    */
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
   
   /**
    * Get the vertex that is shared by the two given edges. 
    */
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
   
   /**
    * Given a known edge and a connected edge, get the vertex of the connected 
    * edge that isn't shared with the known edge.
    */
   public static Vertex3d getOtherVertex(HalfEdge knownEdge, HalfEdge connEdge) {
      Vertex3d commVtx = MeshUtil.getCommonVertex (knownEdge, connEdge);
      if (connEdge.head == commVtx) {
         return connEdge.tail;
      }
      else {
         return connEdge.head;
      }
   }
  
   
   /**
    * Among the 3 adjacent faces of the given face, return the adjacent face 
    * that contains the given vertex.
    * 
    * @param excludeFace
    * Must not return this face.  
    */
   public static Face getAdjacentFace(
   Face curFace, Face excludeFace, Vertex3d reqVtx) {
      //
      for (int e = 0; e < 3; e++) {
         HalfEdge edge = curFace.getEdge (e);
         Face oppFace = edge.getOppositeFace ();  // getFace() is just curFace
         if (oppFace != null && MeshUtil.isHasVertex (reqVtx, oppFace) && oppFace != excludeFace) {
            return oppFace;
         }
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
    * Given two faces (specified by their common edge), return the four 
    * vertices of the two faces. 
    */
   public static Vertex3d[] getQuadrilateralVtxs(HalfEdge edge) {
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
   
   /**
    * Get a copy of the edges that comprises the given faces.
    */
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
      
      MeshUtil.clearEdgeFlags (visitedEdges, visitedFlag);
      ModelComponentBase.removeTempFlag (visitedFlag);

      return rvEdges;
   }
   
   /**
    * Create a half edge copy.
    */
   public static HalfEdge createIndependentCopy(HalfEdge edge) {
      HalfEdge heCopy = new HalfEdge();
      heCopy.head = edge.head; 
      heCopy.tail = edge.tail; 
      return heCopy;
   }
   
   /**
    * Get the edge that is shared between two adjacent faces.
    * 
    * @return 
    * null if faces are not adjacent.
    */
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
   
   /**
    * Print the edges of a mesh.
    */
   public static void printEdges(PolygonalMesh mesh) {
      for (Face evenFace : mesh.getFaces ()) {
         for (int h = 0; h < 3; h++) {
            HalfEdge he = MeshUtil.getHalfEdgeWithMinHeadIdx (
               evenFace.getEdge (h));
            
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
   
   /**
    * Create a mesh with hexagonal-cells using triangles only. 
    * 
    * @param isShaveHalfTriangles
    * If true, boundary of mesh will be shaven to become rectangular.
    */
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

   /**
    * Create a disc mesh. 
    */
   public static PolygonalMesh createUniformDisc(int res, int circRes) {
      double d = 1.0/res;
      
      LinkedList<Vertex3d> vtxs = new LinkedList<Vertex3d>();
      vtxs.add ( new Vertex3d() );      // Center
      
      LinkedList<Integer> faceIdxs = new LinkedList<Integer>();
      
      // Create vertices for each circle
      for (int circ = 0; circ < res; circ++) {
         int numTri = (circ+1)*circRes;
         // 2PI / 6
         // 2PI / 12
         double angleStep = ((Math.PI * 2) / numTri);
         
         for (int point = 0; point < numTri; point++) {
            Vertex3d vtx = new Vertex3d(
               Math.cos (angleStep * point),
               Math.sin (angleStep * point),
               0);
            vtx.pnt.scale (d * (circ+1));
            
            vtxs.add ( vtx );
         }
      }
      
      // Connect vertices into triangles
      for (int circ = 0; circ < res; circ++) {
         int numTri = (circ+1)*circRes;
         
         int other = 0;
         for (int point = 0; point < numTri; point++) {
            if (point % (circ+1) != 0) {
               // Create 2 triangles 
               faceIdxs.add ( _getPointIndex(circ-1, other+1, circRes) );
               faceIdxs.add ( _getPointIndex(circ-1, other, circRes) );
               faceIdxs.add ( _getPointIndex(circ, point, circRes) );
               
               faceIdxs.add ( _getPointIndex(circ, point, circRes) );
               faceIdxs.add ( _getPointIndex(circ, point+1, circRes) );
               faceIdxs.add ( _getPointIndex(circ-1, other+1, circRes) );
               
               other++;
            }
            else {
               faceIdxs.add ( _getPointIndex(circ, point, circRes) );
               faceIdxs.add ( _getPointIndex(circ, point+1, circRes) );
               faceIdxs.add ( _getPointIndex(circ-1, other, circRes) );
            }
            
         }
      }
      
      // Create mesh
      PolygonalMesh mesh = new PolygonalMesh();
      
      for (Vertex3d vtx : vtxs) 
         mesh.addVertex (vtx);
      
      for (int i = 0; i < faceIdxs.size(); i+=3) 
         mesh.addFace (new int[] {
            faceIdxs.get(i),         
            faceIdxs.get(i+1), 
            faceIdxs.get(i+2)
         });
      
      return mesh;
   }

   protected static int _getPointIndex(int c, int x, int circRes) {
      if (c < 0) return 0;   // Center point
      x = x % ((c+1)*circRes);   // Make point index ciricular 
      return ((circRes/2)*c*(c+1)+x+1);
   }
   
}
