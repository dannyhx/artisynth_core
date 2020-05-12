package artisynth.demos.growth.remesh;

import java.util.ArrayList;

import artisynth.demos.growth.util.MathUtil;
import artisynth.demos.growth.util.MeshUtil;
import maspack.geometry.Face;
import maspack.geometry.HalfEdge;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Vector3d;

/**
 * Remeshing operations (e.g. bisect, flip, collapse) for a polygonal mesh.
 */
public class RemeshOps {
   
   public boolean isDebug = false;
   
   protected PolygonalMesh mMesh;
   
   public RemeshOps(PolygonalMesh mesh) {
      setTarget(mesh);
   }
   
   protected void setTarget(PolygonalMesh mesh) {
      mMesh = mesh;
   }
   
   /* --- Primary Remeshing Operations --- */
   
   /**
    * Bisect the given edge into two edges.
    * 
    * @return
    * Information regarding the bisection (e.g. new faces that are created).
    */
   public OpRv bisectEdge(HalfEdge he) {
      OpRv rv = new OpRv(4,2);
      
      Vertex3d vtxH = he.head; 
      Vertex3d vtxT = he.tail;
      
      Face locFace = he.getFace ();
      Face oppFace = he.getOppositeFace ();

      // Get node opposite of edge in local face
      Vertex3d locEleOppVtx = he.getNext ().getHead ();
      
      // Get node opposite of edge in opposite face
      Vertex3d oppEleOppVtx = null;
      if (oppFace != null) {
         oppEleOppVtx = he.opposite.getNext ().getHead ();
      }
      
      rv.mNewVertex = addVertexViaLinearInterpolation (vtxH, vtxT);
      
      // Replace local face with two smaller faces, using new edge between
      // new vertex and edge's opposite vertex.
      
      Object locRemoved = removeFace (locFace);
      rv.mRemovedFaces[0] = locFace;
      rv.mAddedFaces[0] = MeshUtil.createFace (locEleOppVtx, rv.mNewVertex, vtxH, locFace);
      rv.mAddedFaces[1] = MeshUtil.createFace (locEleOppVtx, vtxT, rv.mNewVertex, locFace);
      
      addFace(rv.mAddedFaces[0], locRemoved);
      addFace(rv.mAddedFaces[1], locRemoved);
      
      // Do same for opposite face
      
      if (oppFace != null) {
         Object oppRemoved = removeFace (oppFace);
         rv.mRemovedFaces[1] = oppFace;
         rv.mAddedFaces[2] = MeshUtil.createFace (oppEleOppVtx, vtxH, rv.mNewVertex, oppFace);
         rv.mAddedFaces[3] = MeshUtil.createFace (oppEleOppVtx, rv.mNewVertex, vtxT, oppFace);
         addFace(rv.mAddedFaces[2], oppRemoved);
         addFace(rv.mAddedFaces[3], oppRemoved);
      }
      
      return rv;
   }

   /**
    * Given an edge that is shared between two adjacent faces, "move" the edge
    * such that it connects to the other 2 vertices of the adjacent faces.
    * 
    * @return
    * Information regarding the flip (e.g. new faces that are created).
    */
   public OpRv flip(HalfEdge edge) {
      OpRv rv = new OpRv(2,2);
      
      rv.mRemovedFaces[0] = edge.getFace ();
      rv.mRemovedFaces[1] = edge.getOppositeFace ();
      
      rv.mRemovedEdge = MeshUtil.getCommonEdge(rv.mRemovedFaces[0], rv.mRemovedFaces[1]);
      
      removeFace (rv.mRemovedFaces[0]);
      removeFace (rv.mRemovedFaces[1]);
      
      Vertex3d[] quadVtxs = MeshUtil.getQuadrilateralVtxs(edge);
      
      // Each new face contains only one vertex from the edge to be flipped.
      
      rv.mAddedFaces[0] = MeshUtil.createFace (quadVtxs[0], quadVtxs[1], quadVtxs[3], rv.mRemovedFaces[0]) ;
      rv.mAddedFaces[1] = MeshUtil.createFace (quadVtxs[3], quadVtxs[1], quadVtxs[2], rv.mRemovedFaces[0]) ;  
      
      addFace (rv.mAddedFaces[0], null);
      addFace (rv.mAddedFaces[1], null);
      
      rv.mAddedEdge = MeshUtil.getCommonEdge(rv.mAddedFaces[0], rv.mAddedFaces[1]);
      
      return rv;
   }
   
   /**
    * Collapse and remove edge a-b.
    * 
    * @param rVtx
    * Vertex, which is part of edge, to be removed. Represented as 'b'
    * 
    * @param useVtxAvg 
    * If true, vertex that won't be removed (i.e. 'a') will be set as the 
    * average of 'a' and 'b'.
    */
   public OpRv collapseEdge(HalfEdge e, Vertex3d rVtx, boolean useVtxAvg) {
      Vertex3d a = MeshUtil.getOtherVertex (rVtx, e);
      Vertex3d b = rVtx;                                // Vertex to delete
         
      // Collapse 'a' into 'b' along e
       
      // 1. Remove faces (1 or 2) along e
       
      Face face = e.getFace ();
      removeFace (face);
      
      Vector3d refNrm = new Vector3d();
      face.computeNormal (refNrm);
       
      Face oppFace = e.getOppositeFace ();
      if (oppFace != null) removeFace (oppFace);
       
      // 2. (OPTIONAL) Modify 'a' to midpoint
     
      if (useVtxAvg) {
         setVertexViaLinearInterpolation(a, b, a);
      }
      
      // 4. Replace faces of 'b' with new faces that connect to 'a'
       
      ArrayList<Face> newFacesB = new ArrayList<Face>();
       
      ArrayList<Face> faceRingB = MeshUtil.getOneRingFaces(b);
      
      for (Face faceB : faceRingB) {
         Vertex3d[] triVtxs = faceB.getTriVertices ();
         for (int i = 0; i < 3; i++) {
            if (triVtxs[i] == b) {
               triVtxs[i] = a;
               Face newFace = MeshUtil.createFace (triVtxs[0], triVtxs[1], triVtxs[2], refNrm);
               
               newFacesB.add (newFace);
               break;
            }
         }
      }
      
      ArrayList<Object> removedRingObjB = new ArrayList<Object>();
      
      for (Face faceToDelete : faceRingB) {
         Object removedObj = removeFace (faceToDelete);
         removedRingObjB.add (removedObj);
      }
       
      for (Face faceToAdd : newFacesB) {
         addFace (faceToAdd, removedRingObjB.remove (0));
      }
       
      removeVertex (b);

      OpRv rv = new OpRv();
      rv.mAddedFaces = newFacesB.toArray (new Face[0]);
      rv.mRemovedFaces = faceRingB.toArray (new Face[0]);
      rv.mNewVertex = null;
      return rv;
   }
   
   /* --- Secondary Functions --- */
   
   protected void setVertexViaLinearInterpolation(Vertex3d v0, Vertex3d v1, Vertex3d vTgt) {
      vTgt.setPosition ( MathUtil.avg(v0.pnt,v1.pnt) );
   }
   
   protected Vertex3d addVertexViaLinearInterpolation(Vertex3d v0, Vertex3d v1) {
      Vertex3d newVtx = new Vertex3d();
      mMesh.addVertex (newVtx);
      
      setVertexViaLinearInterpolation(v0, v1, newVtx);
      
      return newVtx;
   }
   
   protected void removeVertex(Vertex3d vtx) {
      mMesh.removeVertex (vtx);
   }
   
   protected Object addFace(Face face, Object removedParent) {
      Face newFace = mMesh.addFace (face.getTriVertices ());
      return newFace;
   }
   
   protected Object removeFace(Face face) {
      mMesh.removeFace (face);
      return face;
   }
  
   
  
   
   /* --- Helper Classes --- */

   /**
    * Stores information regarding the effected mesh features for a given
    * remeshing operation.
    */
   protected class OpRv {
      public Face[] mAddedFaces;
      public Face[] mRemovedFaces;
      
      // Flip operation
      public HalfEdge mAddedEdge;
      public HalfEdge mRemovedEdge;
      
      // Bisect operation and linear interpolation
      public Vertex3d mNewVertex;
      
      public OpRv() {}
      
      public OpRv(int numAddedFaces, int numRemovedFaces) {
         mAddedFaces = new Face[numAddedFaces];
         mRemovedFaces = new Face[numRemovedFaces];
      }
      
      public ArrayList<Face> getAddedFacesAsList() {
         int numAddedFaces = 0;
         for (int i = 0; i < mAddedFaces.length; i++) {
            if (mAddedFaces[i] != null) {
               numAddedFaces++;
            }
         }
         
         ArrayList<Face> rv = new ArrayList<Face>(numAddedFaces);
         
         for (int i = 0; i < mAddedFaces.length; i++) {
            if (mAddedFaces[i] != null) {
               rv.add (mAddedFaces[i]);
            }
         }
         
         return rv;
      }
      
      public boolean isEmpty() {
         return (mAddedFaces == null && mRemovedFaces == null) ||
                (mAddedFaces.length == 0 && mRemovedFaces.length == 0);
      }
   }
   
   
   
   /* --- Interfacing --- */
   
   public PolygonalMesh getMesh() {
      return this.mMesh;
   }

}
