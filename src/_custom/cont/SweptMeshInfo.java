package _custom.cont;

import java.util.ArrayList;
import java.util.LinkedHashSet;

import maspack.geometry.AABBTree;
import maspack.geometry.Face;
import maspack.geometry.HalfEdge;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderableUtils;

public class SweptMeshInfo {
   
   public PolygonalMesh myMesh;
   
   /** Previous positions of vertices in local-space. */
   public Point3d[] prevPositions;      
   
   /** Transformed instance of {@link #prevPositions} such that if they are
    * transformed to the current world-space of {@link #myMesh}, the positions 
    * in the previous world-space of {@link #myMesh} is actually obtained.
    * This allows BVTree intersection methods to rely on myMesh.getMeshToWorld()
    * only rather than needing to be refactored to account for 2 different
    * transformation matrices (previous and current). */
   public Point3d[] prevPositionsX0;    
   
   /** World-space transformation matrix of {@link #myMesh} in the previous
    * timestep. */
   public RigidTransform3d X0 = new RigidTransform3d();
   
   public AABBTree myVertexTree;
   public AABBTree myEdgeTree;
   public AABBTree myTriangleTree;

   public SweptMeshInfo (PolygonalMesh mesh) {
      build(mesh);
   }

   public void build() {
      build (myMesh);
   }
   
   public void build(PolygonalMesh mesh) {
      if (!mesh.isTriangular()) {
         throw new IllegalArgumentException (
            "Mesh is not triangular");
      }
      
      myMesh = mesh;
  
      /* --- Vertices --- */
      
      int numv = mesh.numVertices();
      prevPositions = new Point3d[numv];
      prevPositionsX0 = new Point3d[numv];
      
      for (int i=0; i<numv; i++) {
         prevPositions[i] = new Point3d(mesh.getVertex(i).getPosition());
         prevPositionsX0[i] = new Point3d(mesh.getVertex(i).getPosition());
      }
      
      SweptVertex[] sweptVertices = new SweptVertex[numv];
      for (int i=0; i<numv; i++) {
         sweptVertices[i] = new SweptVertex(mesh.getVertex(i), prevPositionsX0);
      }
      
      /* --- Faces --- */
      
      int numf = mesh.numFaces();
      SweptTriangle[] sweptTriangles = new SweptTriangle[numf];
      
      // Collect edges along the way
      ArrayList<HalfEdge> edges = new ArrayList<HalfEdge>();
      
      for (int i=0; i<numf; i++) {
         Face face = mesh.getFace(i);
         sweptTriangles[i] = new SweptTriangle(face, prevPositionsX0);
         
         for (int e=0; e<3; e++) {
            HalfEdge he = face.getEdge (e);
            int h = he.getHead ().getIndex ();
            int t = he.getTail ().getIndex ();
           
            if (h < t && he.getOppositeFace () != null)
               continue;
            
            edges.add (he);
         }
      }
      
      /* --- Edges --- */
      
      int nume = edges.size();
      SweptEdge[] sweptEdges = new SweptEdge[nume];
      int i = 0;
      for (HalfEdge he : edges) {
         sweptEdges[i++] = new SweptEdge(he, prevPositionsX0);
      }
      
      /* --- Assemble AABB trees --- */
      
      double margin = 1e-6*RenderableUtils.getRadius(mesh);
      
      myVertexTree = new AABBTree();
      myVertexTree.setMaxLeafElements(1);
      myVertexTree.setMargin (margin);
      myVertexTree.build (sweptVertices, sweptVertices.length);

      myEdgeTree = new AABBTree();
      myEdgeTree.setMaxLeafElements(1);
      myEdgeTree.setMargin (margin);
      myEdgeTree.build (sweptEdges, sweptEdges.length);

      myTriangleTree = new AABBTree();
      myTriangleTree.setMaxLeafElements(1);
      myTriangleTree.setMargin (margin);
      myTriangleTree.build (sweptTriangles, sweptTriangles.length);
   }
   
   public void savePrevPositions() {
      int numv = myMesh.numVertices();
      for (int i=0; i<numv; i++) {
         prevPositions[i].set (myMesh.getVertex(i).getPosition());
      }

      X0.set( myMesh.getMeshToWorld () ); 
   }
   
   public void updatePrevPositionsX0() {
      for (int p=0; p<prevPositionsX0.length; p++) {
         // Get world-space of previous position
         Point3d ptPrevW = new Point3d( prevPositions[p] );
         ptPrevW.transform (X0);
         
         // Apply inverse of meshToWorld. When AABB performs intersection 
         // lookup, it will apply meshToWorld, obtaining the world-space
         // of previous positions of the previous timestep.
         prevPositionsX0[p].set(ptPrevW);
         
         prevPositionsX0[p].inverseTransform (myMesh.getMeshToWorld ());
      }
   }
   
   public void updateBVTrees() {
      myVertexTree.update();
      myEdgeTree.update();
      myTriangleTree.update();
   }
}
