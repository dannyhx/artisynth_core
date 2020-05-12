package artisynth.demos.growth.collision;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.LinkedList;

import artisynth.core.femmodels.FemMeshComp;
import maspack.geometry.AABBTree;
import maspack.geometry.BVNode;
import maspack.geometry.Face;
import maspack.geometry.HalfEdge;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderableUtils;

/** Space-time traversal of a mesh. */
public class SweptMeshInfo {
   
   public PolygonalMesh myMesh;
      
   /** Previous vertex positions in the mesh's local-space. */
   public Point3d[] prevPositions;      
   
   /** 
    * Previous vertex positions in the mesh's local-space {@link #prevPositions}
    * that have been pre-transformed such that if a "to current world-space" 
    * matrix is applied, the vertex positions in the "previous world-space" is
    * actually obtained (i.e. previous vertex positions in world-space).
    * 
    * This allows BVTree intersection methods to rely on a single matrix
    * (i.e. provided by myMesh.getMeshToWorld()) only, instead of needing 
    * to refactor the BVTree to account for 2 different transformation 
    * matrices to obtain either the previous and current world-space vertex
    * positions.
    * 
    * The comments in {@link #updatePrevPositionsX0()} provides more 
    * details regarding this property. 
    */
   public Point3d[] prevPositionsX0;    
   
   /** Matrix that can transform {@link #previousPositions} from local-space
    *  to the previous world-space. In other words, this is the 
    *  "to world-space" transformation matrix that was used in the previous 
    *  time step.  */
   public RigidTransform3d X0 = new RigidTransform3d();
   
   /** Space-time traversal of the vertices, organized in a AABB tree. */
   public AABBTree myVertexTree;
   
   /** Space-time traversal of the edges, organized in a AABB tree. */
   public AABBTree myEdgeTree;
   
   /** Space-time traversal of the triangles, organized in a AABB tree. */
   public AABBTree myTriangleTree;
   
   public static double myMargin = 0.6e-2;    // Half of cloth thickness

   
   
   public SweptMeshInfo (PolygonalMesh mesh) {
      build(mesh);
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
      
      myVertexTree = new AABBTree();
      myVertexTree.setMaxLeafElements(1);
      myVertexTree.setMargin (myMargin);
      myVertexTree.build (sweptVertices, sweptVertices.length);

      myEdgeTree = new AABBTree();
      myEdgeTree.setMaxLeafElements(1);
      myEdgeTree.setMargin (myMargin);
      myEdgeTree.build (sweptEdges, sweptEdges.length);

      myTriangleTree = new AABBTree();
      myTriangleTree.setMaxLeafElements(1);
      myTriangleTree.setMargin (myMargin);
      myTriangleTree.build (sweptTriangles, sweptTriangles.length);
   }
   
   /** Update the previous vertex positions to the current vertex positions.
    * 
    * Also update the previous "to world-space" transformation matrix to the 
    * current one.  
    */
   public void copyCurrent2PreviousPositions() {
      int numv = myMesh.numVertices();
      for (int i=0; i<numv; i++) {
         prevPositions[i].set (myMesh.getVertex(i).getPosition());
      }

      X0.set( myMesh.getMeshToWorld () ); 
   }
   
   /** Update {@link #prevPositionsX0}, which become the previous vertex 
    *  positions in world-space when the current "to world-space" 
    *  transformation matrix is applied. */
   public void updatePrevPositionsX0() {
      for (int p=0; p<prevPositionsX0.length; p++) {
         // Get world-space of previous position.
         Point3d ptPrevW = new Point3d( prevPositions[p] );
         ptPrevW.transform (X0);
         
         // However, when AABB tests are carried out, the AABB tests
         // automatically applies the "to current world-space" matrix.
         // ---> We don't want ptPrevW to be further transformed.
         
         // Therefore, apply inverse of the "to current world-space" matrix.
         // That way, when the AABB tests automatically apply the 
         // "to current world-space" matrix, we end up with the original
         // ptPrevW.
         prevPositionsX0[p].set(ptPrevW);
         prevPositionsX0[p].inverseTransform (myMesh.getMeshToWorld ());
      }
   }
   
   public void updateBVTrees() {
      myVertexTree.update();
      myEdgeTree.update();
      myTriangleTree.update();
   }
   
   
   
   
   /* --- Functions for Continuous Collision Response --- */
   
   public void saveCurrentPositions() {
      for (BoundablePointArray bnd : getSweptBoundables()) {
         bnd.saveCurrentPositions ();
      }
   }
   
   public void loadCurrentPositions() {
      for (BoundablePointArray bnd : getSweptBoundables()) {
         bnd.loadCurrentPositions ();
      }
   }
   
   public void copyPrevious2CurrentPositions() {
      for (BoundablePointArray bnd : getSweptBoundables()) {
         bnd.copyPrevious2CurrentPositions ();
      }
   }
   
//   public void computeAvgVelocities(double t) {
//      for (BoundablePointArray bnd : getSweptBoundables()) {
//         bnd.computeAvgVelocity (t);
//      }
//   }
   
   /* --- Helper Functions --- */
   
   public ArrayList<BoundablePointArray> getSweptBoundables() {
      ArrayList<BVNode> bvNodes = new ArrayList<BVNode>();
      bvNodes.addAll ( myVertexTree.getLeafNodes () );
      bvNodes.addAll ( myEdgeTree.getLeafNodes () );
      bvNodes.addAll ( myTriangleTree.getLeafNodes () );
      
      ArrayList<BoundablePointArray> bnds = 
         new ArrayList<BoundablePointArray>( bvNodes.size () );
      
      for (BVNode node : bvNodes) {
         bnds.add( (BoundablePointArray)node.getElements ()[0] );
      }
      
      return bnds;
   }
   
   
}
