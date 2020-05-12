package _custom.cont;

import java.awt.Color;
import java.util.ArrayList;

import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.StepAdjustment;
import artisynth.core.workspace.RootModel;
import _custom.cont.MeshUtil;
import maspack.collision.EdgeEdgeContact;
import maspack.collision.PenetratingPoint;
import maspack.geometry.BVNode;
import maspack.geometry.Face;
import maspack.geometry.HalfEdge;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.Renderer.DrawMode;
import maspack.render.Renderer.FaceStyle;

public class ContinuousCollisionsMeshTest extends RootModel {
   
   /* --- Models --- */
   
   protected static int mNumModels = 1;
   
   protected PolygonalMesh[] mMesh;
   protected RigidBody[] mRB;
   
   protected PolygonalMesh[] mMesh_prev;
   protected RigidBody[] mRB_prev;
   
   protected MechModel mMechModel;
   
   /* --- Model Rendering --- */
   
   protected RenderProps[] mRB_rp;
   protected RenderProps[] mRB_prev_rp;
   protected RenderProps mMech_rp;
   
   /* --- Modules --- */

   protected CollisionDetector mContCldr;
   protected SweptMeshInfo[] mSMI;
   
   /* --- UI --- */
   
   protected ControlPanel mPanel;
   

   
   
   public void build (String[] args) {
      build_modelSkeleton();
      build_modelAssemble();
      build_modelProperties();
      build_environment();
      build_rendering();
      build_modules();
      build_assembleUI();
      build_addUI();
      build_post();
   }    
   
   protected void build_modelSkeleton() {
      mMesh = new PolygonalMesh[mNumModels];
      mMesh_prev = new PolygonalMesh[mNumModels];
      
      double boxSize0 = 3.0;
      double boxSize1 = 3.0;
      
      mMesh[0] = MeshFactory.createPlane (10, 10, 10, 10);
      mMesh_prev[0] = MeshFactory.createPlane (10, 10, 10, 10);
      
      mMesh[0].translate (new Vector3d(0,0,5));
      
//      mMesh[0] = MeshFactory.createIcosahedralSphere (3,3);
//      mMesh_prev[0] = MeshFactory.createIcosahedralSphere (3,3);
      
//      mMesh[1] = MeshFactory.createBox (boxSize1, boxSize1, boxSize1,             0, 0, 0);
//      mMesh_prev[1] = MeshFactory.createBox (boxSize1, boxSize1, boxSize1,        0, 0, 0);
      
//      mMesh[2] = new PolygonalMesh();  mMesh[2].addVertex (10, 0, 0);
//      mMesh_prev[2] = new PolygonalMesh();  mMesh_prev[2].addVertex (10, 5, 0);
   }
   
   protected void build_modelAssemble() {
      mMechModel = new MechModel("mech");
      
      mRB = new RigidBody[mNumModels];
      mRB_prev = new RigidBody[mNumModels];
      
      for (int i=0; i<mNumModels; i++) {
         mRB[i] = new RigidBody("RB"+i);
         mRB_prev[i] = new RigidBody("RB_prev"+i);
         
         mRB[i].setSurfaceMesh (mMesh[i]);
         mRB_prev[i].setSurfaceMesh (mMesh_prev[i]);
         
         mMechModel.addRigidBody (mRB[i]);
         mMechModel.addRigidBody (mRB_prev[i]);
      }

      addModel(mMechModel);
   }
   
   protected void build_modelProperties() {
   }
   
   protected void build_environment() {
      mMechModel.setDynamicsEnabled (false);
   }
   
   protected void build_rendering() {
      mRB_rp = new RenderProps[mNumModels];
      mRB_prev_rp = new RenderProps[mNumModels];
      
      for (int i=0; i<mNumModels; i++) {
         mRB_rp[i] = mRB[i].getRenderProps ();
         mRB_prev_rp[i] = mRB_prev[i].getRenderProps ();
         mRB_prev_rp[i].setFaceColor (Color.CYAN);
      }
      
      mMech_rp = mMechModel.getRenderProps ();
      
      mMech_rp.setFaceColor (Color.MAGENTA);
      mMech_rp.setAlpha (1);
      mMech_rp.setDrawEdges (true);
   }
   
   protected void build_modules() {
      mContCldr = new CollisionDetector();

      mSMI = new SweptMeshInfo[mNumModels];
      
      for (int i=0; i<mNumModels; i++) 
         mSMI[i] = new SweptMeshInfo(mMesh[i]);
   }
   
   protected void build_assembleUI() {
      mPanel = new ControlPanel("cp");
      
      mPanel.addWidget (this, "Enable_broad_edge_collision");
      mPanel.addWidget (this, "Enable_broad_face_collision");
      
      mPanel.addWidget (this, "Enable_narrow_edge_collision");
      mPanel.addWidget (this, "Enable_narrow_face_collision");
      
      for (int i=0; i<mNumModels; i++) {
         mPanel.addWidget (this, "Show_AABB_"+i);
         mPanel.addWidget (this, "Show_sweeps_"+i);
      }
      
      mPanel.addWidget (this, "Elipson_edge");
      mPanel.addWidget (this, "Elipson_face");
   }
   
   protected void build_addUI() {
      addControlPanel(mPanel);
      
//      double minDist = mContCol.collideVertexTrianglePnts ( new Point3d(0,0,0), new Point3d(0.51,0,0),
//         new Point3d(0.5, 0.5, 0),     new Point3d(0.5, 0.5, 0),
//         new Point3d(0.5, -0.5, 0.5),  new Point3d(0.5, -0.5, 0.5),
//         new Point3d(0.5, -0.5, -0.5), new Point3d(0.5, -0.5, -0.5), 1e-6);
//      System.out.println ("Min: "+ minDist);
   }
   
   protected void build_post() {

   }
   
   /* --- Properties --- */
   
   public static PropertyList myProps =
      new PropertyList (ContinuousCollisionsMeshTest.class, RootModel.class);
   
   @Override
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }
   
   protected static boolean Enable_broad_edge_collision = false;
   public boolean getEnable_broad_edge_collision() { return Enable_broad_edge_collision; }
   public void setEnable_broad_edge_collision(boolean val) { Enable_broad_edge_collision = val; }
   
   protected static boolean Enable_broad_face_collision = false;
   public boolean getEnable_broad_face_collision() { return Enable_broad_face_collision; }
   public void setEnable_broad_face_collision(boolean val) { Enable_broad_face_collision = val; }
   
   protected static boolean Enable_narrow_edge_collision = false;
   public boolean getEnable_narrow_edge_collision() { return Enable_narrow_edge_collision; }
   public void setEnable_narrow_edge_collision(boolean val) { Enable_narrow_edge_collision = val; }
   
   protected static boolean Enable_narrow_face_collision = false;
   public boolean getEnable_narrow_face_collision() { return Enable_narrow_face_collision; }
   public void setEnable_narrow_face_collision(boolean val) { Enable_narrow_face_collision = val; }
   
   protected static boolean[] Show_AABB = new boolean[mNumModels];
   protected static boolean[] Show_sweeps = new boolean[mNumModels];
   
   public boolean getShow_AABB_0() { return Show_AABB[0]; }
   public void setShow_AABB_0(boolean val) { Show_AABB[0] = val; }
  
   public boolean getShow_AABB_1() { return Show_AABB[1]; }
   public void setShow_AABB_1(boolean val) { Show_AABB[1] = val; }
   
   public boolean getShow_AABB_2() { return Show_AABB[2]; }
   public void setShow_AABB_2(boolean val) { Show_AABB[2] = val; }
   

   public boolean getShow_sweeps_0() { return Show_sweeps[0]; }
   public void setShow_sweeps_0(boolean val) { Show_sweeps[0] = val; }
   
   public boolean getShow_sweeps_1() { return Show_sweeps[1]; }
   public void setShow_sweeps_1(boolean val) { Show_sweeps[1] = val; }
   
   public boolean getShow_sweeps_2() { return Show_sweeps[2]; }
   public void setShow_sweeps_2(boolean val) { Show_sweeps[2] = val; }
   
   protected static double Elipson_edge = 1e-6;
   protected static double Elipson_face = 1e-6;
   
   public double getElipson_edge() { return Elipson_edge; }
   public void setElipson_edge(double val) { Elipson_edge = val; }
   
   public double getElipson_face() { return Elipson_face; }
   public void setElipson_face(double val) { Elipson_face = val; }
   
   
   static {
      myProps.add("Enable_broad_edge_collision", "", Enable_broad_edge_collision);
      myProps.add("Enable_broad_face_collision", "", Enable_broad_face_collision);
      myProps.add("Enable_narrow_edge_collision", "", Enable_narrow_edge_collision);
      myProps.add("Enable_narrow_face_collision", "", Enable_narrow_face_collision);
      
      for (int i=0; i<mNumModels; i++) {
         myProps.add("Show_AABB_"+i, "", Show_AABB[i]);
         myProps.add("Show_sweeps_"+i, "", Show_sweeps[i]);
      }
      
      myProps.add("Elipson_edge", "", Elipson_edge);
      myProps.add("Elipson_face", "", Elipson_face);
      
      Show_AABB[0] = false;
//      Show_AABB[1] = false;
//      Show_AABB[2] = false;
      
      Show_sweeps[0] = true;
//      Show_sweeps[1] = true;
//      Show_sweeps[2] = true;
      
      Enable_broad_edge_collision = true;
      Enable_narrow_edge_collision = true;
      
      Enable_broad_face_collision = true;
      Enable_narrow_face_collision = true;
   }
   



   
   /* --- Loop --- */
   
   public StepAdjustment advance (double t0, double t1, int flags) {  
      return super.advance(t0, t1, flags);
   }
   
   protected boolean isFirstStep = true; 
   
   public void render(Renderer renderer, int flags) {
      if (isFirstStep) {
         getMainViewer ().setBackgroundColor (new Color(105, 102, 94));
         getMainViewer ().setHighlightColor (Color.BLUE);
         isFirstStep = false;
      }

      for (int i=0; i<mNumModels; i++) 
         savePrevPositions(mRB[i], mRB_prev[i], mSMI[i]);
      
      for (int i=0; i<mNumModels; i++) 
         if (Show_sweeps[i]) 
            showSweeps(mRB[i], mRB_prev[i], renderer, Color.ORANGE);
      
      for (int i=0; i<mNumModels; i++) 
         applyTransformsToAABB(mRB[i], mRB_prev[i], mSMI[i]);
      
      for (int i=0; i<mNumModels; i++) 
         mSMI[i].updateBVTrees ();
      
      for (int i=0; i<mNumModels; i++) 
         if (Show_AABB[i]) {
            mSMI[i].myTriangleTree.render (renderer, 0);
         }
            
      if (Enable_broad_edge_collision) {
//         showIntersectingEdges(renderer, 0, 1);
         showIntersectingEdges(renderer, 0, 0);
      }
      if (Enable_broad_face_collision) {
//         showIntersectingVertexFaces(renderer, 2, 1);
//         showIntersectingVertexFaces(renderer, 0, 1);
//         showIntersectingVertexFaces(renderer, 1, 0);
         
         showIntersectingVertexFaces(renderer, 0, 0);
      }
      
      super.render (renderer, flags);
   }
   
   protected void savePrevPositions(RigidBody rb, RigidBody rb_prev, SweptMeshInfo smi) {
      smi.myMesh = rb_prev.getSurfaceMesh ();
      smi.copyCurrent2PreviousPositions ();
      smi.myMesh = rb.getSurfaceMesh ();
   }
   
   protected void applyTransformsToAABB(RigidBody rb, RigidBody rb_prev, SweptMeshInfo smi) {
      
      Vector3d vec = new Vector3d();
      
      // Previous positions must be shifted relative to current RB's local position (i.e. origin).
      // Amount of shift corresponds to distance difference in world space
      
      for (int p=0; p<smi.prevPositions.length; p++) {
         Point3d ptW = new Point3d( rb.getSurfaceMesh ().getVertex (p).pnt );
         rb.getSurfaceMesh ().transformToWorld (ptW);
         
         Point3d ptPrevW = new Point3d( rb_prev.getSurfaceMesh ().getVertex (p).pnt );
         rb_prev.getSurfaceMesh ().transformToWorld (ptPrevW);
         
         vec.sub (ptPrevW, ptW);
         
         smi.prevPositions[p].setZero ();
         smi.prevPositions[p].add (rb.getSurfaceMesh ().getVertex (p).pnt);
         smi.prevPositions[p].add (vec);
      }
      
//      RigidTransform3d tns = new RigidTransform3d();
//      tns.addTranslation ( rb_prev.getPosition () );
//      vec.set(rb.getPosition ());
//      vec.negate ();
//      tns.addTranslation (vec);
//      for (Point3d pt : smi.prevPositions) 
//         pt.transform (tns);
      
      // All positions shifted by current RB's position 
      
      smi.myEdgeTree.setBvhToWorld ( rb.getSurfaceMesh ().getMeshToWorld () );
      smi.myVertexTree.setBvhToWorld ( rb.getSurfaceMesh ().getMeshToWorld () );
      smi.myTriangleTree.setBvhToWorld ( rb.getSurfaceMesh ().getMeshToWorld () );
   }
   
   protected void showSweeps(RigidBody rb_head, RigidBody rb_tail, Renderer renderer, Color lineColor) {
      RenderProps rp = new RenderProps();
      rp.setLineColor (lineColor);
      renderer.setLineColoring (rp, false);
      
      PolygonalMesh headMesh = rb_head.getSurfaceMesh ();
      PolygonalMesh tailMesh = rb_tail.getSurfaceMesh ();
      
      for (int v=0; v < headMesh.numVertices (); v++) {
         Vertex3d vtxHead = headMesh.getVertex (v);
         Vertex3d vtxTail = tailMesh.getVertex (v);

         Point3d pntHead = new Point3d(vtxHead.pnt);
         headMesh.transformToWorld (pntHead);
         
         Point3d pntTail = new Point3d(vtxTail.pnt);
         tailMesh.transformToWorld (pntTail);
         
         renderer.drawArrow (pntTail, pntHead, 0.01, true);
      }
   }
   

   
   protected ArrayList<BVNode> mIntersect0 = new ArrayList<BVNode>();
   protected ArrayList<BVNode> mIntersect1 = new ArrayList<BVNode>();
   String mIntersectStr = "";
   String mPrevIntersectStr = "";
   
   public void showIntersectingEdges(Renderer renderer, int m0, int m1) {
      // Vertices of meshes are always local (i.e. relative to 0,0,0)
      // So, translate all meshes (including previous) to their t1 world
      // Previous vertices have been translated to position of rb_previous
      
      ArrayList<EdgeEdgeContact> eeCts = null;
      if (m0 == m1) {
         eeCts = mContCldr.findEdgeEdgeContacts (mSMI[m0], mSMI[m1]);
      }
      
      // Draw collided edges
      
      renderer.beginDraw (DrawMode.LINES);
      renderer.setLineWidth (10);
      renderer.setColor (Color.YELLOW);
      
      for (EdgeEdgeContact eeCt : eeCts) {
         HalfEdge e0 = eeCt.edge0;
         renderer.addVertex ( e0.head.getWorldPoint () );
         renderer.addVertex ( e0.tail.getWorldPoint () );
         
         HalfEdge e1 = eeCt.edge1;
         renderer.addVertex ( e1.head.getWorldPoint () );
         renderer.addVertex ( e1.tail.getWorldPoint () );
      }
      
      renderer.endDraw();
      
//      mIntersect0.clear ();
//      mIntersect1.clear ();
//      
//      mSMI[m0].myEdgeTree.intersectTree (mIntersect0, mIntersect1, mSMI[m1].myEdgeTree);
//      
//      renderer.beginDraw (DrawMode.LINES);
//      renderer.setLineWidth (10);
//      renderer.setColor (Color.YELLOW);
//      
//      mPrevIntersectStr = new String( mIntersectStr );
//      mIntersectStr = "";
//      
//      mIntersectStr += String.format("Num Edges: %d, Num INTXT Edges: %d\n", mSMI[m0].myEdgeTree.getLeafNodes ().size (), mIntersect0.size ());
//      
//      for (int i=0; i < mIntersect0.size (); i++) {
//         // Get edge pair
//         SweptEdge se0 = (SweptEdge)mIntersect0.get (i).getElements ()[0];
//         SweptEdge se1 = (SweptEdge)mIntersect1.get (i).getElements ()[0];
//         
//         HalfEdge e0 = se0.myEdge;
//         HalfEdge e1 = se1.myEdge;
//         
//         // If edge pair are identical or adjacent, ignore:
//         if (MeshUtil.isShareVertex (e0, e1)) {
//            continue;
//         }
//         
//         mIntersectStr += String.format ("Intersection: %d-%d vs %d-%d\n", 
//            e0.head.getIndex (), e0.tail.getIndex (),
//            e1.head.getIndex (), e1.tail.getIndex ()); 
//         
//         Point3d[] se0p = se0.getPoints ();
//         Point3d[] se1p = se1.getPoints ();
//         
//         // Transform the edge points to world first 
//         Point3d[] se0pw = new Point3d[se0p.length];
//         Point3d[] se1pw = new Point3d[se1p.length];
//         for (int p=0; p<se0p.length; p++) {
//            se0pw[p] = new Point3d( se0p[p] );
//            se1pw[p] = new Point3d( se1p[p] );
//            
//            se0pw[p].transform (mMesh[m0].getMeshToWorld ());
//            se1pw[p].transform (mMesh[m1].getMeshToWorld ());
//         }
//         
//         boolean isDraw = false;
//         if (Enable_narrow_edge_collision) {
//            double minDist = mContCol.collideEdgeEdgePnts (
//               // Edge 0
//               se0pw[2], se0pw[0],   // t=0 to t=1
//               se0pw[3], se0pw[1],
//               // Edge 1
//               se1pw[2], se1pw[0],
//               se1pw[3], se1pw[1], Elipson_edge).minDist;
//            
//            mIntersectStr += String.format("Min Distance: %.4f\n", minDist);
//            isDraw = (minDist >= 0);
//         }
//         else {
//            isDraw = true;
//         }
//         
//         if (isDraw) {
//            // Mesh0
//            renderer.addVertex ( se0pw[0] ); // Current
//            renderer.addVertex ( se0pw[1] );
//            renderer.addVertex ( se0pw[2] ); // Prev
//            renderer.addVertex ( se0pw[3] );
//            
//            // Mesh1
//            renderer.addVertex ( se1pw[0] ); // Current
//            renderer.addVertex ( se1pw[1] );
//            renderer.addVertex ( se1pw[2] ); // Prev
//            renderer.addVertex ( se1pw[3] );
//         }         
//      }
//      
//      if (! mIntersectStr.equals (mPrevIntersectStr)) {
//         System.out.println (mIntersectStr);
//      }
//      
//      renderer.endDraw();
   }
   
   public void showIntersectingVertexFaces(Renderer renderer, int vm, int fm) {
      ArrayList<PenetratingPoint> pentPts = null;
      if (vm == fm) {
         pentPts = mContCldr.findPenetratingPoints (mSMI[vm], mSMI[vm]);
      }
      
      // Draw collided vertices and edges
      for (PenetratingPoint pentPt : pentPts) {
         Vertex3d vtx = pentPt.vertex;
         Face face = pentPt.face;
         
         /* --- Colliding Vertex --- */
         
         renderer.beginDraw (DrawMode.POINTS);
         renderer.setPointSize (20);
         renderer.setColor (Color.GREEN);
         
         renderer.addVertex ( vtx.getWorldPoint () );
         
         renderer.endDraw ();
         
         /* --- Colliding Triangle --- */
         
         renderer.beginDraw (DrawMode.TRIANGLES);
         renderer.setFaceStyle (FaceStyle.FRONT_AND_BACK);
         renderer.setLineWidth (10);
         renderer.setColor (Color.RED);
         
         for (int p=0; p < face.numVertices (); p++) {
            renderer.addVertex ( face.getVertex (p).getWorldPoint () );
         }
         
         renderer.endDraw ();
      }
      
//      mIntersect0.clear ();
//      mIntersect1.clear ();
//      
//      mSMI[vm].myVertexTree.intersectTree (mIntersect0, mIntersect1, mSMI[fm].myTriangleTree);
//      
//      mPrevIntersectStr = new String( mIntersectStr );
//      mIntersectStr = "";
//      mIntersectStr += String.format("Num Faces: %d, Num INTXT Faces: %d\n",
//         mSMI[fm].myTriangleTree.getLeafNodes ().size (), mIntersect0.size ());
//      
//      int numNarrowCollisions = 0;
//      
//      for (int i=0; i < mIntersect0.size (); i++) {
//         // Get vertex-triangle pair
//         SweptVertex sv   = (SweptVertex)mIntersect0.get (i).getElements ()[0];
//         SweptTriangle st = (SweptTriangle)mIntersect1.get (i).getElements ()[0];
//         
//         // Ignore vertex-triangle pair if vertex is connected to triangle 
//         if (MeshUtil.isHasVertex (sv.myVertex, st.myFace)) {
//            continue;
//         }
//         
//         Point3d[] svPts = sv.getPoints ();
//         Point3d[] stPts = st.getPoints ();
//         
//         // Transform the vertex and triangle points to world first 
//         Point3d[] svPtsW = new Point3d[svPts.length];
//         Point3d[] stPtsW = new Point3d[stPts.length];
//         
//         // Vertex
//         for (int p=0; p<svPtsW.length; p++) {
//            svPtsW[p] = new Point3d( svPts[p] );
//            svPtsW[p].transform (mMesh[vm].getMeshToWorld ());
//         }
//         
//         // Triangle
//         for (int p=0; p<stPtsW.length; p++) {
//            stPtsW[p] = new Point3d( stPts[p] );
//            stPtsW[p].transform (mMesh[fm].getMeshToWorld ());
//         }
//         
//         boolean isDraw = false;
//         if (Enable_narrow_face_collision) {
//            double minDist = mContCol.collideVertexTrianglePnts ( 
//               svPtsW[1], svPtsW[0],   // vertex pt from t=0 to t=1 
//               stPtsW[3], stPtsW[0],   // triangle pt from t=0 to t=1
//               stPtsW[4], stPtsW[1],
//               stPtsW[5], stPtsW[2],
//               Elipson_face).hitTime;
//            
//            if (minDist >= 0)
//               numNarrowCollisions++;
//            
//            isDraw = (minDist >= 0);
//         }
//         else {
//            isDraw = true;
//         }
//         
//         if (isDraw) {
//            renderer.beginDraw (DrawMode.TRIANGLES);
//            renderer.setFaceStyle (FaceStyle.FRONT_AND_BACK);
//            renderer.setLineWidth (10);
//            renderer.setColor (Color.RED);
//            
//            // Highlight the face on current
//            renderer.addVertex (stPtsW[0]);
//            renderer.addVertex (stPtsW[1]);
//            renderer.addVertex (stPtsW[2]);
//            
//            // Highlight the face on previous
//            renderer.addVertex (stPtsW[3]);
//            renderer.addVertex (stPtsW[4]);
//            renderer.addVertex (stPtsW[5]);
//            
//            renderer.endDraw();
//            
//            renderer.beginDraw (DrawMode.POINTS);
//            renderer.setPointSize (20);
//            renderer.setColor (Color.GREEN);
//            
//            // Highlight the vertex on current 
//            renderer.addVertex (svPtsW[0]);
//            
//            // Highlight the vertex on previous
//            renderer.addVertex (svPtsW[1]);
//            
//            renderer.endDraw();
//         }         
//      }
//      
//      mIntersectStr += "Num Narrow Collisions: " + numNarrowCollisions;
//      
////      if (! mIntersectStr.equals (mPrevIntersectStr)) {
////         System.out.println (mIntersectStr);
////      }
   }
}
