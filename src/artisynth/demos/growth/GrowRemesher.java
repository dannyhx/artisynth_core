package artisynth.demos.growth;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.ShellElement3d;
import artisynth.core.femmodels.ShellTriElement;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.demos.growth.diffusion.MeshChemicals;
import maspack.geometry.Face;
import maspack.geometry.HalfEdge;
import maspack.geometry.KDTree3d;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Matrix3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.util.DataBuffer;
import artisynth.demos.growth.remesh.Index3d;
import artisynth.demos.growth.remesh.ShellRemesher;
import artisynth.demos.growth.util.MeshUtil;
import artisynth.demos.growth.util.ShellUtil;

/** 
 * FEM-Shell remesher that accounts for morphogen and plastic strain 
 * interpolation.
 * 
 * Also includes functions to perform arbitrary incisions.
 */
public class GrowRemesher extends ShellRemesher {

   /** Chemicals that are associated with the nodes. */
   protected MeshChemicals mMeshChems;
   
   /** How much should morphogen concentration influence the mesh resolution? */
   protected double mRefineMorphogen = 0;
   
   /**
    * Plastic deformation gradients of the model before remeshing.
    * 
    * mFps_prev[f*numEles+k] = plastic deformation gradient stored in 
    * element e at integration data k. Indexed by face index `f`.
    */
   protected Matrix3d[] mFps_prev;
   
   /** Global coordinates (world-space) of the integration points before
    *  remeshing. */
   protected Index3d[] mIntegPos_prev;
   
   /** How should the plastic deformation gradient be interpolated? */
   protected String mInterpolationMode = "nearest"; // "none", "nearest"
   
   /** Distance offset when searching for nearby integration points of the 
    *  old mesh. */
   protected double mNeighTol = 1e-6; 
   
   /** Number of nearby integration points of the old mesh to search for, when
    *  given a position. */
   protected int mNumNeighs = 6;
   
   /** When a incision occurs at a node (which is replaced with two nodes), 
    *  much should the two nodes be pushed apart? */
   public double mSepScaleForCutNodes = 0.10;
   
  
   
   public GrowRemesher (PolygonalMesh mesh, FemModel3d femModel, double sizeMin,
   double sizeMax, double aspectMin, double refineAngle, double refineComp,
   double refineVelocity, double refineMorphogen, MeshChemicals meshChems) {
      super (mesh, femModel, sizeMin, sizeMax, aspectMin, refineAngle, refineComp,
             refineVelocity);
      mMeshChems = meshChems;
      mRefineMorphogen = refineMorphogen;
   }
   
   /** Change the model that is be remeshed. Useful to use the same remesher
    *  object for multiple FE models. */
   public void setTarget(PolygonalMesh mesh, FemModel3d femModel,
   MeshChemicals meshChems) 
   {
      super.setTarget (mesh, femModel);
      mMeshChems = meshChems;
   }
   
   
   /* --- Primary Public Functions--- */
      
   public void remesh() {
      preRemesh();
      super.remesh ();
      postRemesh();
   }
   
   /**
    * Copy plastic deformation gradient and world-space of each integration data
    * before remeshing.
    */
   public void preRemesh() {
      if (mFemModel.myThinShellAux != null) {
         return;
      }
   
      int numEles = mMesh.numFaces ();
      int numIntegPts = mFemModel.getShellElement(0).numIntegrationPoints ();
      
      mFps_prev = new Matrix3d[numEles*numIntegPts];
      mIntegPos_prev = new Index3d[numEles*numIntegPts];
      
      for (int f = 0; f < mMesh.numFaces (); f++) {
         GrowTriElement gEle = (GrowTriElement)mFemModel.getShellElement (f);
         int numIpnts = gEle.numIntegrationPoints ();
         
         for (int k = 0; k < numIpnts; k++) {
            GrowIntegrationData3d idat = gEle.getIntegrationData ()[k];
            mFps_prev[f*numIpnts+k] = idat.getFp ().clone ();
            
            if (mFps_prev[f*numIpnts+k].maxNorm () > 2) 
               System.out.printf (
                  "Warning: Detected pre-remesh inelastic strain that exceed.\n");
            
            Point3d ipnt_pos = ShellUtil.computePositionAtIntegrationPoint (k, gEle);
            mIntegPos_prev[f*numIpnts+k] = new Index3d(ipnt_pos, f*numIpnts+k);
         }
      }
   }
   
   /** 
    * Transfer the plastic deformation gradients from the old mesh to the new
    * mesh. 
    */
   public void postRemesh() {
      if (mFemModel.myThinShellAux != null) {
         return;
      }
      
      if (isEleModified) {
//         System.out.println ("Transferring residual strain to new mesh...");
         transferPlasticDeformationGradients (mFemModel.getShellElements ());
      }
   }
   
   /**
    * Transfer the plastic deformation gradients from the old mesh to the new
    * mesh. 
    * 
    * @param newEles
    * List of elements from the new mesh that need their plastic deformation 
    * gradient computed.
    */
   protected void transferPlasticDeformationGradients(
   RenderableComponentList<ShellElement3d> newEles) {
      
      KDTree3d nearestNeighInterpolator =
         new KDTree3d(Arrays.asList (mIntegPos_prev));

      for (int e = 0; e < newEles.size (); e++) {
         GrowTriElement newEle = (GrowTriElement)newEles.get (e);
         int numK = newEle.numIntegrationPoints ();

         // For each integ pt to be filled with a Fp.
         GrowIntegrationData3d[] newEleIdats = newEle.getIntegrationData ();
         for (int k = 0; k < numK; k++) {
            Point3d new_k_pos = 
               ShellUtil.computePositionAtIntegrationPoint (k, newEle);
            
            Matrix3d new_k_Fp = null;
            
            if (mInterpolationMode.equals("nearest")) {
               new_k_Fp = interpolatePlasticDeformationGradient_nearestNeigh (
                  new_k_pos, nearestNeighInterpolator);
            }
            else {
               new_k_Fp = new Matrix3d(); 
               new_k_Fp.setIdentity ();
            }
            
            if (new_k_Fp.maxNorm () > 2) 
               System.out.printf (
                  "Warning: Detected an exceeded interpolated plastic strain.\n");
            
            newEleIdats[k].setFp (new_k_Fp);
         }
      }
   }
   
   /** 
    * Given a point, calculate its interpolated plastic deformation gradient 
    * from the integration points of the old mesh.
    * 
    * @param samplePnt
    * Point where the plastic deformation gradient is to be sampled from.
    * 
    * @param nearestNeighInterpolator
    * 3D KD-Tree interpolator that is initialized with the global coordinates 
    * (world-space) of the integration points of the old mesh.
    */
   protected Matrix3d interpolatePlasticDeformationGradient_nearestNeigh(
   Point3d samplePnt, KDTree3d nearestNeighInterpolator) 
   {
      ArrayList<Point3d> neighPnts = 
         nearestNeighInterpolator.nearestNeighbourSearch (
            samplePnt, mNumNeighs, mNeighTol);

      Collections.sort (neighPnts, (Point3d A, Point3d B) -> {
         double distA = A.distance(samplePnt);
         double distB = B.distance(samplePnt);
         return Double.compare(distA, distB);
      });
      
      // Only get 6 closest neighbors
      List<Point3d> closestPnts = neighPnts.subList (0, mNumNeighs);
      
      double[] wts = new double[mNumNeighs];
      for (int p = 0; p < mNumNeighs; p++) {
         double dist = closestPnts.get (p).distance (samplePnt);
         wts[p] = 1/dist;
      }
   
      double wtSum = 0;
      for (double wt : wts)
         wtSum += wt;
      
      int numIntegPts = mFemModel.getShellElement(0).numIntegrationPoints ();
      
      Matrix3d itpPrestrain = new Matrix3d();
      for (int p = 0; p < mNumNeighs; p++) {
         Index3d neighPnt = (Index3d)closestPnts.get (p);
         int f = neighPnt.idx / numIntegPts;
         int k = neighPnt.idx % numIntegPts;
         
         if (Double.isInfinite (wts[p])) {
            return this.mFps_prev[ f*numIntegPts+k ];
         }
         else {
            itpPrestrain.scaledAdd (
               wts[p], this.mFps_prev[ f*numIntegPts+k ]);
         }
      }
      itpPrestrain.scale (1/wtSum);

      return itpPrestrain;
   }
  
   /* --- Primary Remeshing Operations --- */
   
//   public OpRv bisectEdge(HalfEdge he) {
//      OpRv rv = super.bisectEdge (he);
//      
//      return rv;
//   }
//   
//   public OpRv flip(HalfEdge edge) {
//      return super.flip (edge);
//   }
//   
//   public OpRv collapseEdge(HalfEdge e, Vertex3d rVtx, boolean useVtxAvg) {
//      return super.collapseEdge (e, rVtx, useVtxAvg);
//   }

   /* --- Secondary Functions --- */ 
   
   /** 
    * Set the attributes of a given node using the average attributes of two 
    * other nodes.
    * 
    * @param v0 v1
    * Nodes whose attributes are to be averaged together.
    * 
    * @param vTgt
    * Given node whose attributes are to be set.
    */
   protected void setVertexViaLinearInterpolation(
   Vertex3d v0, Vertex3d v1, Vertex3d vTgt)
   {
      super.setVertexViaLinearInterpolation (v0, v1, vTgt);
      
      GrowNode3d n0 = (GrowNode3d)mFemModel.getNode (v0.getIndex ());
      GrowNode3d n1 = (GrowNode3d)mFemModel.getNode (v1.getIndex ());
      
      if (n0.mIsMorphogenSrc && n1.mIsMorphogenSrc) {
         ((GrowNode3d)mFemModel.
            getNode (vTgt.getIndex ())).mIsMorphogenSrc = true;
      }
      
      if (n0.mIsNoMorphogenZone && n1.mIsNoMorphogenZone) {
         ((GrowNode3d)mFemModel.
            getNode (vTgt.getIndex ())).mIsNoMorphogenZone = true;
      }
      
      if (! n0.isDynamic () && ! n1.isDynamic ()) {
         ((GrowNode3d)mFemModel.
            getNode (vTgt.getIndex ())).setDynamic (false);
      }
   }
   
   /** Remove a node from the model. */
   public void removeVertex(Vertex3d vtx) {
      mMeshChems.removeChemVtx (vtx.getIndex ());
      super.removeVertex (vtx);
   }
   
   
   /* --- Node and Shell specific Tertiary Functions --- */
   
   protected GrowNode3d[] createNodeArray(int size) {
      return new GrowNode3d[size];
   }
   
   protected ShellTriElement createElement(FemNode3d n0, FemNode3d n1,
   FemNode3d n2, double thickness) {
      GrowTriElement ele = new GrowTriElement(
         (GrowNode3d)n0, 
         (GrowNode3d)n1,
         (GrowNode3d)n2, thickness, !this.mHasBackNode);
      
      isEleModified = true;
      ele.setMaterial (mFemModel.getMaterial ());
      
      return ele;
   }
   
   protected GrowNode3d createNode() {
      GrowNode3d node = new GrowNode3d(!mHasBackNode);
      mSizingField.addVertexSF (new Matrix3d());
      
      node.mChems = new VectorNd( mMeshChems.getNumChemTypes () );
      mMeshChems.addChemVtx ( node.mChems );
      
      return node;
   }
   
   protected DataBuffer saveNodeState(int n) {      
      GrowNode3d node = (GrowNode3d)mFemModel.getNode (n);
      
      DataBuffer buf = super.saveNodeState (n);
      buf.dput ( mMeshChems.getVtxChems (n) );
      buf.dput ( node.m_ES_front_vel );
      buf.dput ( node.m_ES_back_vel );
      
      return buf;
   }
   
   protected void loadNodeState(int n , DataBuffer buf) {
      GrowNode3d node = (GrowNode3d)mFemModel.getNode (n);
      
      super.loadNodeState (n, buf);
      buf.dget ( mMeshChems.getVtxChems (n) );
      buf.dget ( node.m_ES_front_vel );
      buf.dget ( node.m_ES_back_vel );
   }
   
   protected double edgeMetric(HalfEdge edge) {
      FemNode3d node0 = mFemModel.getNode (edge.head.getIndex());
      FemNode3d node1 = mFemModel.getNode (edge.tail.getIndex());
      
      double metric = super.edgeMetric (edge);
      
      double mrph0 = ((GrowNode3d)node0).getChem3 ();
      double mrph1 = ((GrowNode3d)node1).getChem3 ();
      
      double avgMrph = (mrph0 + mrph1) / 2.0;
      metric += mRefineMorphogen * avgMrph;
      
      return metric;
   }
   
   /* --- Experimental Cutting Function --- */
   
   /**
    * Perform an incision on a mesh.
    * 
    * @param nodeSeq
    * Sequence of nodes to cut through. The nodes must be ordered and connected.
    * If the first and last nodes in the sequence are connected, then the edge
    * between them will also be incised.
    */
   public void cutNodes(LinkedList<FemNode3d> nodeSeq) {
      preRemesh ();
      
      mSizingField.computeVertexSizingFields ();
      
      int seqLen = nodeSeq.size ();
      if (seqLen < 1) {
         throw new RuntimeException(
            "Number of nodes to incise must be 2 or more.");
      }
      
      TwinVertexCollection twinVtxs = new TwinVertexCollection();
      
      // Use sliding window
      for (int n=0; n<seqLen-1;) {
         Vertex3d vtx0 = mMesh.getVertex( Integer.parseInt ( 
            nodeSeq.get(n).getName ().substring (8) ) );
         Vertex3d vtx1 = mMesh.getVertex( Integer.parseInt ( 
            nodeSeq.get(n+1).getName ().substring (8) ) );
         
         boolean isBnd0 = MeshUtil.isBoundaryVtx (vtx0);
         boolean isBnd1 = MeshUtil.isBoundaryVtx (vtx1);
         
         if (isBnd0 && isBnd1) {
            cutNodes(new Vertex3d[] {vtx0, vtx1, null}, "b-b", twinVtxs);
            n++;
         }
         else if (isBnd0 && !isBnd1) {
            cutNodes(new Vertex3d[] {vtx0, vtx1, null}, "b-n", twinVtxs);
            n++;
         }
         else if (!isBnd0 && isBnd1) {
            // Flip vertices => same as second case.
            cutNodes(new Vertex3d[] {vtx1, vtx0, null}, "b-n", twinVtxs);
            n++;
         }
         else if (!isBnd0 && !isBnd1 && seqLen == 2) {
            throw new RuntimeException("Unnecessary incision case. Use 3 "
            + "or more nodes when incising only non-boundary nodes.");
         }
         // 3 node non-boundary incision
         else { 
            Vertex3d vtx2 = mMesh.getVertex( Integer.parseInt ( 
               nodeSeq.get(n+2).getName ().substring (8) ) );
            cutNodes(new Vertex3d[] {vtx0, vtx1, vtx2}, "n-n-n", twinVtxs);
            n += 2;
         }
      }
      
      // Are first and last nodes in sequence connected?
      {
         Vertex3d firstVtx = mMesh.getVertex ( Integer.parseInt ( 
            nodeSeq.get(0).getName ().substring (8) ) );
         Vertex3d lastVtx = mMesh.getVertex ( Integer.parseInt ( 
            nodeSeq.get(seqLen-1).getName ().substring (8) ) );
         
         if (MeshUtil.isConnected (firstVtx, lastVtx)) {
            cutNodes(new Vertex3d[] {firstVtx, lastVtx, null}, "b-b", twinVtxs);
         }
      }
      
      twinVtxs.calculateSeparation ();
      twinVtxs.applySeperation ();
      
      postRemesh ();
   }

   
   /**
    * Perform an incision base case.
    * 
    * @param seqVtxs
    * Ordered and connected sequence of vertices to cut. Must be 2 or 3
    * vertices.
    * 
    * @param cutCase
    * "b-b" = seqVtxs specifies 2 boundary vertices.
    * "b-n" = seqVtxs specifies 1 boundary vertex and 1 non-boundary vertex.
    * "n-n-n" = seqVtxs specifies 3 non-boundary vertices.
    * 
    * @param twinVtxs
    * Keeps track of seqVtxs and their duplicates. Used for separating 
    * these vertices/nodes apart later.
    */
   public void cutNodes(Vertex3d[] seqVtxs, String cutCase, 
   TwinVertexCollection twinVtxs) {
      HalfEdge edge0 = MeshUtil.getEdge (seqVtxs[0], seqVtxs[1]);
      System.out.println ("Edge0: " + edge0.vertexStr ());
      System.out.println ("Case: " + cutCase);

      LinkedList<Face> travFaces_s1 = new LinkedList<Face>();
      
      // For each side of edge0
      for (int s=0; s<2; s++) {
         System.out.println ("s: " + s);

         // For each direction
         int numDir = (cutCase.equals ("b-b")) ? 2 : 1;
         for (int d=0; d<numDir; d++) {
            
            Vertex3d revolVtx = 
            (cutCase.equals ("b-n")) ? seqVtxs[0] :   
            (cutCase.equals ("b-b")) ? seqVtxs[d] :  
                                       seqVtxs[1];   // n-n-n case
            
            System.out.println ("Revolve vtx: " + revolVtx.getIndex ());
         
            Face curFace = null;
            Face prvFace = null;
            
            // Traverse through faces
            for (int i=0; true; i++) {
               if (curFace == null) {
                  // Hop to face and keep track of previous face 
                  // (don't want to backtrack).
                  curFace = (s==0) ? edge0.getFace () : edge0.getOppositeFace ();
                  prvFace = (s==0) ? edge0.getOppositeFace () : edge0.getFace ();
               }
               else {
                  Face nxtFace = MeshUtil.getAdjacentFace (
                     curFace, prvFace/*excludeFace*/, revolVtx/*reqVtx*/);
                  prvFace = curFace;
                  curFace = nxtFace;
               }

               System.out.println ("Cur face: " + curFace.idx);
               
               // Exclude first face in d=1 traversal
               if (s==1 && !(d==1 && i==0)) { 
                  travFaces_s1.add (curFace);
                  System.out.println ("Added face!");
               }
               
               // This is the last face for n-n-n case.
               if (cutCase.equals("n-n-n") &&
                  MeshUtil.isHasVertex (seqVtxs[2], curFace)) 
               {
                  break;
               }
               
               Face nxtFace = MeshUtil.getAdjacentFace (
                  curFace, prvFace/*excludeFace*/, revolVtx/*reqVtx*/);
               if (nxtFace == null) {
                  break;
               }
            }
         }
      }
      // Finished traversal.
      
      // Create necessary duplicates of vertices. 
      
      Vertex3d[] dupSeqVtxs = new Vertex3d[3];
      if (cutCase.equals("b-n")) {
         dupSeqVtxs[0] = addVertexViaLinearInterpolation (seqVtxs[0], seqVtxs[0]);
         
      } 
      else if (cutCase.equals("b-b")) {
         dupSeqVtxs[0] = addVertexViaLinearInterpolation (seqVtxs[0], seqVtxs[0]);
         dupSeqVtxs[1] = addVertexViaLinearInterpolation (seqVtxs[1], seqVtxs[1]);
      } 
      else {
         dupSeqVtxs[1] = addVertexViaLinearInterpolation (seqVtxs[1], seqVtxs[1]);
      }
      
      // Add seqVtx and its duplicate in twin collection
      for (int v=0; v<3; v++) {
         twinVtxs.addAdjVtxToIgnore (seqVtxs[v]);
         twinVtxs.addAdjVtxToIgnore (dupSeqVtxs[v]);
         
         if (dupSeqVtxs[v] != null) {
            twinVtxs.addTwin (seqVtxs[v], dupSeqVtxs[v]);
         }
      }
      
      // Replaces the faces 
      
      for (Face refFace : travFaces_s1) {
         Vertex3d[] refFaceVtxs = refFace.getTriVertices ();
         
         Vertex3d[] newFaceVtxs = new Vertex3d[3];
         for (int v=0; v<3; v++) {
            // Is vertex same as vertex that was duplicated?
            for (int c=0; c<3; c++) {
               if (refFaceVtxs[v] == seqVtxs[c] && dupSeqVtxs[c] != null) {
                  newFaceVtxs[v] = dupSeqVtxs[c];
               }
            }
            
            if (newFaceVtxs[v] == null)
               newFaceVtxs[v] = refFaceVtxs[v];
         }
         
         Face newFace = MeshUtil.createFace (newFaceVtxs[0], newFaceVtxs[1], newFaceVtxs[2], refFace);
         removeFace (refFace, null);
         addFace (newFace, null);
      }
   }
   
   /** Collection of twin vertices along an incised path. */
   public class TwinVertexCollection {
      ArrayList<TwinVertex> twinVtxs = new ArrayList<TwinVertex>();
      LinkedList<Vertex3d> adjVtxsToIgnore = new LinkedList<Vertex3d>();
      
      public TwinVertexCollection() {}
      
      public void addTwin(Vertex3d A, Vertex3d B) {
         twinVtxs.add (new TwinVertex(A,B));
      }
      
      public void addAdjVtxToIgnore(Vertex3d vtx) {
         adjVtxsToIgnore.add (vtx);
      }
      
      public void calculateSeparation() {
         for (TwinVertex twin : twinVtxs) {
            twin.calculateSeparation (adjVtxsToIgnore);
         }
      }
      
      public void applySeperation() {
         for (TwinVertex twin : twinVtxs) {
            twin.applySeparation ();
         }
      }
   }
   
   /** Along an incision path (series of connected nodes), a twin vertex is a 
    *  pair of nodes that results from the incision at a single node.
    */
   public class TwinVertex {
      public Vertex3d A;
      public Vertex3d B;
      
      public LinkedList<Point3d> newPosStack = new LinkedList<Point3d>();
      
      public TwinVertex(Vertex3d A, Vertex3d B) {
         this.A = A;
         this.B = B;
      }
      
      public void calculateSeparation(LinkedList<Vertex3d> adjVtxsToIgnore) {
         calculateSeparation("A", adjVtxsToIgnore);
         calculateSeparation("B", adjVtxsToIgnore);
      }
      
      public void applySeparation() {
         applySeparation ("A");
         applySeparation ("B");
      }
      
      protected void calculateSeparation(String whichVtx, 
      LinkedList<Vertex3d> adjVtxsToIgnore) {
         Vertex3d vtx = (whichVtx.equals ("A")) ? A : B;
         Vertex3d oppVtx = (whichVtx.equals ("A")) ? B : A;

         // Get vertices that are adjacent to 'vtx' and not connected to 
         // oppVtx.
         
         LinkedList<Vertex3d> validAdjVtxs = new LinkedList<Vertex3d>();
         for (Vertex3d candVtx : MeshUtil.getConnectedVertices (vtx)) {
            if (! MeshUtil.isConnected (candVtx, oppVtx) && 
                ! adjVtxsToIgnore.contains (candVtx)) 
               validAdjVtxs.add (candVtx);
         }
         
         // Map these vertices to nodes 
         
         System.out.println ("Calcuating Sep for Vtx: " + vtx.getIndex ());
         
         LinkedList<FemNode3d> validAdjNodes = new LinkedList<FemNode3d>();
         for (Vertex3d validAdjVtx : validAdjVtxs) {
            System.out.println ("ValidAdjVtx: " + validAdjVtx.getIndex ());
            validAdjNodes.add ( mFemModel.getNode (validAdjVtx.getIndex()) );
         }
         
         // Calculate the new proposed position for the node
         
         FemNode3d node = mFemModel.getNode (vtx.getIndex ());
         
         for (boolean isFront : new boolean[] {true, false}) {
            for (boolean isWorld : new boolean[] {true, false}) {
    
               Point3d posA = ShellUtil.getPosition (node, isFront, isWorld);
               Point3d posB = ShellUtil.getAvgPosition (validAdjNodes, isFront, isWorld);
      
               Vector3d A2B = new Vector3d(posB).sub (posA);
               A2B.scale (mSepScaleForCutNodes);

               Point3d newPosA = (Point3d)new Point3d(posA).add (A2B);
               
               // Save it; don't apply new position yet.
               newPosStack.push (newPosA);
            }
         }
      }
      
      protected void applySeparation(String whichVtx) {
         Vertex3d vtx = (whichVtx.equals ("A")) ? A : B;
         FemNode3d node = mFemModel.getNode (vtx.getIndex ());
         
         for (boolean isFront : new boolean[] {true, false}) {
            for (boolean isWorld : new boolean[] {true, false}) {
    
               Point3d newPos = newPosStack.removeLast ();
               ShellUtil.setPosition (node, isFront, isWorld, newPos);
            }
         }
      }
      
   }

}
