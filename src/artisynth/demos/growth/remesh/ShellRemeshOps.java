package artisynth.demos.growth.remesh;

import java.util.LinkedList;

import artisynth.core.femmodels.BackNode3d;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.ShellElement3d;
import artisynth.core.femmodels.ShellTriElement;
import artisynth.core.mechmodels.CollidableDynamicComponent;
import artisynth.core.mechmodels.CollisionHandler;
import artisynth.core.mechmodels.ContactConstraint;
import artisynth.core.mechmodels.ContactMaster;
import artisynth.core.mechmodels.ContactPoint;
import artisynth.demos.growth.collision.ContactConstraintAgg;
import maspack.geometry.Face;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.util.DataBuffer;

/**
 * Remeshing operations (e.g. edge bisect, flip, and collapse) that operate
 * on FEM-Shell models.
 * 
 * The FEM-Shell model is treated as a surface mesh where the vertices, edges,
 * and faces of the mesh corresponds with the front nodes, front edges, and 
 * front faces of the FEM-Shell model.
 * 
 * Constraints are also interpolated.
 */
public class ShellRemeshOps extends RemeshOps {
   
   protected FemModel3d mFemModel;
   
   /** New shell elements are set to this thickness. */
   protected double mShellThickness;
   
   // CSNCMT
   protected ContactConstraintAgg mCCAgg;
   
   public ShellRemeshOps(PolygonalMesh mesh, FemModel3d femModel) {
      super(mesh);
      mFemModel = femModel;
      mShellThickness = mFemModel.getShellElement (0).getDefaultThickness ();
   }
   
   protected void setTarget(PolygonalMesh mesh, FemModel3d femModel) {
      super.setTarget (mesh);
      mFemModel = femModel;
   }
   
   // CSNCMT
   public void setConstraints(ContactConstraintAgg ccAgg) {
      this.mCCAgg = ccAgg;
   }
   
   
   /* --- Secondary Functions --- */ 
   
   /** Given two nodes, create an average node. */
   protected void setVertexViaLinearInterpolation(
   Vertex3d v0, Vertex3d v1, Vertex3d vTgt) {
      super.setVertexViaLinearInterpolation (v0, v1, vTgt);
      
      DataBuffer buf_n0 = saveNodeState (v0.getIndex ());
      DataBuffer buf_n1 = saveNodeState (v1.getIndex ());
      
      DataBuffer buf_newNode = linearInterpolateDataBuffer (buf_n0, buf_n1, 0.5);
      loadNodeState (vTgt.getIndex (), buf_newNode);
      
      // CSNCMT
      if (mCCAgg != null && mCCAgg.isHasConstraints ()) {
         createInterpolatedConstraint(v0, v1, vTgt);
      }
   }
   
   /** Given two node, create an average node, and add the average node
    *  to the model. */
   protected Vertex3d addVertexViaLinearInterpolation(Vertex3d v0, Vertex3d v1) {
      FemNode3d newNode = createNode();
      mFemModel.addNode (newNode);
      
      // Calls setVertexViaLinearInterpolation()
      Vertex3d newVtx = super.addVertexViaLinearInterpolation (v0, v1);  
      newNode.setName ("MyNode_#" + newVtx.getIndex ());
      
      return newVtx;
   }
   
   /** Remove a node from the mesh. */
   protected void removeVertex(Vertex3d vtx) {
      // When a node is removed, the reordering of subsequent nodes
      // shouldn't affect the existing constraints, which are linked
      // to specific nodes. When constraint block matrix is being generated,
      // node.getSolveIndex() is used. The node's solve index should retrieve
      // the latest index.
      
      // CSNCMT
      if (mCCAgg != null && mCCAgg.isHasConstraints ()) {
         disableContactMasters(vtx);
      }
      
      super.removeVertex (vtx);
      
      FemNode3d node = mFemModel.getNode (vtx.getIndex ());
      mFemModel.removeNode (node);
      
      // Synchronize node names with their reordered indices.
      for (int i = 0; i < mFemModel.numNodes (); i++) {
         mFemModel.getNode (i).setName ("MyNode_#" + i);
      }
   }
   
   /** Add a new FEM element. */
   protected Object addFace(Face face, Object removedParent) {
      super.addFace (face, removedParent);
      
      ShellTriElement newEle = createElement(face);
      newEle.setName ("MyEle_#" + mMesh.numFaces());
      mFemModel.addShellElement (newEle);
      newEle.getIntegrationData()[0].computeInverseRestJacobian (
         newEle.getIntegrationPoints()[0], newEle.getNodes ());
      
      return newEle;
   }
   
   /** Remove a FEM element. */
   public Object removeFace(Face face) {
      super.removeFace (face);
      
      ShellElement3d removedEle = mFemModel.getShellElements ().
         remove (face.idx);
      
      // Rename subsequent ele
      for (int i = 0; i < mFemModel.numShellElements (); i++) {
         mFemModel.getShellElement (i).setName ("MyEle_#" + i);
      }
      
      return removedEle;
   }
   

   
   /* --- Node and Shell specific Tertiary Functions --- */
   
   /** Create a standalone element. */
   protected ShellTriElement createElement(Face face) {
      Vertex3d[] triVtx = face.getTriVertices();
      FemNode3d[] triNodes = createNodeArray(3);
      for (int i = 0; i < 3; i++) {
         int v = triVtx[i].getIndex ();
         triNodes[i] = mFemModel.getNode (v);
      }

      ShellTriElement newEle = createElement(triNodes[0], triNodes[1],
         triNodes[2], mShellThickness);

      return newEle;
   }
   
   /** Create an empty array of nodes. */
   protected FemNode3d[] createNodeArray(int size) {
      return new FemNode3d[size];
   }
   
   /** Create a standalone element that contains 3 given nodes. */
   protected ShellTriElement createElement(FemNode3d n0, FemNode3d n1,
   FemNode3d n2, double thickness) {
      return new ShellTriElement(n0, n1, n2, thickness);
   };
   
   /** Create a standalone and uninitialized node. */
   protected FemNode3d createNode() {
      FemNode3d node = new FemNode3d();
      return node;
   }
   
   /** Create a copy of a given node's attributes. */
   protected DataBuffer saveNodeState(int n) {
      FemNode3d node = mFemModel.getNode (n);
      
      DataBuffer buf = new DataBuffer();
      
      buf.dput (node.getRestPosition ());
      buf.dput (node.getPosition ());
      buf.dput (node.getVelocity ());
   
      buf.dput (node.getBackRestPosition ());
      buf.dput (node.getBackPosition ());
      buf.dput (node.getBackVelocity ());

      return buf;
   }
   
   /** Set a given node's attributes using a populated DataBuffer. See 
    *  saveNodeState() on how the DataBuffer is laid out. */
   protected void loadNodeState(int n, DataBuffer buf) {
      FemNode3d node = mFemModel.getNode (n);
      
      Point3d tmpPnt = new Point3d();
      Vector3d tmpVec = new Vector3d(); 
      
      buf.dget (tmpPnt);
      node.setRestPosition (tmpPnt);
      
      buf.dget (tmpPnt);
      node.setPosition (tmpPnt);
      
      buf.dget (tmpVec);
      node.setVelocity (tmpVec);

      buf.dget (tmpPnt);
      node.setBackRestPosition (tmpPnt);
      
      buf.dget (tmpPnt);
      node.setBackPosition (tmpPnt);

      buf.dget (tmpVec);
      node.setBackVelocity (tmpVec);
   }
   
   /** Given the attributes of two nodes, linearly interpolate the attributes.
    *  
    *  @param s 
    *  Fraction of A that should be used. Use 0.5 for an average of A and B.
    */
   protected DataBuffer linearInterpolateDataBuffer(DataBuffer A, DataBuffer B, double s) {
      DataBuffer C = new DataBuffer();
      
      for (int i=0; i < A.dsize (); i++) {
         double val = (s)*A.dget () + (1-s)*B.dget ();
         C.dput ( val );
      }
      
      return C;
   }
   
   /** 
    * Create a constraint master for a newly created node based on the
    * constraint masters of its two parent nodes.
    * 
    * The new interpolated constraint master is then wrapped into its own
    * contact constraint, which is then appended with the existing contact
    * constraints.
    * 
    * @param vtxA 
    * Parent Node #0
    * 
    * @param vtxB 
    * Parent Node #1
    * 
    * @param vtxC 
    * New Node
    */
   protected void createInterpolatedConstraint(Vertex3d vtxA, Vertex3d vtxB,
   Vertex3d vtxC) {

      FemNode3d nodeA = mFemModel.getNode (vtxA.getIndex ());
      FemNode3d nodeB = mFemModel.getNode (vtxB.getIndex ());
      FemNode3d nodeC = mFemModel.getNode (vtxC.getIndex ());
      
      BackNode3d backC = nodeC.getBackNode ();
      
      // For each collision pair
      // CSNCMT
      for (CollisionHandler colHdlr : mCCAgg.myColHdlrs) {
         for (boolean isBackNode : new boolean[] {false, true}) {

            LinkedList<ContactConstraint> n0_ccs = 
               getFilteredContactConstraints(colHdlr, nodeA, isBackNode);
            LinkedList<ContactConstraint> n1_ccs = 
               getFilteredContactConstraints(colHdlr, nodeB, isBackNode);
            
            if (n0_ccs.size () == 0 && n1_ccs.size () == 0) 
               return;
            
            // Take the average of 2 constraints and their masters

            // Constraint specific
            Vector3d nrm_avg = new Vector3d();// Nrm calc as if for single mast.
            double dist_avg = 0; // Distance calculated as if for single master.
            int numCCs = n0_ccs.size() + n1_ccs.size ();

            // Contact point specific (not really needed for impulses)
            Point3d cpnt0_avg = new Point3d();
            Point3d cpnt1_avg = new Point3d();
            
            for (int i=0; i<2; i++) {
               LinkedList<ContactConstraint> ccs = (i==0) ? n0_ccs : n1_ccs;
               FemNode3d node = (i==0) ? nodeA : nodeB;
                  
               for (ContactConstraint cc : ccs) {
                  Vector3d nrm = cc.getNormal ();
                  double dist = cc.getDistance ();
                  
                  cpnt0_avg.add( cc.myCpnt0.getPoint () );
                  cpnt1_avg.add( cc.myCpnt1.getPoint () );
                  
                  for (ContactMaster cm : 
                       getFilteredContactMasters (node, cc, isBackNode)) 
                  {
                     double wt = cm.getWeight ();
                     
                     nrm_avg.scaledAdd (wt, nrm); 
                     dist_avg += wt*dist;
                  }
               }
            }
            
            nrm_avg.scale ( 1.0/2 );  // Interpolate 
            nrm_avg.normalize ();
            
            cpnt0_avg.scale ( 1.0/numCCs );
            cpnt1_avg.scale ( 1.0/numCCs );
          
            dist_avg *= (1.0 / 2);   // Interpolate

            // Store average in new master (node impulse relies on 'weight').
            
            ContactMaster cm_avg = (isBackNode) ? 
               new ContactMaster(backC, 1.0) :
               new ContactMaster(nodeC, 1.0);
            
            // Create contact points (this isn't really needed for 
            // node impulses).
               
            ContactPoint cpntC0 = new ContactPoint(cpnt0_avg);
            ContactPoint cpntC1 = new ContactPoint(cpnt1_avg);  

            // Store new master into new unilateral.
            
            ContactConstraint cc = new ContactConstraint();
            cc.m = (n0_ccs.size() > 0) ? n0_ccs.get (0).m : n1_ccs.get(0).m;
            cc.setNormal (nrm_avg);
            cc.setDistance (dist_avg);
            cc.setContactPoints (cpntC0, cpntC1);
            cc.getMasters ().add (cm_avg);
            
            // Add our interpolated unilateral.
            colHdlr.myUnilaterals.add (cc);
         }
         
      }  // For each colHdlr
   }
   

   /** 
    * Remove the constraint masters that are associated with the vertex.
    * The vertex is typically the vertex that's to be removed from the mesh.
    */
   protected void disableContactMasters(Vertex3d vtx) {
      FemNode3d node = mFemModel.getNode (vtx.getIndex ());
      
      // CSNCMT
      for (CollisionHandler colHdlr : mCCAgg.myColHdlrs) {
         for (ContactConstraint cc : colHdlr.myUnilaterals) {
            for (boolean isBackNode : new boolean[] {false, true}) {
               LinkedList<ContactMaster> cms = 
                  getFilteredContactMasters (node, cc, isBackNode);
               cc.getMasters ().removeAll (cms);
            }
            // TODO. Need to remove vtx from the vertices of cm.cpnt?
         }
      }
      
      // Bug fix for 
      // java.lang.IllegalArgumentException: Requested block location
      // 270,28 is out of bounds; matrix block size is 588X26
      for (CollisionHandler colHdlr : mCCAgg.myColHdlrs) {
         colHdlr.myUnilaterals.removeIf (cc -> cc.getMasters ().isEmpty ());
      }
   }
   
   
   /* --- Constraint Utility --- */
   
   /**
    * Get contact constraints that are involved with the given node.
    * 
    * A contact constraint is a collection of contact masters, which are 
    * associated with each node.
    */
   protected LinkedList<ContactConstraint> getFilteredContactConstraints(
   CollisionHandler colHdlr, FemNode3d node, boolean isBackNode) {
      LinkedList<ContactConstraint> n_ccs = new LinkedList<ContactConstraint> ();
      
      // CSNCMT
      for (ContactConstraint cc : colHdlr.myUnilaterals) {
         if (getFilteredContactMasters (node, cc, isBackNode).size () > 0) {
            n_ccs.add (cc);
         }
      }
      
      return n_ccs;
   }
   
   /**
    * Get contact masters (if any) within the given contact constraint cc
    * that are associated with the given node.
    * 
    * Recall that contact masters are assigned on a node basis, each with their
    * own weight.
    */
   protected LinkedList<ContactMaster> getFilteredContactMasters(FemNode3d node,
   ContactConstraint cc, boolean isBackNode) {
      LinkedList<ContactMaster> n_cms = new LinkedList<ContactMaster> ();
      
      BackNode3d backNode = node.getBackNode ();
      
      // CSNCMT
      // For each contact master in the constraint.
      for (ContactMaster cm : cc.getMasters ()) {
         CollidableDynamicComponent cmComp = cm.getComp ();
         
         if (!isBackNode && cmComp == node) {
            n_cms.add (cm);
         }
         else if (isBackNode && cmComp == backNode) {
            n_cms.add (cm);
         }
      }
      
      return n_cms;
   }
   
   /* --- Interfacing --- */
   
   public FemModel3d getFemModel() {
      return this.mFemModel;
   }

}
