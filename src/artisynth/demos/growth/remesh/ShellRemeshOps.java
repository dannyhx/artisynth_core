package artisynth.demos.growth.remesh;

import java.util.ArrayList;
import java.util.LinkedList;

import artisynth.core.femmodels.BackNode3d;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.ShellElement3d;
import artisynth.core.femmodels.ShellTriElement;
import artisynth.core.femmodels.FemElement.ElementClass;
import artisynth.core.mechmodels.CollidableDynamicComponent;
import artisynth.core.mechmodels.CollisionHandler;
import artisynth.core.mechmodels.ContactConstraint;
import artisynth.core.mechmodels.ContactMaster;
import artisynth.core.mechmodels.ContactPoint;
import artisynth.core.mechmodels.Particle;
import artisynth.core.mechmodels.PointParticleAttachment;
import artisynth.core.mechmodels.VertexContactMaster;
import artisynth.demos.growth.collision.ContactConstraintAgg;
import artisynth.demos.growth.remesh.RemeshOps.OpRv;
import artisynth.demos.growth.remesh.RemeshOps.RemeshOp;
import artisynth.demos.growth.util.ShellUtil;
import maspack.geometry.Face;
import maspack.geometry.HalfEdge;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Matrix3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.util.DataBuffer;
import maspack.util.Pair;

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
   
   protected boolean mHasBackNode;
   
   public ShellRemeshOps(PolygonalMesh mesh, FemModel3d femModel) {
      super(mesh);
      mFemModel = femModel;
      mShellThickness = mFemModel.getShellElement (0).getDefaultThickness ();
      
      mHasBackNode = 
         femModel.getShellElement (0).getElementClass () == ElementClass.SHELL;
   }
   
   protected void setTarget(PolygonalMesh mesh, FemModel3d femModel) {
      super.setTarget (mesh);
      mFemModel = femModel;
      
      mHasBackNode = 
         femModel.getShellElement (0).getElementClass () == ElementClass.SHELL;
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
   protected Object addFace(Face face, OpRv opRv) {
      super.addFace (face, opRv);
      
      ShellTriElement newEle = createElement(face);
      newEle.setName ("MyEle_#" + mMesh.numFaces());
      mFemModel.addShellElement (newEle);
      
      ShellOpRv sOpRv = (ShellOpRv) opRv;
      
      if (mFemModel.myThinShellAux == null) {
         newEle.getIntegrationData()[0].computeInverseRestJacobian (
            newEle.getIntegrationPoints()[0], newEle.getNodes ());
      } else if (sOpRv.mOp == RemeshOp.BISECT) {
         // Newly created elements uses same strain as parent's.
         Matrix3d plasMembStrain = sOpRv.mParentPlasticMembStrains.get (0);
         Matrix3d plasBendStrain = sOpRv.mParentPlasticBendStrains.get (0);
         newEle.getPlasticDeformation ().set(plasMembStrain);
         newEle.getPlasticBendStrain ().set(plasBendStrain);
      } else if (sOpRv.mOp == RemeshOp.FLIP) {
         // Newly created elements uses area-weighted average of parents'.
         Pair<Matrix3d,Matrix3d> strainPair = sOpRv.getAvgParentPlasticStrains ();
         Matrix3d plasMembStrain = strainPair.first;
         Matrix3d plasBendStrain = strainPair.second;
         newEle.getPlasticDeformation ().set(plasMembStrain);
         newEle.getPlasticBendStrain ().set(plasBendStrain);
      } else if (sOpRv.mOp == RemeshOp.COLLAPSE) {
         // Newly created element uses corresponding parent's.
         Matrix3d plasMembStrain = sOpRv.mParentPlasticMembStrains.removeFirst ();
         Matrix3d plasBendStrain = sOpRv.mParentPlasticBendStrains.removeFirst ();
         newEle.getPlasticDeformation ().set(plasMembStrain);
         newEle.getPlasticBendStrain ().set(plasBendStrain);
      }
      
      return newEle;
   }
   
   /** Remove a FEM element. */
   public Object removeFace(Face face, OpRv opRv) {
      ShellTriElement rmEle = (ShellTriElement)mFemModel.getShellElement(face.idx);

      // Save the thin-shell element-wise plastic strain.
      
      if (opRv != null && mFemModel.myThinShellAux != null) {
         Matrix3d membStrain = new Matrix3d(rmEle.getPlasticDeformation ());
         Matrix3d bendStrain = mFemModel.myThinShellAux.bendStrain_edgesToFace (face);

         ShellOpRv sOpRv = (ShellOpRv) opRv;
         sOpRv.mParentPlasticMembStrains.add (membStrain);
         sOpRv.mParentPlasticBendStrains.add (bendStrain);
         sOpRv.mParentRestAreas.add( ShellUtil.area(rmEle.getNodes (), true) );
      }

      // Remove face and its element from model.
      
      super.removeFace (face, opRv);
      mFemModel.getShellElements ().remove (face.idx);
      
      // Rename subsequent ele
      
      for (int i = 0; i < mFemModel.numShellElements (); i++) {
         mFemModel.getShellElement (i).setName ("MyEle_#" + i);
      }
      
      return rmEle;
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
      return new ShellTriElement(n0, n1, n2, thickness, !mHasBackNode);
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
   
      if (mHasBackNode) {
         buf.dput (node.getBackRestPosition ());
         buf.dput (node.getBackPosition ());
         buf.dput (node.getBackVelocity ());
      }

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

      if (mHasBackNode) {
         buf.dget (tmpPnt);
         node.setBackRestPosition (tmpPnt);
         
         buf.dget (tmpPnt);
         node.setBackPosition (tmpPnt);

         buf.dget (tmpVec);
         node.setBackVelocity (tmpVec);
      }
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
            if (isBackNode && !mHasBackNode) {
               continue;
            }

            LinkedList<ContactConstraint> n0_ccs = 
               getFilteredContactConstraints(colHdlr, nodeA, isBackNode);
            LinkedList<ContactConstraint> n1_ccs = 
               getFilteredContactConstraints(colHdlr, nodeB, isBackNode);
            
            if (n0_ccs.size () == 0 && n1_ccs.size () == 0) 
               return;
            
            // Take the average of 2 constraints and their masters

            // Constraint specific
//            Vector3d nrm_avg = new Vector3d();// Nrm calc as if for single mast.
            
//            double dist_avg = 0; // Distance calculated as if for single master.
            
            int numCCs = n0_ccs.size() + n1_ccs.size ();

            // Contact point specific (not really needed for impulses)
            Point3d cpnt0_avg = new Point3d();
            Point3d cpnt1_avg = new Point3d();
            
            Particle nodeC_fb = (isBackNode) ? backC : nodeC;
            
            ContactConstraint cc_avg = new ContactConstraint();
            cc_avg.myMasters0.add(createSingleNodeVertexContactMaster(nodeC_fb, 1));
            cc_avg.myMasters1.add(createSingleNodeVertexContactMaster(nodeC_fb, 1));
            cc_avg.m = (n0_ccs.size() > 0) ? n0_ccs.get (0).m : n1_ccs.get(0).m;
            
            // Calculate average normal and distance among the two nodes.
            
            // For nodeA and nodeB...
            for (int i=0; i<2; i++) {
               LinkedList<ContactConstraint> ccs = (i==0) ? n0_ccs : n1_ccs;
               FemNode3d node = (i==0) ? nodeA : nodeB;
                  
               // For each constraint (normal, dist, 2 contact points)
               for (ContactConstraint cc : ccs) {
                  Vector3d nrm = cc.getNormal ();
                  double dist = cc.getDistance ();
                  
                  cpnt0_avg.add( cc.myCpnt0.getPoint () );
                  cpnt1_avg.add( cc.myCpnt1.getPoint () );
                  
//                  for (ContactMaster cm : 
//                       getFilteredContactMasters (node, cc, isBackNode)) 
//                  {
                     // DAN21TODO
//                     double wt = cm.getWeight ();
//                     
//                     nrm_avg.scaledAdd (wt, nrm); 
//                     dist_avg += wt*dist;
//                  }
                  
                  for (int m = 0; m < 2; m++) {
                     ArrayList<ContactMaster> masters = (m == 0) ? cc.myMasters0 : cc.myMasters1;
                     for (ContactMaster cm : masters) {
                        VertexContactMaster vcm = (VertexContactMaster)cm; 
                        double wt = vcm.myWgts[0];
                        
//                      nrm_avg.scaledAdd (wt, nrm); 
//                      dist_avg += wt*dist;
                        
                        cc_avg.getNormal ().scaledAdd(wt, nrm);
                        cc_avg.setDistance (cc_avg.getDistance () + wt*dist);
                     }
                  }
                  
               }
            }
            
            cc_avg.getNormal().scale ( 1.0/2 );  // Interpolate 
            cc_avg.getNormal ().normalize ();
            
            cpnt0_avg.scale ( 1.0/numCCs );
            cpnt1_avg.scale ( 1.0/numCCs );
          
            cc_avg.setDistance(cc_avg.getDistance () * (1.0 / 2));   // Interpolate

            // Store average in new master (node impulse relies on 'weight').
            
            // Create contact points (this isn't really needed for 
            // node impulses).
               
            ContactPoint cpntC0 = new ContactPoint(cpnt0_avg);
            ContactPoint cpntC1 = new ContactPoint(cpnt1_avg);  

            // Store new master into new unilateral.
            
            cc_avg.setContactPoints (cpntC0, cpntC1);
            
            // Add our interpolated unilateral.
            colHdlr.myUnilaterals.add (cc_avg);
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
            // Scan through ContactConstraint's ContactMasters, searching for
            // ones involving the given node. If found such ContactMaster,
            // delete it.
            for (int m = 0; m < 2; m++) {
               ArrayList<ContactMaster> masters = (m == 0) ? cc.myMasters0 : cc.myMasters1;
               for (ContactMaster cm : masters) {
                  if (cm instanceof VertexContactMaster) {
                     VertexContactMaster vcm = (VertexContactMaster)cm; 
                    
                     ContactMaster ppaToDelete = null;
                     int ppaIdxToDelete = -1;
                     
                     for (ArrayList<ContactMaster> vcm_cms : vcm.myMasterLists) {
                        
                        // For each ContactMaster list in the VertexContactMaster
                        int ppaIdx = 0;
                        for (ContactMaster vcm_cm : vcm_cms) {
                           if (vcm_cm instanceof PointParticleAttachment) {
                              PointParticleAttachment ppa = (PointParticleAttachment)vcm_cm;
                              Particle particle = ppa.myParticle;
                              
                              if (particle == node) {
                                 ppaToDelete = vcm_cm;
                                 ppaIdxToDelete = ppaIdx;
                                 break;
                              }
                           } else {
                              throw new AssertionError("Unexpected ContactMaster type."); 
                           }
                           
                           ppaIdx++;
                        } 
                        
                        if (ppaToDelete != null) {
                           // Delete PPA.
                           vcm_cms.remove (ppaToDelete);
                           break;
                        }
                     }
                     
                     if (ppaToDelete != null) {
                        // Delete PPA's weight.
                        int new_wgt_idx = 0;
                        double[] new_wgts = new double[vcm.myWgts.length-1];
                        
                        for (int i = 0; i < vcm.myWgts.length; i++) {
                           if (i != ppaIdxToDelete) {
                              new_wgts[new_wgt_idx] = vcm.myWgts[i];
                              new_wgt_idx++;
                           }
                        }
                     }
                  } else {
                     throw new AssertionError("Unexpected ContactMaster type."); 
                  }
               }
               
               // If VertexContactMaster is actually empty, then remove it
               // from ContactConstraint's list of masters (0 or 1).
               
               ArrayList<ContactMaster> masters_copy = (ArrayList<ContactMaster>)masters.clone();
               for (ContactMaster curMaster : masters_copy) {
                  VertexContactMaster curVcm = (VertexContactMaster)curMaster; 
                  
                  if (curVcm.myWgts.length == 0) {
                     masters.remove (curVcm);
                  }
               }
            }
               
            // TODO. Need to remove vtx from the vertices of cm.cpnt?
         }
      }
      
      // Bug fix for 
      // java.lang.IllegalArgumentException: Requested block location
      // 270,28 is out of bounds; matrix block size is 588X26
      for (CollisionHandler colHdlr : mCCAgg.myColHdlrs) {
         colHdlr.myUnilaterals.removeIf (cc -> cc.getAllMasters ().isEmpty ());
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
         ContactConstraint filtered_cc = getFilteredContactMasters (node, cc, isBackNode);
         if (filtered_cc.myMasters0.size () > 0 || filtered_cc.myMasters1.size() > 0) {
            n_ccs.add (cc);
         }
      }
      
      return n_ccs;
   }
   
   /**
    * Get contact masters (if any) within the given contact constraint cc
    * that are associated with the given node.
    * 
    * Recall that contact masters are usually instances of VertexContactMaster,
    * which either involves one node or 3 nodes.
    * 
    * @return
    * ContactConstraint with myMasters0 and myMasters1 populated.
    * Each myMasters list will contain 0 or 1 VertexContactMaster. 
    * In turn, every VertexContactMaster will have a PointParticleAttachment 
    * (filtered from the 1 or 3 nodes), which identifies the interested node, 
    * and associated weight.
    */
   protected ContactConstraint getFilteredContactMasters(FemNode3d node,
   ContactConstraint cc, boolean isBackNode) {
      ContactConstraint n_cc = new ContactConstraint();

      BackNode3d backNode = node.getBackNode ();
//      
      // CSNCMT
      // For each contact master in the constraint.
      for (int m = 0; m < 2; m++) {
         ArrayList<ContactMaster> masters = (m == 0) ? cc.myMasters0 : cc.myMasters1;
         
         // ContactMasters belonging to the given (n)ode.
         ArrayList<ContactMaster> n_masters = (m == 0) ? n_cc.myMasters0 : n_cc.myMasters1; 

         int vcm_idx = 0;
         for (ContactMaster cm : masters) {
            // DAN21:
            if (cm instanceof VertexContactMaster) {
               VertexContactMaster vcm = (VertexContactMaster)cm; 
              
               // For each ContactMaster list in the VertexContactMaster
               for (ArrayList<ContactMaster> vcm_cms : vcm.myMasterLists) {
                  
                  for (ContactMaster vcm_cm : vcm_cms) {
                     if (vcm_cm instanceof PointParticleAttachment) {
                        PointParticleAttachment ppa = (PointParticleAttachment)vcm_cm;
                        Particle particle = ppa.myParticle;
                        
                        if (!isBackNode && particle == node || isBackNode && particle == backNode) {
                           VertexContactMaster n_vcm = new VertexContactMaster();
                           n_vcm.myMasterLists = new ArrayList[] {new ArrayList<ContactMaster>()};
                           n_vcm.myMasterLists[0].add (ppa);
                           n_vcm.myWgts = new double[] {vcm.myWgts[vcm_idx]};
                           
                           n_masters.add (n_vcm);
                        }
                     } else {
                        throw new AssertionError("Unexpected ContactMaster type."); 
                     }
                  } 
               }
            } else {
               throw new AssertionError("Unexpected ContactMaster type."); 
            }
            
            vcm_idx++;
         }
         
         
         // CSNOLD
//         CollidableDynamicComponent cmComp = cm.getComp ();
//         
//         if (!isBackNode && cmComp == node) {
//            n_cms.add (cm);
//         }
//         else if (isBackNode && cmComp == backNode) {
//            n_cms.add (cm);
//         }
      }
//      
      return n_cc;
   }

   /* --- Convenient --- */
   
   protected VertexContactMaster createSingleNodeVertexContactMaster(Particle node, double wt) {
      VertexContactMaster vcm = new VertexContactMaster();
      vcm.myMasterLists = new ArrayList[] {new ArrayList<ContactMaster>()};
      vcm.myMasterLists[0].add (new PointParticleAttachment(node, null));
      vcm.myWgts = new double[] {wt};

      return vcm;
   }
   
   /* --- Helper Classes --- */
   
   protected class ShellOpRv extends OpRv {
      protected LinkedList<Double> mParentRestAreas;
      protected LinkedList<Matrix3d> mParentPlasticMembStrains; 
      protected LinkedList<Matrix3d> mParentPlasticBendStrains;
      
      public ShellOpRv() {
         super();
         init();
      }
      
      public ShellOpRv(int numAddedFaces, int numRemovedFaces) {
         super(numAddedFaces, numRemovedFaces);
         init();
      }
      
      protected void init() {
         this.mParentRestAreas = new LinkedList<Double>();
         this.mParentPlasticMembStrains = new LinkedList<Matrix3d>();
         this.mParentPlasticBendStrains = new LinkedList<Matrix3d>();
      }
      
      public Pair<Matrix3d, Matrix3d> getAvgParentPlasticStrains() {
         Matrix3d membAvg = new Matrix3d();
         Matrix3d bendAvg = new Matrix3d();
         
         double areaSum = 0;
         for (double area : this.mParentRestAreas) {
            areaSum += area;
         }
         
         for (int i = 0; i < this.mParentRestAreas.size (); i++) {
            double area = this.mParentRestAreas.get (i);
            double wt = area / areaSum;
            
            membAvg.scaledAdd (wt, this.mParentPlasticMembStrains.get (i));
            bendAvg.scaledAdd (wt, this.mParentPlasticBendStrains.get (i));
         }
         
         return new Pair<Matrix3d, Matrix3d>(membAvg, bendAvg);
      }
   }
   
   protected OpRv createOpRv() {
      return new ShellOpRv();
   }
   
   protected OpRv createOpRv(int numAddedFaces, int numRemovedFaces) {
      return new ShellOpRv(numAddedFaces, numRemovedFaces);
   }
   
   /* --- Interfacing --- */
   
   public FemModel3d getFemModel() {
      return this.mFemModel;
   }

}
