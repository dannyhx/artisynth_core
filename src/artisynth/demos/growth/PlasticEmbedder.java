package artisynth.demos.growth;

import artisynth.core.femmodels.FemElement.ElementClass;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.ShellElement3d;
import artisynth.core.femmodels.ShellTriElement;
import artisynth.core.modelbase.ComponentChangeEvent;
import artisynth.core.modelbase.ComponentChangeEvent.Code;
import artisynth.demos.growth.thinshell.EdgeDataMap;
import artisynth.demos.growth.thinshell.EdgeDataMap.EdgeData;
import artisynth.demos.growth.util.MathUtil;
import artisynth.demos.growth.util.MeshUtil;
import artisynth.demos.growth.util.ShellUtil;
import maspack.geometry.Face;
import maspack.geometry.HalfEdge;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Vector3d;

/**
 * Provides ability to perform a secondary simulation where rest positions are
 * altered via plastic forces. 
 */
public class PlasticEmbedder {
   
   protected FemModel3d mFemModel;
   protected PolygonalMesh mMesh;
   protected Boolean mHasBackNodes;
   
   public PlasticEmbedder() {
   }
   
   public void setTarget(FemModel3d femModel) {
      mFemModel = femModel;
   }
   
   public void setTarget(FemModel3d femModel, PolygonalMesh mesh) {
      mFemModel = femModel;
      mMesh = mesh; 
   }
   
   // --- Primary Functions ---
   
   /** 
    * Advance the reference configuration, according to the plastic forces. 
    * 
    * Plastic strain is decremented accordingly whenever as rest shape is 
    * altered.
    * */
   public void advance(double t0, double t1) {
      backupWorldSpace ();
      initReferenceSpaceAdvance ();
      
      mFemModel.advance (t0, t1, /*flags=*/0);
      
      for (ShellElement3d ele : mFemModel.getShellElements ()) {
         ((GrowTriElement)ele).useResidualPlasticStrain ();
      }
      
      concludeReferenceSpaceAdvance();
      
      ComponentChangeEvent compEvt = new ComponentChangeEvent(Code.STRUCTURE_CHANGED);
      mFemModel.componentChanged (compEvt);
   }
   
   /**
    * Set the rest configuration to be identical to the world configuration.
    * 
    * Plastic strain is decremented according to the change in rest 
    * configuration.
    * 
    * This method is an alternative to advance() where the world configuration
    * acts like the growing reference configuration. The actual reference
    * configuration simply mirrors the world configuration. This is 
    * particularly useful if you want to simulate growth/plasticity 
    * without any elasticity. 
    */
   public void copyWorldSpaceToReferenceSpace() {
      // Assume world-space has shifted by a timestep.
      
      for (ShellElement3d ele : mFemModel.getShellElements ()) {
         ((GrowTriElement)ele).useResidualPlasticStrain ();
      }
      
      for (FemNode3d node : mFemModel.getNodes ()) {
         GrowNode3d gNode = (GrowNode3d)node; 
       
         gNode.setRestPosition ( gNode.getPosition () );        
         gNode.setVelocity ( Vector3d.ZERO );
         
         if (hasBackNode()) {
            gNode.setBackRestPosition ( gNode.getBackPosition () ); 
            gNode.setBackVelocity ( Vector3d.ZERO );
         }
      }
      
      if (mFemModel.myThinShellAux != null) {
         for (HalfEdge edge : EdgeDataMap.getMeshRealHalfEdges (mMesh)) {
            double curAng = ShellUtil.getDihedralAngle (mFemModel, edge, false);
            mFemModel.myEdgeDataMap.get (edge).mRestTheta = curAng;
         }
      }
      
      ComponentChangeEvent compEvt = new ComponentChangeEvent(
         Code.STRUCTURE_CHANGED);
      mFemModel.componentChanged (compEvt);
      
      ShellUtil.invalidateFem (mFemModel);
   }
   
   /* --- Manipulating nodal DoFs --- */
   
   /** Create a backup of the world configuration. */
   public void backupWorldSpace() {
      for (FemNode3d node : mFemModel.getNodes ()) {
         GrowNode3d gNode = (GrowNode3d)node; 
         
         gNode.m_WS_front_pos.set ( gNode.getPosition () );
         gNode.m_WS_front_restPos.set ( gNode.getRestPosition () );         
         gNode.m_WS_front_vel.set ( gNode.getVelocity () );
         
         if (hasBackNode()) {
            gNode.m_WS_back_pos.set ( gNode.getBackPosition () );
            gNode.m_WS_back_restPos.set ( gNode.getBackRestPosition () );
            gNode.m_WS_back_vel.set ( gNode.getBackVelocity () );
         }
      }
      
      // Note that backing up the world positions will implicitly 
      // backup the world dihedral angles.
      // But the rest dihedral angles should still be backed up.
      if (mFemModel.myThinShellAux != null) {
         for (HalfEdge edge : EdgeDataMap.getMeshRealHalfEdges (mMesh)) {
            EdgeData edgeData = mFemModel.myEdgeDataMap.get (edge);
            edgeData.m_WS_restTheta = edgeData.mRestTheta;
         }
      }
   }
   
   /**
    * Switch to optimization mode where both reference and world space are set 
    * to the current rest configuration.
    * 
    * Afterwards, FEMModel.advance() should be called to find the new rest
    * configuration, subjected to plastic forces.
    * 
    * @precond backupWorldSpace() 
    */
   public void initReferenceSpaceAdvance() {
      // Copy the rest DoF to the current DoF
      for (FemNode3d node : mFemModel.getNodes ()) {
         GrowNode3d gNode = (GrowNode3d)node; 
         
         /* --- Use Embedded-Space --- */
         
         // Rest (unoptimized)
         gNode.setRestPosition ( gNode.m_WS_front_restPos );
         
         // Cur (soon-to-be optimized result)
         gNode.setPosition ( gNode.m_WS_front_restPos );
 
         gNode.setVelocity ( gNode.m_ES_front_vel );
         gNode.setVelocity ( Vector3d.ZERO );
         
         if (hasBackNode()) {
            gNode.setBackRestPosition ( gNode.m_WS_back_restPos );
            gNode.setBackPosition ( gNode.m_WS_back_restPos );
            gNode.setBackVelocity ( gNode.m_ES_back_vel );
            gNode.setBackVelocity ( Vector3d.ZERO );
         }
      }
      
      if (mFemModel.myThinShellAux != null) {
         for (HalfEdge edge : EdgeDataMap.getMeshRealHalfEdges (mMesh)) {
            EdgeData edgeData = mFemModel.myEdgeDataMap.get (edge);
            edgeData.mRestTheta = edgeData.m_WS_restTheta;
         }
      }
      
      ShellUtil.invalidateFem (mFemModel);   
   }
   
   /**
    * End the optimization mode where reference-space is set to the newly
    * optimized (i.e. advanced) rest configuration and the world-space is
    * restored from the backup copy. 
    * 
    * @precond initReferenceSpaceAdvance()
    */
   public void concludeReferenceSpaceAdvance() {
      for (FemNode3d node : mFemModel.getNodes ()) {
         GrowNode3d gNode = (GrowNode3d)node; 
       
         /* --- Backup Embedded-Space velocity and force --- */
         
         gNode.m_ES_front_vel.set( gNode.getVelocity () );
         
         /*--- Use World-Space --- */
         
         // Copy the current DoF (i.e. energy-minimized embedded space) to 
         // the rest DoF (i.e. material space for world space).
         gNode.setRestPosition ( gNode.getPosition () );        

         // Restore the current DoF of the world space.
         gNode.setPosition ( gNode.m_WS_front_pos );
         
         // Restore world-space velocity and force.  
         // DANTEST. This wasn't included originally.
         gNode.setVelocity ( gNode.m_WS_front_vel );
         
         if (hasBackNode()) {
            gNode.m_ES_back_vel.set( gNode.getBackVelocity () );
            gNode.setBackRestPosition ( gNode.getBackPosition () ); 
            gNode.setBackPosition ( gNode.m_WS_back_pos );
            gNode.setBackVelocity ( gNode.m_WS_back_vel );
         }
      }
      
      // Copy energy-minimized embedded edges to the rest edges.
      if (mFemModel.myThinShellAux != null) {
         for (HalfEdge edge : EdgeDataMap.getMeshRealHalfEdges (mMesh)) {
            double curAng = ShellUtil.getDihedralAngle (mFemModel, edge, false);
            mFemModel.myEdgeDataMap.get (edge).mRestTheta = curAng;
         }
      }
      
      ShellUtil.invalidateFem (mFemModel);
   }
   
   // --- Secondary Functions
   
   protected boolean hasBackNode() {
      if (this.mHasBackNodes == null) {
         this.mHasBackNodes = (
         mFemModel.getShellElement (0).getElementClass () == ElementClass.SHELL);
      }
      
      return this.mHasBackNodes;
   }
}
