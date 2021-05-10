package artisynth.demos.growth.collision;

import java.util.ArrayList;

import artisynth.core.femmodels.BackNode3d;
import artisynth.core.femmodels.FemElement.ElementClass;
import artisynth.core.femmodels.FemMeshComp;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.ShellElement3d;
import artisynth.core.mechmodels.CollidableBody;
import artisynth.core.mechmodels.ContactConstraint;
import artisynth.core.mechmodels.ContactMaster;
import artisynth.core.mechmodels.ContactPoint;
import artisynth.core.mechmodels.PointParticleAttachment;
import artisynth.core.mechmodels.VertexContactMaster;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;

public class CollisionHandlerCD {
   public static boolean IsCollidableBodyHasBackNodes(CollidableBody col) {
      if (col instanceof FemMeshComp) {
         FemModel3d model = ((FemMeshComp)col).getFem ();
         FemNode3d node = model.getNode (0);
         
         return node.getBackNode () != null;
      }
      
      return false;
   }
   
   public static boolean IsCollisionConstraintInvolveShellNode(
      ContactConstraint cc) 
   {
      for (int m = 0; m < 2; m++) {
         ArrayList<ContactMaster> masters = (m == 0) ? 
            cc.myMasters0 : cc.myMasters1;
         
         for (ContactMaster cm : masters) {
            VertexContactMaster vcm = (VertexContactMaster)cm; 
            for (ArrayList<ContactMaster> vcm_cms : vcm.myMasterLists) {
               for (ContactMaster vcm_cm : vcm_cms) {
                  if (vcm_cm instanceof PointParticleAttachment) {
                     PointParticleAttachment ppa = (PointParticleAttachment)cm;
                     if (
                        ppa.myParticle instanceof FemNode3d || 
                        ppa.myParticle instanceof BackNode3d) 
                     {
                        return true;
                     }
                  }
               }
            }
         }
      }
      
      return false;
   }
   
   public static void AlignContactPointsToBackNodes(ContactConstraint cc) {
      FemMeshComp col0 = (FemMeshComp) cc.col0;
      FemMeshComp col1 = (FemMeshComp) cc.col1;
      
      for (int i = 0; i < 2; i++) {
         ContactPoint cp = (i == 0) ? cc.myCpnt0 : cc.myCpnt1;
         FemMeshComp mesh = (i == 0) ? col0 : col1;
         
         Point3d avgFrontPos = new Point3d();
         Point3d avgBackPos = new Point3d();
         for (Vertex3d vtx : cp.getVertices ()) {
            FemNode3d node = mesh.getNodeForVertex (vtx);
            BackNode3d nodeb = node.getBackNode ();
            
            avgFrontPos.add (node.getPosition ());
            avgBackPos.add(nodeb.getPosition ());
         }
         avgFrontPos.scale (1.0/cp.numVertices ());
         avgBackPos.scale (1.0/cp.numVertices ());
         
         Vector3d front2back = new Vector3d();
         front2back.sub(avgBackPos, avgFrontPos);
         
         cp.getPoint ().add (front2back);
      }
   }
   
   
}
