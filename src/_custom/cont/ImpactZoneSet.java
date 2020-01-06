package _custom.cont;

import java.util.ArrayList;
import java.util.LinkedList;

import artisynth.core.femmodels.BackNode3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.mechmodels.CollisionHandler;
import artisynth.core.mechmodels.ContactConstraint;
import artisynth.core.mechmodels.ContactMaster;
import artisynth.core.mechmodels.DynamicComponent;
import artisynth.core.mechmodels.Particle;

public class ImpactZoneSet {
 
   public class ImpactZone extends LinkedList<ContactConstraint> {
      private static final long serialVersionUID = 1L;
      
      public void setImpactZoneConstraintsActive(
      ArrayList<CollisionHandler> colHdlrs) {
         for (ContactConstraint cc : this) {
            cc.myCsnHldr.myUnilaterals.add (cc);
         }
      }
      
      public void setImpactZoneNodesDynamicOnly(ArrayList<DynamicComponent> comps) {
         for (DynamicComponent comp : comps) {
            if (! hasComp(comp)) {
               if (comp instanceof FemNode3d) {
                  ((FemNode3d)comp).setDynamic (false);
               }
               else if (comp instanceof BackNode3d) {
                  ((BackNode3d)comp).setDynamic (false);
               }
               else {
                  assert(false);
               }
            }
         }
      }
      
      protected boolean hasComp(DynamicComponent comp) {
         for (ContactConstraint cc : this) {
            for (ContactMaster cm : cc.getMasters ()) {
               if (cm.getComp () == comp) {
                  return true;
               }
            }
         }
         return false;
      }
   }
   
   public ArrayList<ImpactZone> mIZs = new ArrayList<ImpactZone>();
   
   public ImpactZoneSet(ArrayList<CollisionHandler> colHdlrs) {
      for (CollisionHandler colHldr : colHdlrs) {
         for (ContactConstraint cc : colHldr.myUnilaterals) {
            cc.myCsnHldr = colHldr;
            addConstraint(cc);
         }
      }
   }
   
   public static void clearConstraints(ArrayList<CollisionHandler> colHdlrs) {
      for (CollisionHandler colHldr : colHdlrs) {
         colHldr.myUnilaterals.clear ();
      }
   }
   
   public void restoreConstraints(ArrayList<CollisionHandler> colHdlrs) {
      clearConstraints(colHdlrs);
      for (ImpactZone iz : mIZs) {
         iz.setImpactZoneConstraintsActive (colHdlrs);
      }
   }
   
   public static void loadDynamics(ArrayList<DynamicComponent> comps) {
      for (DynamicComponent comp : comps) {
         if (comp instanceof FemNode3d) {
            ((FemNode3d)comp).setDynamic (((FemNode3d)comp).wasDynamic);
         }
         else if (comp instanceof BackNode3d) {
            ((BackNode3d)comp).setDynamic (((BackNode3d)comp).wasDynamic);
         }
         else {
            assert(false);
         }
      }
   }
   
   public static void saveDynamics(ArrayList<DynamicComponent> comps) {
      for (DynamicComponent comp : comps) {
         if (comp instanceof FemNode3d) {
            ((FemNode3d)comp).wasDynamic = comp.isDynamic ();
         }
         else if (comp instanceof BackNode3d) {
            ((BackNode3d)comp).wasDynamic = comp.isDynamic ();
         }
         else {
            assert(false);
         }
      }
   }
  
   protected void addConstraint(ContactConstraint cc) {
      ImpactZone iz = findCorrespondingImpactZone (cc);
      if (iz == null) {
         ImpactZone newIz = new ImpactZone();
         newIz.add (cc);
         mIZs.add (newIz);
         return;
      }
      
      iz.add (cc);
   }
   
   protected ImpactZone findCorrespondingImpactZone(ContactConstraint cc) {
      for (ImpactZone iz : mIZs) {
         for (ContactConstraint cur_cc : iz) {
            if (isConstraintPairShareCommonVertex(cc, cur_cc)) {
               return iz;
            }
         }
      }
      
      return null;
   }
   
   protected boolean isConstraintPairShareCommonVertex(ContactConstraint A,
   ContactConstraint B) {
      
      for (ContactMaster cmA : A.getMasters ()) {
         for (ContactMaster cmB : B.getMasters ()) {
            if (cmA.getComp () == cmB.getComp ()) {
               return true;
            }
         }
      }
      
      return false;
   }
   
}