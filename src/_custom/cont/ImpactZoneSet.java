package _custom.cont;

import java.util.ArrayList;
import java.util.LinkedList;

import artisynth.core.mechmodels.ContactConstraint;
import artisynth.core.mechmodels.ContactMaster;

public class ImpactZoneSet {
 
   public class ImpactZone extends LinkedList<ContactConstraint> {
      private static final long serialVersionUID = 1L;
   }
   
   public ArrayList<ImpactZone> mIZs = new ArrayList<ImpactZone>();
   
   public ImpactZoneSet() {
      
   }
   
   public void addConstraint(ContactConstraint cc) {
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