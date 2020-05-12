package _custom.cont;

import java.util.ArrayList;
import java.util.LinkedList;

import artisynth.core.mechmodels.CollisionHandler;
import artisynth.core.mechmodels.ContactConstraint;
import artisynth.core.mechmodels.ContactMaster;

/** Collection of impact zones. */
public class ImpactZoneSet {
 
   /** A single impact zone, which contains unilateral constraints that share
    *  common vertices.  */
   public class ImpactZone extends LinkedList<ContactConstraint> {
      private static final long serialVersionUID = 1L;
      
      /** For each unilateral constraint, place them back to their original
       *  belonging collidable body pair. */
      public void setImpactZoneConstraintsActive() {
         for (ContactConstraint cc : this) {
            cc.myCsnHldr.myUnilaterals.add (cc);
         }
      }
   }
   
   /** Container for the impact zones. */
   public ArrayList<ImpactZone> mIZs = new ArrayList<ImpactZone>();
   
   /** Create a collection of impact zones. 
    *
    * @param colHdlrs
    * Collection of collidable pairs. Unilateral constraints will be retrieved
    * from them, and grouped into impact zones.
    */
   public ImpactZoneSet(ArrayList<CollisionHandler> colHdlrs) {
      for (CollisionHandler colHldr : colHdlrs) {
         for (ContactConstraint cc : colHldr.myUnilaterals) {
            cc.myCsnHldr = colHldr;
            addConstraint(cc);
         }
      }
   }
   
   /** Remove all the unilateral constraints from the collection of 
    *  collidable pairs. */
   public static void clearConstraints(ArrayList<CollisionHandler> colHdlrs) {
      for (CollisionHandler colHldr : colHdlrs) {
         colHldr.myUnilaterals.clear ();
      }
   }
   
   /** Return the unilateral constraints back to their original belonging 
    *  collidable body pair.  */
   public void restoreConstraints(ArrayList<CollisionHandler> colHdlrs) {
      clearConstraints(colHdlrs);
      for (ImpactZone iz : mIZs) {
         iz.setImpactZoneConstraintsActive ();
      }
   }
   
   /** Bin the given unilateral constraint to its corresponding impact zone. */
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
   
   /** Given a unilateral constraint, get the impact zone that it is binned 
    *  into. */
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
   
   /** Do the two unilateral constraints share a common vertex? */
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