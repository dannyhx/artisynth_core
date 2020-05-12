package _custom.cont;

import java.util.ArrayList;

import artisynth.core.mechmodels.CollisionHandler;

/** Aggregation of unilateral constraints.
 * 
 * Maintains a reference to the unilaterals that were used to undo
 * the penetration points. These unilaterals are adjusted during remeshing
 * before they're applied to the subsequent constraint backward Euler 
 * solve.
 */
public class ContactConstraintAgg {
   
   /** Collision handler for each collidable body pair. Each handler has its
    *  own set of unilateral constraints. */
   public ArrayList<CollisionHandler> myColHdlrs;
   
   public ContactConstraintAgg() {
   }
   
   public ContactConstraintAgg(
   ArrayList<CollisionHandler> colHdlrs, boolean isShallowCopy) {
      set(colHdlrs, isShallowCopy);
   }
   
   public void set(ArrayList<CollisionHandler> colHdlrs, boolean isShallowCopy) {
      if (isShallowCopy) 
         myColHdlrs = new ArrayList<CollisionHandler>(colHdlrs);
      else 
         myColHdlrs = colHdlrs;
   }
   
   /** Remove the unilateral constraints from the collidiable body pairs. */
   public void clearConstraints() {
      for (CollisionHandler colHdlr : myColHdlrs) {
         colHdlr.myUnilaterals.clear ();
      }
   }
   
   /** Count the number of unilateral constraints. */
   public int numConstraints() {
      int cnt = 0;
      for (CollisionHandler colHdlr : myColHdlrs) {
         cnt += colHdlr.myUnilaterals.size ();
      }
      return cnt;
   }
   
   /**
    * Merge the unilateral constraints from another aggregation into this.
    * 
    * @param newCCAgg
    * Another aggregation of unilateral constraints that is to be merged into 
    * this.
    */
   public void addConstraints(ContactConstraintAgg newCCAgg) {
      if (newCCAgg.myColHdlrs == null) {
         return;
      }
      
      for (CollisionHandler newColHdlr : newCCAgg.myColHdlrs) {
         // Find the corresponding collision handler to merge with.
         for (CollisionHandler myColHdlr : myColHdlrs) {
            if (isSameCollidablePair(newColHdlr, myColHdlr)) {
               myColHdlr.myUnilaterals.addAll (newColHdlr.myUnilaterals );
            }
            
         }
      }
   }
   
   /** Are the collidable body pairs the same? */
   protected boolean isSameCollidablePair(CollisionHandler A, CollisionHandler B) {
      return (A.getCollidable (0) == B.getCollidable (0) && 
              A.getCollidable (1) == B.getCollidable (1));
   }
   
   public boolean isHasConstraints() {
      if (myColHdlrs == null) {
         return false;
      }
      
      for (CollisionHandler colHdlr : myColHdlrs) {
         if (colHdlr.numUnilateralConstraints () > 0) 
            return true;
      }
      return false;
   }
}