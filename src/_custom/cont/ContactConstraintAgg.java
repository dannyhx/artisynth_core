package _custom.cont;

import java.util.ArrayList;

import artisynth.core.mechmodels.CollisionHandler;

public class ContactConstraintAgg {
   
   public ArrayList<CollisionHandler> myColHdlrs;
   
   public ContactConstraintAgg() {
      
   }
   
   public ContactConstraintAgg(ArrayList<CollisionHandler> colHdlrs, boolean isShallowCopy) {
      set(colHdlrs, isShallowCopy);
   }
   
   public void set(ArrayList<CollisionHandler> colHdlrs, boolean isShallowCopy) {
      if (isShallowCopy) 
         myColHdlrs = new ArrayList<CollisionHandler>(colHdlrs);
      else 
         myColHdlrs = colHdlrs;
   }
   
   public void clearConstraints() {
      for (CollisionHandler colHdlr : myColHdlrs) {
         colHdlr.myUnilaterals.clear ();
      }
   }
   
   /**
    * Merge the unilateral constraints from newCCAgg into this.
    */
   public void addConstraints(ContactConstraintAgg newCCAgg)
   {
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