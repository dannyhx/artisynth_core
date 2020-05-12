package artisynth.demos.growth.diffusion;

import java.util.ArrayList;

import artisynth.core.modelbase.HasNumericState;
import maspack.matrix.VectorNd;
import maspack.util.DataBuffer;

/** 
 * Collection of chemicals that belong to the vertices of a mesh. 
 * Accessor and setter methods are provided.  
 */
public class MeshChemicals implements HasNumericState {

   /** Number of vertices. */
   protected int mNumChemVtxs;
   
   /** Number of unique chemical types. */
   protected int mNumChemTypes;
   
   /** mChemVtxs.get(487).get(3) is the amount of chemical#3 for vertex#487 */
   protected ArrayList<VectorNd> mChemVtxs;   
   
   public MeshChemicals(int numChemVtxs, int numChemTypes) {
      mNumChemVtxs = numChemVtxs;
      mNumChemTypes = numChemTypes;
      mChemVtxs = new ArrayList<VectorNd>(numChemVtxs);
      for (int v = 0; v < numChemVtxs; v++) {
         mChemVtxs.add ( new VectorNd(numChemTypes) );
      }
   }
   
   public int getNumChemTypes() {return mNumChemTypes;}
   
   public double getVtxChem0(int vtxIdx) {
      return mChemVtxs.get (vtxIdx).get (0 /*chemIdx*/);
   }
   
   public void setVtxChem0(int vtxIdx, double amt) {
      mChemVtxs.get (vtxIdx).set (0 /*chemIdx*/, amt);
   }
   
   public double getVtxChem(int vtxIdx, int chemIdx) {
      return mChemVtxs.get (vtxIdx).get (chemIdx);
   }
   
   public void setVtxChem(int vtxIdx, int chemIdx, double amt) {
      mChemVtxs.get (vtxIdx).set (chemIdx, amt);
   }
   
   public VectorNd getVtxChems(int vtxIdx) {
      return mChemVtxs.get (vtxIdx);
   }
   
   public void addChemVtx(VectorNd chems) {
      if (mNumChemVtxs >= mChemVtxs.size ()) {
         mChemVtxs.add (chems);
      }
      else {
         mChemVtxs.set (mNumChemVtxs, chems);
      }
      
      mNumChemVtxs++;
   }
   
   public void removeChemVtx(int vtxIdx) {
      // Replace the deleted vertex with its subsequent vertex.
      // Do the same as well for the subsequent vertex, and so on.
      for (int i = vtxIdx; i < mNumChemVtxs-1; i++) {
         VectorNd nextChemVtx = mChemVtxs.get (i+1);
         mChemVtxs.set (i, nextChemVtx);
      }
      mChemVtxs.set (mNumChemVtxs-1, null);
      
      mNumChemVtxs--;
   }
   
   /* --- State --- */
   
   public void getState(DataBuffer buf) {
      buf.zput (mNumChemVtxs);
      buf.zput (mNumChemTypes);
      
      buf.zput (mChemVtxs.size ());
      for (int v=0; v < mChemVtxs.size (); v++)
         buf.dput (mChemVtxs.get (v));
   }
   
   public void setState(DataBuffer buf) {
      mNumChemVtxs = buf.zget ();
      mNumChemTypes = buf.zget ();
      
      mChemVtxs = new ArrayList<VectorNd>(mNumChemVtxs);
      for (int v=0; v < mChemVtxs.size (); v++) {
         VectorNd vtxChems = new VectorNd(mNumChemTypes);
         buf.dget (vtxChems);
         mChemVtxs.add ( vtxChems );
      }
   }
   
   public boolean hasState() {
      return true;
   }
}
