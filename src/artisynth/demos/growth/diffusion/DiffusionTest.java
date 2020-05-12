package artisynth.demos.growth.diffusion;

import java.awt.Color;

import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.StepAdjustment;
import artisynth.core.workspace.RootModel;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;

// -model artisynth.models.diffusion.DiffusionTest

public class DiffusionTest extends RootModel {
   
   /* Diffusion */
   
   protected Diffusion mDiffusion;
   protected MeshChemicals mMeshChems;
   
   /* Mesh */
   
   protected MechModel mModel;
   protected RigidBody mRigidBody;
   protected PolygonalMesh mMesh;

   /* Mesh Rendering */
   
   protected final Color mRearMeshColor = Color.RED;
   protected final Color mMeshEdgeColor = Color.BLUE;
   
   /* Particles */
   
   protected ChemicalParticle[] mChemParticles;
   protected final int mNumChemTypes = 1;
   
   /* Particle Rendering */
   
   protected final double mParticleRadius = 0.4;
   protected final Color mParticleColor = Color.CYAN;
   protected final double mParticleMass = 1;
   
   /* Physics */
   
   protected Vector3d mGravity = Vector3d.ZERO;
   
   
   @Override
   public void initialize (double t) {
      super.initialize (t);
      
      if (mModel != null) {
         clear ();
         build(null);
      }
   }

   public void build (String[] args) {
      System.out.println ("build");
      build_modelStructure();
      build_modelRendering();
      build_post();
   }
   
   /* --- Build Helpers --- */

   protected void build_modelStructure() {
      mMesh = MeshFactory.createPlane (30,30,30,30);

      mRigidBody = new RigidBody();
      mRigidBody.setMesh(mMesh);
      
      mModel = new MechModel ("mech");
      mModel.addRigidBody(mRigidBody);
      
      mModel.setGravity(mGravity);
      addModel(mModel);
   }
   
   protected void build_modelRendering() {
      /* Make mesh edges visible*/
      mRigidBody.getRenderProps().setEdgeColor(mMeshEdgeColor);
      mRigidBody.getRenderProps().setDrawEdges(true);
      
      /* Create a particle for each mesh vertex */
      mChemParticles = new ChemicalParticle[mMesh.numVertices()];
      for (int i = 0; i < mMesh.numVertices(); i++) {
         Vertex3d v = mMesh.getVertices().get(i);
         ChemicalParticle p = new ChemicalParticle(mParticleMass, 
            v.getPosition(), mNumChemTypes);
         p.setDynamic (false);
         /* Make particle visible using a sphere */
         RenderProps.setSphericalPoints(p, mParticleRadius, mParticleColor);
         mChemParticles[i] = p;
         mModel.addParticle(p);
      }
      
      mRigidBody.setDynamic (false);
   }
   
   protected void build_post() {
      mMeshChems = new MeshChemicals(mMesh.numVertices (), mNumChemTypes);
      mDiffusion = new Diffusion(mMesh, mMeshChems);
      
      for (int v = 0; v < mMesh.numVertices (); v++) {
         mChemParticles[v].setChems (mMeshChems.getVtxChems (v));
      }
      
      mMeshChems.setVtxChem0 (220, 1);
      updateChemicalParticleColors();
   }
   
   /* --- Advance --- */
   
   @Override
   public StepAdjustment advance (double t0, double t1, int flags) {  
      mDiffusion.advance (t1-t0);     
      
      mMeshChems.setVtxChem0 (220, 1);
      mMeshChems.setVtxChem0 (440, -1);
      updateChemicalParticleColors();
      return super.advance(t0,t1,flags);
   }
   
   /* --- Util --- */
   
   protected void updateChemicalParticleColors() {
      // Update node colors according to their chemical levels
      for (int v = 0; v < mMesh.numVertices(); v++) {
         double amt = mMeshChems.getVtxChem0 (v);
         mChemParticles[v].setChem0 (amt);
         float amtPct = (float)convertAmtToColorSaturation(amt);
         mChemParticles[v].getRenderProps ().setPointColor (
            Color.getHSBColor (1.f, amtPct, 1.f));
      }
   }
   
   protected double convertAmtToColorSaturation(double amt) {
      double max = 1;
      if (amt > max) {
         return 1;
      }
      else if (amt < 0) {
         return 0;
      }
      else {
         return amt / max;
      }
   }
}
