package artisynth.demos.growth.models.paper;

import java.awt.Color;

import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.RigidTransform3d;

public class _Figure_Plasticity extends Intricate_Fruit {

   protected void build_modelSkeleton() {
      RigidTransform3d tns = new RigidTransform3d();
      tns.mulRotX (45);
        
      mMesh = new PolygonalMesh[M];
      mMesh[0] = MeshFactory.createIcosahedralSphere (0.5, 2);
   }
   
   protected void build_pre() {
      super.build_pre();
      
      this.mRemeshFreq = 1000000;
   }
   
   protected void build_renderConfig() {
      super.build_renderConfig ();
      
      RenderConfig cfg = mRendCfgPresets.get (RenderMode.DEFAULT);
      cfg.mNodeRadius = 0.01;
      
      cfg = mRendCfgPresets.get (RenderMode.MORPHOLOGY);
      cfg.mFrontMeshColor = Color.yellow;
      cfg.mBackgroundColor = Color.white;
      cfg.mDrawEdges = true;
   }
   
}
