package artisynth.demos.growth.models.paper;

import java.awt.Color;
import java.util.ArrayList;

import artisynth.demos.growth.GrowNode3d;
import artisynth.demos.growth.collision.CollisionDetector;
import artisynth.demos.growth.collision.ContinuousRenderable;
import artisynth.demos.growth.models.base.GrowDemo;
import artisynth.demos.growth.util.MeshUtil;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point2d;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderer;

// artisynth.models.plants.growth.paperdemos.Intricate_RippleBouquet
public class Intricate_RippleBouquet extends GrowDemo {
   
   /** Use 0, 8.33, 16.66, and 25.00 as time increments 
    * Color = 5th left block from most-yellow */

   /* Figure uses default polarity. Use mPolDir=(0,0,1) to grow upwards */
   
   /*
   Top-Down:
          mCameraEye = new Point3d(0.00840759, 0.00982214, 10.2219);
          mCameraCenter = new Point3d(0.00528906, 0.0176713, -0.384257);  
    
   Perspective:
          mCameraEye = new Point3d(1.55666, -8.16246, 4.52825);
          mCameraCenter = new Point3d(-0.0481113, -0.20805, -0.559241);
          
   Side:
      mCameraEye = new Point3d(0.0316826, -8.00221, -0.49313);
      mCameraCenter = new Point3d(0.0316826, 0.0176713, -0.49313);
      
   Top-Down (no edge exp):
      mCameraEye = new Point3d(0.0322226 0.00913619 10.2464);
      mCameraCenter = new Point3d(0.0316826 0.0176713 -0.49313);  
   */
   
   protected double mMorphogenSrcConc = 0.5;
   
   protected void build_pre() {
      super.build_pre();
      
      mIsShowRefMesh = false;
      
      m_shellThickness = 0.001;
      m_youngsModulus = 1e7; 
      
      mDiffusionTimestepScale = 0.002;
      
      this.mPenetrationTol = -5e-2;
      
      mSizeMin = 0.25;
      mSizeMax = mSizeMin*100;
      mMorphogenSrcConc = 0.5;
      mPauseEveryInterval = 8.33;
      
      // Collision partitioning test 
      mEnableCollisionHandling = false;
      mEnableContinuousDetection = false;
      
      // -disableHybridSolves
      
      // DEBUG
      mPauseEveryInterval = 25;
      m_youngsModulus = 1e5;
      mShowColorBar = false;
      
      // Test for consistency 
      mRemeshFreq = 0.25;
      
      // Camera
      mCameraEye = new Point3d(0.0322226, 0.00913619, 10.2464);
      mCameraCenter = new Point3d(0.0316826, 0.0176713, -0.49313);  
   }
   
   protected void build_modelSkeleton() {

      ArrayList<PolygonalMesh> meshes = new ArrayList<PolygonalMesh>();
      
      int numRings = 5;
      int baseDiv = 3;
      double radBase = 1;
      int resolBase = 1;   // Fixed
      
      double radGap = 0.1;
      double stripRad = 0.1;
      double zGap = 0.1;
      
      double numSubLayers = 1;
      
      Point3d baseBotPnt = null;
      
      resolBase = 3;  // Needed?

      RigidTransform3d X = new RigidTransform3d();
      for (int i=0; i<numRings; i++) {
         for (int a=0; a<numSubLayers; a++) {
            double radRef = radBase/resolBase;
            
            PolygonalMesh layer = MeshFactory.createOctadecahedralSphere (
               radBase+(radRef)*(i), resolBase+i);  // radBase*(3+1.5*i) , resolBase+i+1
            
            _Exp_Toolbox.drillDown(layer, 0.25, 0, Double.POSITIVE_INFINITY);

            // Align to base bottom point 
            
            if (baseBotPnt == null) {
               baseBotPnt = new Point3d( _Exp_Toolbox.getBotPoint(layer) );
            }
            else {
               Point3d curBotPnt = _Exp_Toolbox.getBotPoint(layer);
               Vector3d toBaseBotPnt = new Vector3d(baseBotPnt).sub (curBotPnt);
               X.setIdentity ();
               X.addTranslation (toBaseBotPnt);
               X.addTranslation (0,0, i*-zGap);
               layer.transform (X);
            }
            
            // Form leaves
            
            double minZ = -1;  // -1
            if (i==0)
               minZ = -0.50;
            
            _Exp_Toolbox.trimStrip(layer, Point2d.ZERO,
               (Point2d)(new Point2d(-1.448, 2.509).scale (10)), stripRad, minZ);
            
            _Exp_Toolbox.trimStrip(layer, Point2d.ZERO,
               (Point2d)(new Point2d(-1.448, -2.509).scale (10)), stripRad, minZ);
            
            _Exp_Toolbox.trimStrip(layer, Point2d.ZERO,
               (Point2d)(new Point2d(3, 0).scale (10)), stripRad, minZ);
            
            // Rotate
            double rotAng = i*45 % 360;
            X.setIdentity ();
            X.mulRotZ (Math.toRadians (rotAng));
            layer.transform (X);

            // Cut the top half
            double maxZ = 0;
            _Exp_Toolbox.trimVertically (layer, Double.NEGATIVE_INFINITY, maxZ);
            
            meshes.add (layer);
         }
      }
      
      M = meshes.size ();
      mMesh = new PolygonalMesh[M];
      for (int m=0; m<M; m++) {
         mMesh[m] = meshes.get (m);
      }
      
//      M = 1;
//      mMesh = new PolygonalMesh[M];
//      mMesh[0] = new PolygonalMesh();
//      for (int m=0; m<meshes.size (); m++) {
//         mMesh[0].addMesh( meshes.get (m) );
//      }
   }
   
   protected void build_renderConfig() {
      super.build_renderConfig ();
      
      RenderConfig defaultCfg = mRendCfgPresets.get (RenderMode.DEFAULT);
      defaultCfg.mNodeRadius = 0.01;
      
      mRendCfg = mRendCfgPresets.get(RenderMode.MORPHOLOGY);
      mRendCfg.mDrawEdges = false;
      mRendCfg.mNodeRadius = 0;
   }
   
   protected void build_femRendering() {
      super.build_femRendering ();
      
      Color[] frontColors = new Color[] {
        new Color(255,0,102),  
        new Color(255,204,204),
        new Color(255,51,153),
        new Color(204,153,153),
        new Color(255,51,51),
      };
      
      Color[] backColors = new Color[] {
        new Color(255,127,179),
        new Color(255,229,229),
        new Color(255,153,204),
        new Color(229,204,204),
        new Color(255,153,153),
      };

      for (int m=0; m<M; m++) {
         RenderProps.setFaceColor (mFemModel[m], frontColors[m]);
         RenderProps.setBackColor (mFemModel[m], backColors[m]);
      }
   }
   
   protected void build_post() {
      for (int m=0; m<M; m++) {
         for (int v = 0; v < mMesh[m].numVertices (); v++) {
            if ( isMorphogenSrcNode(m,v) ) {
               GrowNode3d gNode = (GrowNode3d)mFemModel[m].getNode (v);
               gNode.mIsMorphogenSrc = true;
            }
         }
      }
     
      super.build_post ();
   }
   
   public void advanceCustom(double t0, double t1, int flags) {      
      for (int m=0; m<M; m++) {
         for (int v = 0; v < mMesh[m].numVertices (); v++) {
            GrowNode3d gNode = (GrowNode3d)mFemModel[m].getNode (v);
            if (gNode.mIsMorphogenSrc) {
               gNode.mChems.set (3, mMorphogenSrcConc);
            }
         }
      }
      
      super.advanceCustom (t0, t1, flags);
   }
   
   public boolean isMorphogenSrcNode(int m, int v) {
      Vertex3d vtx = mMesh[m].getVertex (v);
      Point3d pnt = mFemModel[m].getNode (v).getPosition ();
      
      return (pnt.z > -2.00) && MeshUtil.isBoundaryVtx (vtx);
   }
   
   public void render(Renderer renderer, int flags) {
      
      ContinuousRenderable contRend = CollisionDetector.mContRend;
      if (contRend != null) {
//         contRend.render (renderer, flags);
      }

      super.render (renderer, flags);
   }
    
}
