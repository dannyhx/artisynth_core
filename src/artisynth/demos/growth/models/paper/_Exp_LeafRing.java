// Same size triangles

package artisynth.demos.growth.models.paper;

import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

import java.awt.Color;
import java.util.ArrayList;

import artisynth.core.driver.Main;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.modelbase.ModelComponentBase;
import artisynth.demos.growth.GrowNode3d;
import artisynth.demos.growth.models.base.GrowDemo;
import artisynth.demos.growth.util.MeshUtil;
import maspack.geometry.Face;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.Renderer.Shading;

// artisynth.models.plants.growth.paperdemos.LeafRing
public class _Exp_LeafRing extends GrowDemo {
   
   /** Use 0, 8.33, 16.66, and 25.00 as time increments 
    * Color = 5th color on left of most yellow */

   protected double mMorphogenSrcConc = 0.5;
   
   protected int mNumRings = 5;
   protected int mSubDivide = 2;
   
   protected boolean mDomeCollisionOnly = false;
   
   protected void build_pre() {
      super.build_pre();
      
      mIsShowRefMesh = false;
      
      m_shellThickness = 0.001;
      m_youngsModulus = 1e5; 
      
      mDiffusionTimestepScale = 0.01;
      
      mSizeMin = 0.20;
      mSizeMax = mSizeMin*100;
      
      mEnableCollisionHandling = true;
      
      mEnableProximityDetection = true;
      mEnableContinuousDetection = false;
      mEnableImpactZoneDetection = true;
      
      mEnableSelfCollision = false;
      
      
      // -disableHybridSolves 
      // 4 OMP_NUM_THREADS
      
      /* Observations:
       *   - if youngs modulus too high, it will oscilate a lot around
       *     clamp node.
       */

      // High res good
//      mSizeMin = 0.25;
//      mSizeMax = mSizeMin*100;
//      mMorphogenSrcConc = 1.5;  // 1.1 ->
//      m_shellThickness = 2e-3;
//      m_youngsModulus = 5e5;
//      mSubDivide = 2;
//      mNumRings = 1;
//      mRemeshFreq = 0.25;
//      mPenetrationTol = -5e-2;
//      mPauseEveryInterval = 40;  // 43
//      
//      mDomeCollisionOnly = false;
//      
//      ContinuousCollider.myImpactZonePenetrationTol = -1e-3;
      
      
      
      // 0.35 res good (single thread)
       mSizeMin = 0.35;
       mSizeMax = mSizeMin*100;
       mMorphogenSrcConc = 1.1;  
       m_shellThickness = 1e-3;
       m_youngsModulus = 1e5;
       mSubDivide = 2;
       mNumRings = 1;
       mRemeshFreq = 0.25;
       mPenetrationTol = -8e-2;
       mPauseEveryInterval = 40;  // 43
      
       mShowColorBar = false;
       
      // Low res good
//      mSizeMin = 0.40;
//      mSizeMax = mSizeMin*100;
//      mMorphogenSrcConc = 1.0;  // 1.1 -> 
//      m_shellThickness = 1e-3;
//      m_youngsModulus = 1e5;
//      mSubDivide = 2;
//      mNumRings = 1;
//      mRemeshFreq = 0.25;
//      mPenetrationTol = -7.5e-2;
//      mPauseEveryInterval = 40;  // 43
      
      mDomeCollisionOnly = false;
      
      // Top 
//      mCameraEye = new Point3d(0, 0, 8.09756);
//      mCameraCenter = new Point3d(0,0,0);
      
      // Side
   }

   boolean isDome = true;
   
   protected void build_modelSkeleton() {
      ArrayList<PolygonalMesh> meshes = new ArrayList<PolygonalMesh>();
      
      int n = 8; // 8
      double apothem = 1;
      
      double sideLen = sideLen(n, apothem);
      
      double vertGapBtRings = 0.4;  // 0.2
      double vertAltGapBtSide = 0.10; // 0.05
      
      double vertStart = -1;
      
      // For each polygonal side 
      for (int s=0; s<n; s++) {
         
         double rot = s*(2*PI/n);
         double curVert = vertStart + vertAltGapBtSide*Math.pow (-1, s);
         
         // For each level
         for (int r=0; r<mNumRings; r++) {

            // Create triangle
            Face face = equilateralTriangle (sideLen);
            Vertex3d[] triVtxs = face.getTriVertices ();
            
            PolygonalMesh leaf = new PolygonalMesh();
            
            leaf.addVertex (triVtxs[0]);
            leaf.addVertex (triVtxs[1]);
            leaf.addVertex (triVtxs[2]);
            leaf.addFace (triVtxs);
            
            leaf = MeshFactory.subdivide (leaf, mSubDivide);
            _Exp_Toolbox.trimRangeY (leaf, Double.NEGATIVE_INFINITY, 0.38);
            flagVertices(leaf);
            
            // Prepare for transform
            RigidTransform3d X = new RigidTransform3d();
             
            // Transform to level
            leaf.translate (new Vector3d(0,0,curVert));
            curVert -= vertGapBtRings;
             
            // Translate away from center.
            leaf.translate (new Vector3d(0,apothem*0.75,0));  // 0.75*apothem
            
            // Rotate to its polygon side
            X.setIdentity ();
            X.mulRotZ (rot);
            leaf.transform(X);

            meshes.add (leaf);
         }
      }
      
      
      // Add dome
      if (isDome) {
         // Observation: Still get assymetry using different dome changes.
         // Assymetry likely due to the initial twisting caused by buckling.
         PolygonalMesh dome = MeshFactory.createIcosahedralSphere (2.00, 3);  // 2.00
         meshes.add (dome);
      }

      M = meshes.size ();
      mMesh = new PolygonalMesh[M];
      for (int m=0; m<M; m++) {
         mMesh[m] = meshes.get (m);
//       MeshToolbox.trimVertically (mMesh[m], Double.NEGATIVE_INFINITY, 0.17);
      }
   }
   
   protected int mFrozenFlag = ModelComponentBase.createTempFlag ();
   protected int mNoMphZoneFlag = ModelComponentBase.createTempFlag ();
   protected int mMphSrcFlag = ModelComponentBase.createTempFlag ();
   
   protected void flagVertices(PolygonalMesh mesh) {
      for (Vertex3d vtx : mesh.getVertices ()) {
         Point3d pos = vtx.getPosition ();
         
//         if (pos.y < -0.22) {
//            vtx.setFlag (mFrozenFlag);
//         }
         if (vtx.getIndex () == 3) {
            vtx.setFlag (mFrozenFlag);
         }
         
//         if (pos.y < 0.21) {
//            vtx.setFlag (mNoMphZoneFlag);
//         }
         
         if (pos.y > 0.21) {
            vtx.setFlag (mMphSrcFlag);
         }
      }
   }
   
   protected double sideLen(int n, double apothem) {
      return (apothem / cos (PI/n)) * 2*sin (PI/n);
   }
   
   protected Face equilateralTriangle(double sideLen) {
      // Left
      Vertex3d vtx0 = new Vertex3d(0,0,0);
      
      // Right
      Vertex3d vtx1 = new Vertex3d(sideLen,0,0);
      
      // Top
      Vertex3d vtx2 = new Vertex3d(sideLen/2,sin (toRadians (60)) * sideLen, 0);
      
      Point3d centroid = new Point3d();
      centroid.add (vtx0.pnt);
      centroid.add (vtx1.pnt);
      centroid.add (vtx2.pnt);
      centroid.scale (1/3.0);
      
      Vector3d cent2origin = new Vector3d().sub(centroid);
      vtx0.pnt.add (cent2origin);
      vtx1.pnt.add (cent2origin);
      vtx2.pnt.add (cent2origin);

      return MeshUtil.createFace (vtx0, vtx1, vtx2, Vector3d.Z_UNIT);
   }
   
  
   protected void build_renderConfig() {
      super.build_renderConfig ();
      
      RenderConfig defaultCfg = mRendCfgPresets.get (RenderMode.DEFAULT);
      defaultCfg.mNodeRadius = 0.01;
      
      RenderConfig morphCfg = mRendCfgPresets.get (RenderMode.MORPHOLOGY);
      morphCfg.mFrontMeshColor = new Color(0,255,255);
      morphCfg.mRearMeshColor = new Color(0,255,0);
      morphCfg.mDrawEdges = false;
      morphCfg.mBackgroundColor = new Color(0,255,0);
      
      mRendCfg = morphCfg;
   }
   
   protected void build_femRendering() {
      super.build_femRendering ();
      
      Color[] frontColors = new Color[] {
        new Color(127,255,127),  
        new Color(255,229,203),
        new Color(140,255,101),
        new Color(178,255,127),
        
//        new Color(101,255,255),
        new Color(204,204,255),
        
        new Color(255,178,178),
        new Color(255,255,127),
        new Color(101,255,140),
      };
      
      Color[] backColors = new Color[] {
        new Color(0,255,0),
        new Color(255,204,153),
        new Color(51,204,0),
        new Color(102,255,0),
        
        new Color(0,153,153),
        new Color(255,102,102),
        new Color(255,255,0),
        new Color(0,204,51)
      };
      
      for (int m=0; m<M-1; m++) {
         RenderProps.setFaceColor (mFemModel[m], frontColors[m]);
         RenderProps.setBackColor (mFemModel[m], backColors[m]);
      }
      
//      // Unique color for each leaf 
//      float h = 0.0f;
//      float s = 0.7f;
//      float v = 0.7f;
//      
//      boolean tick = true;
//      for (int i=0; i<(M/2)+1;) {
//         int m = -1;
//         if (tick) {
//            m = i;
//         } 
//         else {
//            m = (M-1)-i;
//            i++;
//         }
//         tick = !tick;
//         
//         Color frontColor = Color.getHSBColor (h, s/1.5f, v);
//         RenderProps.setFaceColor (mFemModel[m], frontColor);
//         
//         Color backColor = Color.getHSBColor (h, s, v);
//         RenderProps.setBackColor (mFemModel[m], backColor);
//         
//         h += 0.15;
//      }
      
      // Dome transparent
      
      int m = M-1; 
      
      mFemModel[m].setSurfaceRendering (mRendCfg.mSurfaceRender);
      RenderProps.setFaceColor (mFemModel[m], new Color(0,0,255));
      RenderProps.setBackColor (mFemModel[m], new Color(0,0,255));
      RenderProps.setPointColor (mFemModel[m], mRendCfg.mNodeColor);
      RenderProps.setShininess (
         mFemModel[m], mFemModel[m].getRenderProps().getShininess() * 10);
      RenderProps.setVisible (mFemModel[m], true);
      RenderProps.setFaceStyle (mFemModel[m], Renderer.FaceStyle.FRONT_AND_BACK);
      RenderProps.setPointStyle (mFemModel[m].getNodes(), 
                                 Renderer.PointStyle.SPHERE);
      RenderProps.setPointRadius (mFemModel[m].getNodes(), mRendCfg.mNodeRadius);
      mFemModel[m].setDirectorRenderLen (mRendCfg.mDirectorLen);
      mFemModel[m].getRenderProps ().setLineWidth (mRendCfg.mLineWidth);
      getMainViewer ().setBackgroundColor (mRendCfg.mBackgroundColor);
      RenderProps.setShading (mFemModel[m], Shading.NONE);
      RenderProps.setAlpha (mFemModel[m], 0);
   }
   
   
   protected void build_post() {
      super.build_post ();
     
      // Configure nodes
      for (int m=0; m<M; m++) {
         // TODO NEW
         mFemModel[m].setAbortOnInvertedElements (true);
         
         for (int v=0; v<mMesh[m].numVertices (); v++) {
            Vertex3d vtx = mMesh[m].getVertex (v);
            GrowNode3d gNode = ((GrowNode3d)mFemModel[m].getNode (v));
            
            if (vtx.checkFlag (mFrozenFlag)) {
               gNode.setDynamic (false);
            }
            
            if (vtx.checkFlag (mMphSrcFlag)) {
               gNode.mIsMorphogenSrc = true;
            }
            
            if (vtx.checkFlag (mNoMphZoneFlag)) {
               gNode.mIsNoMorphogenZone = true;
            }
         }
      }

      // Freeze dome
      if (isDome) {
         for (FemNode3d node : mFemModel[M-1].getNodes ()) {
            node.setDynamic (false);
         }
         mFemModel[M-1].setDynamicsEnabled (false);
      }
      
      mPolDir.set(0,1,0);
      
      mRenderMode = RenderMode.MORPHOLOGY;
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
      
      if (t0 < 0.01)
         Main.getMain ().getViewerManager ().setBackgroundColor (mRendCfg.mBackgroundColor);
      
      super.advanceCustom (t0, t1, flags);
   }
   
   protected void customPostDiffusion() {
      for (int m=0; m<M; m++) {
         for (int v = 0; v < mMesh[m].numVertices (); v++) {
            GrowNode3d gNode = (GrowNode3d)mFemModel[m].getNode (v);
            if (gNode.mIsNoMorphogenZone) {
               gNode.mChems.set (3, 0.0);
            }
         }
      }
   }
   
   protected void toggleCollisionBehavior(boolean isEnable) {
      super.toggleCollisionBehavior (isEnable);
      
      if (mDomeCollisionOnly) {
         // Only allow leaf-dome collision
         int c = 0;
         for (int m0=0; m0<M; m0++) {
            for (int m1=0; m1<M; m1++) {
               boolean isLeafDomePair = (m0 == M-1 || m1 == M-1) && m0 != m1;
               mCsnBeh[c].setEnabled (isLeafDomePair);
               c++;
            }
         }
      }
   }
   

   
}
