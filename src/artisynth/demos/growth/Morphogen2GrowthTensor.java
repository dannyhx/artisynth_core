package artisynth.demos.growth;

import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.ShellElement3d;
import artisynth.demos.growth.thinshell.EdgeDataMap.EdgeData;
import artisynth.demos.growth.util.MathUtil;
import artisynth.demos.growth.util.ShellUtil;
import maspack.geometry.Face;
import maspack.geometry.HalfEdge;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Matrix3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;

/** 
 * Handle converting a portion of morphogen into a growth tensor, which is
 * stored/added to the integration points.
 * 
 * TODO:
 * Assumes a single morphogen type that diffuses (TYPE-4). The absorbed
 * TYPE-4 morphogen is simply converted into PAR (TYPE-1) and PER (TYPE-2).
 * 
 * TODO:
 * Need to refactor to handle bending-subtype morphogen. Currently, 
 * bending morphogen is simply hacked on by toggling `isBendingMorphogenHack`.
 * When set to true, all absorbed morphogen is treated as plastic bending.
 */
public class Morphogen2GrowthTensor {
   
   /** Set to 2 if growth tensor is symmetrical. Otherwise, set to 1. */
   protected int SYM_COUNT = 2;
  
   /** Number of unique components in the growth tensor. */
   protected int numStrainComp() {
      return (SYM_COUNT == 2) ? 6 : 9;
   }
  
   // Assume any absorbed morphogen is for plastic bending.
   public boolean isBendingMorphogenHack = false;
   
   protected FemModel3d mFemModel;
   protected PolygonalMesh mMesh; 
   
   public Morphogen2GrowthTensor(FemModel3d femModel) {
      setTarget(femModel);
   }
   
   public Morphogen2GrowthTensor(FemModel3d femModel, PolygonalMesh mesh) {
      setTarget(femModel, mesh);
   }
   
   public void setTarget(FemModel3d femModel) {
      mFemModel = femModel;
   }
   
   public void setTarget(FemModel3d femModel, PolygonalMesh mesh) {
      mFemModel = femModel; 
      mMesh = mesh;
   }

   
   /* --- Methods for manipulating the morphogens --- */
   
   /** Change the polarity direction of every element. */
   public void updatePolarityDirection(Vector3d polDir) {
      for (ShellElement3d ele : mFemModel.getShellElements ()) {
         GrowTriElement gEle = (GrowTriElement) ele;
         gEle.mPolDir.set (polDir);
      }
   }
   
   /**
    * At each node, convert a fraction of the diffusable 
    * morphogen (Type-4) into morphogen PAR (Type-1) and PER (Type-2), which
    * are used to generate the growth tensor.
    * 
    * @param fraction
    * [0.00 to 1.00] of diffusable morphogen to convert into its activate 
    * form (PAR + PER).
    */
   public void activateFractionOfMorphogen(double fraction, boolean isActivatePAR,
   boolean isActivatePER, boolean isActivateNOR) {
      for (FemNode3d node : mFemModel.getNodes ()) {
         GrowNode3d gNode = (GrowNode3d) node;
         double[] chems = gNode.mChems.getBuffer ();
         
         if (chems[3] <= 0) {
            continue;
         }
         
         double xferChem = chems[3]*fraction;
         
         if (isActivatePAR) chems[GrowChemical.PAR.mIdx] += xferChem;
         if (isActivatePER) chems[GrowChemical.PER.mIdx] += xferChem;
         if (isActivateNOR) chems[GrowChemical.NOR.mIdx] += xferChem;
         
         chems[3] -= xferChem;
         
         if (chems[3] < 0) 
            System.out.println (
               "activeFractionOfMorphogen(): Negative morphogen detected.");
      }
   }
   
   
   /**
    * Reset morphogen PAR and PER to zero.
    */
   public void clearActivatedMorphogen() {
      for (FemNode3d node : mFemModel.getNodes ()) {
         GrowNode3d gNode = (GrowNode3d)node;
         
         gNode.setGrowChemsZero();
      }
   }
   
   
   
   /* --- Methods for computing growth tensors from morphogens --- */
   
   /**
    * For each element, compute its local frame of growth direction.
    * 
    * It's dependent on each element's polarity direction (mPolDir).
    */
   protected void createFrames() {
      for (ShellElement3d ele : mFemModel.getShellElements ()) {
         GrowTriElement gEle = (GrowTriElement) ele;
         
         // First, generate the principle axes of this element.
         //     j0 == parallel to normal
         //     j1 == parallel to polDir
         //     j2 == perpendicular to j0 and j1 (i.e. cross(j0,j1) )
         
         Vector3d polDir = new Vector3d(gEle.mPolDir);
         
         Vector3d j0 = ShellUtil.getNormal (gEle, false);
         Vector3d j1 = new Vector3d( polDir ).normalize ();
         Vector3d j2 = new Vector3d().cross (j0, j1).normalize ();
         
         // Now, aggregate principle axes as 3x3 orthogonal matrix where 
         //    col0 == polDir 
         //    col1 == perpendicular 
         //    col2 == normal 
         
         Matrix3d frame = new Matrix3d();
         frame.setColumn (0, j1);
         frame.setColumn (1, j2);
         frame.setColumn (2, j0);
         
         gEle.mFrame = frame;
      }
   }
   
   
   /**
    * For each element, compute its growth tensor, which is dependent on
    * the morphogen concentration of its nodes.
    * 
    * Growth tensor: M[i][j]
    * i => PAR, PER, NOR 
    * j => node index.
    */
   protected void createElementGrowthTensors() {
      for (ShellElement3d ele : mFemModel.getShellElements ()) {
         GrowTriElement gEle = (GrowTriElement) ele;
         
         GrowNode3d[] nodes = gEle.getNodes ();
         
         int numNodes = nodes.length;
         gEle.mElementGrowthTensor = new MatrixNd(numNodes, GrowChemical.NUM_TYPES);
         
         double[] parCol = new double[numNodes];
         double[] perCol = new double[numNodes];
         double[] norCol = new double[numNodes];
         for (int i = 0; i < numNodes; i++) {
            parCol[i] = nodes[i].getGrowChem (GrowChemical.PAR);
            perCol[i] = nodes[i].getGrowChem (GrowChemical.PER);
            norCol[i] = nodes[i].getGrowChem (GrowChemical.NOR);
         }
         gEle.mElementGrowthTensor.setColumn (0, parCol);
         gEle.mElementGrowthTensor.setColumn (1, perCol);
         gEle.mElementGrowthTensor.setColumn (2, norCol);
      }
   }
   
   
   /**
    * For each element, rotate its growth tensor by its frame matrix.
    * 
    * Each rotated growth tensor is stored as a series of symmetric 6-vec
    * strain: each node has its own 6-vec strain.
    */
   protected void rotateElementGrowthTensors() {
      for (ShellElement3d ele : mFemModel.getShellElements ()) {
         GrowTriElement gEle = (GrowTriElement) ele;
         
         int numNodes = gEle.numNodes ();
         GrowNode3d[] nodes = gEle.getNodes ();
         
         // Create a Nx6 matrix where each row (6-vector) corresponds to a 
         // 3x3 symmetrical strain of a node.
         // This Nx6 matrix is created by simply appending a zeroed 3x3 matrix
         // to the right of the the local growth tensor.
         // Local growth tensor is NxNumPrincipleAxes.
         MatrixNd strainVects = new MatrixNd(numNodes, numStrainComp());
         strainVects.addSubMatrix (0, 0, gEle.mElementGrowthTensor);
         
         for (int n = 0; n < numNodes; n++) {
            // Convert strain representation from 6-vector to 3x3 sym matrix
            
            // [ xx, yy, zz, yz, zx, xy ]
            //
            // into
            //
            // xx xy
            //    yy yz
            // zx    zz 
            
            double[] S = new double[numStrainComp()];
            strainVects.getRow (n,S); 
            
            Matrix3d strain = vecToMtx3d(S);
            
            // Rotate strain to be aligned with element's frame of principle 
            // directions.
            
            Matrix3d globalStrain = new Matrix3d();
            
            Matrix3d frame = gEle.mFrame;
            
            // R M R' is standard formula for rotating matrix to a frame
            globalStrain.set(frame);
            globalStrain.mul (strain);
            globalStrain.mulTranspose (frame);      

            // Now convert global strain representation from 3x3 sym matrix
            // to 6-vector.
            
            strainVects.setRow (n, mtx3dToVec(globalStrain));
         }
         gEle.mRotatedElementGrowthStrains = strainVects;
      }
   }
   
   
   /**
    * For each element, interpolate its rotated growth tensor to the 
    * each integration point.
    * 
    * This is only required for solid-shells, or more generally, 
    * non-reduced elements.
    */
   protected void interpolateElementGrowthTensors() {
      for (ShellElement3d ele : mFemModel.getShellElements ()) {
         GrowTriElement gEle = (GrowTriElement) ele;
         
         // eps0
         MatrixNd strainCols = new MatrixNd(gEle.mRotatedElementGrowthStrains);
         strainCols.transpose ();
         
         // The value of shape function at every vertex index and 
         // integration coordinate. N-by-Q matrix.
         MatrixNd shapeMtx = getIntegExtrapolationMatrix();
         
         gEle.mStrainAtIntegPts = new MatrixNd();
         
         // (StrainLength by N) * (N by Q)
         gEle.mStrainAtIntegPts.mul (strainCols, shapeMtx);
      }
   }
   
   /** Convenient method to calculate integration growth tensors. */
   public void computeGrowthTensors() {
      createFrames();
      createElementGrowthTensors();
      rotateElementGrowthTensors();
      
      if (mFemModel.myThinShellAux == null) {
         interpolateElementGrowthTensors();
      }
   }
   
   
   
   /* --- Methods for applying and unapplying computed growth tensors --- */
   
   /**
    * For each element, inject its integration growth tensor into its 
    * plastic strain (or plastic deformation gradient more specifically)
    * attribute.
    */
   public void applyGrowthTensors() {
      for (ShellElement3d ele : mFemModel.getShellElements ()) {
         GrowTriElement gEle = (GrowTriElement) ele;
         
         if (mFemModel.myThinShellAux == null) {
            GrowIntegrationData3d[] idata = gEle.getIntegrationData ();
            for (int k = 0; k < idata.length; k++) {
               // Convert stored strain 6-vector into 3x3 sym strain matrix.
       
               double[] strainVect = new double[numStrainComp()];
               gEle.mStrainAtIntegPts.getColumn (k, strainVect);
               
               Matrix3d strainMtx = vecToMtx3d(strainVect);
               
               strainMtx.set (new double[][] {
                  new double[] {0,0,0}, 
                  new double[] {0,0.25,0},
                  new double[] {0,0,0}
               });
               
               // Custom
               if (isBendingMorphogenHack && k < 3) {
                  // Apply strain as usual to top-surface.
               } 
               else if (isBendingMorphogenHack && k < 6) {
                  // Do not apply strain to mid-surface.
                  strainMtx.setZero ();
               } 
               else if (isBendingMorphogenHack) {
                  // Negate strain to bottom-surface to simulate bending.
                  strainMtx.negate ();
               }
               
//               idata[k].addFp (strainMtx);
               strainMtx.add (Matrix3d.IDENTITY);
               idata[k].setFp (strainMtx);
            }
         } else {
            if (gEle.getPlasticDeformation () == null) {
               gEle.setPlasticDeformation (Matrix3d.IDENTITY);
            }
            
            // Average the strain across the nodes, and consider that as the
            // element's strain.
            
            Matrix3d avgStrain = new Matrix3d(); 
            double[] S = new double[numStrainComp()];
            for (int n = 0; n < gEle.numNodes (); n++) {
               gEle.mRotatedElementGrowthStrains.getRow (n,S); 
               Matrix3d nodeStrain = vecToMtx3d (S);
               avgStrain.add(nodeStrain); 
            }
            avgStrain.scale(1.0 / gEle.numNodes ());
            
            avgStrain.set (new double[][] {
               new double[] {0,0,0}, 
               new double[] {0,-5*Math.PI,0},
               new double[] {0,0,0}
            });
            
            if (isBendingMorphogenHack) {
               int f = ShellUtil.getIndex (gEle);
               Face face = mMesh.getFace (f);
             
               Vector3d edgeStrains = 
                  mFemModel.myThinShellAux.bendStrain_faceToEdges (face, avgStrain);
               
               // For each of the 3 adjacent faces of the element.
               for (int e = 0; e < 3; e++) {
                  HalfEdge edge = face.getEdge (e);
                  
                  if (edge.opposite == null) {
                     continue;
                  }
                  
                  EdgeData edgeData = mFemModel.myEdgeDataMap.get (edge);
                  edgeData.mAngStrain = edgeStrains.get (e);
               }
               
            } else {
               gEle.getPlasticDeformation().add(avgStrain);
            }
         }
      }
      
      ShellUtil.invalidateFem (mFemModel);
   }
   
   /**
    * For each element, reset its plastic deformation gradient attribute back to 
    * the identity matrix; any applied growth tensor is dropped.
    */
   public void unapplyGrowthTensors() {
      for (ShellElement3d ele : mFemModel.getShellElements ()) {
         GrowTriElement gEle = (GrowTriElement) ele;
         
         GrowIntegrationData3d[] idata = gEle.getIntegrationData();
         for (int k = 0; k < idata.length; k++) {
            idata[k].setFp (new Matrix3d());
            idata[k].getFp ().setIdentity ();
         }
      }
      
      ShellUtil.invalidateFem (mFemModel);
   }
   
   
   
   
   /* --- Util --- */
   
   protected Matrix3d vecToMtx3d(double[] V) {
      if (SYM_COUNT == 2) {
         return new Matrix3d(
            V[0],           V[5]/SYM_COUNT, V[4]/SYM_COUNT,
            V[5]/SYM_COUNT, V[1],           V[3]/SYM_COUNT,
            V[4]/SYM_COUNT, V[3]/SYM_COUNT, V[2]); 
      }
      
      return new Matrix3d(
         V[0], V[5], V[4],
         V[8], V[1], V[3],
         V[7], V[6], V[2]); 
   }
   
   protected double[] mtx3dToVec(Matrix3d mtx) {
      if (SYM_COUNT == 2) {
         return new double[] {
             mtx.m00, mtx.m11, mtx.m22, 
             mtx.m12*SYM_COUNT, 
             mtx.m20*SYM_COUNT, 
             mtx.m01*SYM_COUNT};
      }
      
      return new double[] {
          mtx.m00, mtx.m11, mtx.m22,
          mtx.m12, mtx.m02, mtx.m01, 
          mtx.m21, mtx.m20, mtx.m10
      };
   }
   
   protected MatrixNd mIntegExtrapolationMatrixCache;
   protected MatrixNd getIntegExtrapolationMatrix() {
      if (mIntegExtrapolationMatrixCache == null ) {
         mIntegExtrapolationMatrixCache =
            mFemModel.getShellElement (0).getShapeMatrix ();
      }
      
      return mIntegExtrapolationMatrixCache;
   }
   
   /** 
    * From 0 to 1, how parallel are the two vectors?
    */
   protected static double parallelFrac(Vector3d A, Vector3d B) {
      // -PI to PI
      double ang = MathUtil.vectorPairAngle (A, B);
      
      // 0 to PI
      // 0    -> PI/2 (parallel to perpendicular).
      // PI/2 -> PI   (perpendicular to parallel).
      ang = Math.abs(ang);
      
      // 0 -> PI/2 (perpendicular to parallel)
      if (ang < Math.PI/2) {
         ang = Math.PI/2 - ang; 
      } else {
         ang = ang - Math.PI/2;
      }
      
      // 0 to 1 scale.
      ang = ang / (Math.PI/2); 
      
      return ang; 
   }
   


  
}
