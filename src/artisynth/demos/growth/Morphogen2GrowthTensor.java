package artisynth.demos.growth;

import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.ShellElement3d;
import artisynth.core.femmodels.FemElement.ElementClass;
import artisynth.demos.growth.util.HingeUtil;
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
   public void computeIntegrationGrowthTensors() {
      createFrames();
      
      if (isBendingMorphogenHack && 
      mFemModel.getShellElement (0).getElementClass () == ElementClass.MEMBRANE) {
         createElementBendingGrowthTensors();
      } else {
         createElementGrowthTensors();
      }
      
      rotateElementGrowthTensors();
      interpolateElementGrowthTensors();
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
         
         GrowIntegrationData3d[] idata = gEle.getIntegrationData ();
                                     
         if (gEle.getPlasticDeformation () == null) {
            gEle.setPlasticDeformation (Matrix3d.IDENTITY);
         }
         
         for (int k = 0; k < idata.length; k++) {
            // Convert stored strain 6-vector into 3x3 sym strain matrix.
    
            double[] strainVect = new double[numStrainComp()];
            gEle.mStrainAtIntegPts.getColumn (k, strainVect);
            
            Matrix3d strainMtx = vecToMtx3d(strainVect);
            
            // Custom
            if (isBendingMorphogenHack && k < 3) {
               // ok
            } 
            else if (isBendingMorphogenHack && k < 6) {
               strainMtx.setZero ();
            } 
            else if (isBendingMorphogenHack) {
               strainMtx.negate ();
            }
            
            idata[k].addFp (strainMtx);
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
   
   
   
   /* --- Membrane Bending --- */
   
   protected void createElementBendingGrowthTensors() {
      int eleIdx = 0;
      for (ShellElement3d ele : mFemModel.getShellElements ()) {
         GrowTriElement gEle = (GrowTriElement) ele;
         
         int f = Integer.valueOf (gEle.getName ().substring (gEle.getName ().length () - 1));
         Face face = mMesh.getFace (f);
         
         boolean useRest = true;
         
         Vector3d elePAR = new Vector3d(gEle.mPolDir).normalize ();
         Vector3d eleNOR = ShellUtil.getNormal (gEle, useRest);
         Vector3d elePER = new Vector3d().cross (eleNOR, elePAR).normalize ();

         MatrixNd eleStrain = new MatrixNd(gEle.numNodes (), GrowChemical.NUM_TYPES);
         
         // For each of the 3 adjacent faces of the element.
         for (int e = 0; e < 3; e++) {
            HalfEdge edge = face.getEdge (e);
            
            if (edge.opposite == null) {
               continue;
            }
            
            // 2 nodes of the edge.
            Vertex3d vtxH = edge.getHead ();
            Vertex3d vtxT = edge.getTail ();
            GrowNode3d nodeH = (GrowNode3d)mFemModel.getNode (vtxH.getIndex ()); 
            GrowNode3d nodeT = (GrowNode3d)mFemModel.getNode (vtxT.getIndex ());
            Point3d nodeHPos = ShellUtil.getPosition (nodeH, true, !useRest);
            Point3d nodeTPos = ShellUtil.getPosition (nodeT, true, !useRest);
            
            // Adjacent face.
            int oFace = edge.getOppositeFace ().getIndex (); 
            GrowTriElement oEle = (GrowTriElement) this.mFemModel.getShellElement (oFace);
            Vector3d oEleNOR = ShellUtil.getNormal (oEle, useRest);
            
            // Average normal of the 2 faces.
            Vector3d edgeNormalUnit = new Vector3d(eleNOR).add(oEleNOR).normalize ();
            
            // Edge direction.
            Vector3d edgeVec = new Vector3d(nodeHPos).sub (nodeTPos);
            Vector3d edgeVecUnit = new Vector3d(edgeVec).normalize ();
            
            // Perpendicular of the average normal and edge direction. 
            
            Vector3d t = new Vector3d().cross (edgeNormalUnit, edgeVecUnit);
            t.normalize ();
            
            // Perpendicular should point away from triangle.
            Point3d edgeCenter = (Point3d) new Point3d(nodeTPos).
               scaledAdd (0.5, edgeVec);
//            HingeUtil.ensureExteriorVecT (gEle, edgeCenter, t, useRest);
                                    
            Matrix3d txtt = new Matrix3d(); 
            txtt.outerProduct (t, t);
     
            // Compute scaler of outer product.
            
            // Simulate hinge strain existence.
            // PAR morphogen will only affect edges that are parallel 
            // with mPolDir.
            // PER morphogen will only affect edges that are parallel 
            // with cross-product of mPolDir and element's normal.
            
            double alignPctPAR = Math.abs (t.dot (elePAR));
            double alignPctPER = Math.abs (t.dot (elePER));
            alignPctPER = 0;
            
            double chemPAR_H = nodeH.getGrowChem (GrowChemical.PAR);
            double chemPER_H = nodeH.getGrowChem (GrowChemical.PER); 
            
            double chemPAR_T = nodeT.getGrowChem (GrowChemical.PAR);
            double chemPER_T = nodeT.getGrowChem (GrowChemical.PER); 
            
            // Simulated angle offset.
            double chemAngPAR = alignPctPAR * (chemPAR_H + chemPAR_T);
            double chemAngPER = alignPctPER * (chemPER_H + chemPER_T);
            double chemAng = (chemAngPAR + chemAngPER) / 180;
            
            // Pass angle offset to monotonic function.
            double phi = 2 * Math.tan (chemAng);
            
            System.out.printf ("t vector for edge (%s, %s): %s. AlignPctPAR/PER: %.2f, %.2f\n", 
               gEle.getName (), 
               oEle.getName (),
               t.toString ("%.2f"), 
               alignPctPAR, 
               alignPctPER
            );

            
            Matrix3d edgeStrain = new Matrix3d();
            edgeStrain.scaledAdd (phi, txtt);
            edgeStrain.scale (1 / edgeVec.norm ());
            
            // Sum
            eleStrain.add (edgeStrain);
         }
         
         // Scale the strain
         
         double area = gEle.getArea ();
         eleStrain.scale (0.5 * area); 
         
         // Save.
  
         if (eleIdx == 1) {
//            eleStrain.setZero ();
         } else {

         }

         
         gEle.mElementGrowthTensor = eleStrain; 
           
         eleIdx++;
      }
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
   
   
   /* --- Test --- */
   
   public static void main(String[] args) {
      Vector3d a = new Vector3d(0,0,1).normalize ();
      
      Matrix3d outer = new Matrix3d();
      outer.outerProduct (a, a);
      System.out.println (outer);
   }
  
}
