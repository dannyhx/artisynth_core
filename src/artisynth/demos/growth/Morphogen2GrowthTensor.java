package artisynth.demos.growth;

import artisynth.core.femmodels.FemElement.ElementClass;
import artisynth.core.femmodels.FemElement3dBase;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.ShellElement3d;
import artisynth.demos.growth.models.ts.EdgeDataMap.EdgeData;
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
  
   // Assume any absorbed morphogen is for plastic bending.
   public boolean isBendingMorphogenHack = false;

   // Bending strain to apply to all the elements, regardless of their
   // chemical concentration. Only applicable when isBendingMorphogenHack is 
   // true.
   public Matrix3d fixedBendingStrain = null;
   public boolean zeroStrainAtBottom = false;
   
   protected FemModel3d mFemModel;
   protected PolygonalMesh mMesh; 
   
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
      for (FemElement3dBase ele : mFemModel.getElements ()) {
         GrowElementBase gEle = (GrowElementBase) ele;
         gEle.setPolDir (polDir);
      }
      
      for (ShellElement3d ele : mFemModel.getShellElements ()) {
         GrowElementBase gEle = (GrowElementBase) ele;
         gEle.setPolDir (polDir);
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
      for (FemElement3dBase ele : ShellUtil.getAllElements(mFemModel)) {
         GrowElementBase gEle = (GrowElementBase)ele;
         
         Matrix3d curFrame = gEle.getFrame ();
         
         // First, generate the principle axes of this element.
         //     j0 == parallel to normal
         //     j1 == parallel to polDir
         //     j2 == perpendicular to j0 and j1 (i.e. cross(j0,j1) )
         
         Vector3d nrm = ShellUtil.getNormal (ele, true);
         
         // Project POL onto plane of element
         // Vector3d par = MathUtil.projVec3ToPlane (gEle.getPolDir (), nrm);
         // if (par.normSquared () < MathUtil.ELIPSON) {
         //    par = MathUtil.projVec3ToPlane (nrm, gEle.getPolDirAlt());
         // }
         // par.normalize ();
        Vector3d par = gEle.getPolDir ();

         Vector3d per = new Vector3d().cross (nrm, par).normalize ();
         
         // Now, aggregate principle axes as 3x3 orthogonal matrix where 
         //    col0 == polDir 
         //    col1 == perpendicular 
         //    col2 == normal 
         
         Matrix3d frame = new Matrix3d();
         frame.setColumn (0, par);
         frame.setColumn (1, per);
         frame.setColumn (2, nrm);

         gEle.setFrame(frame);
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
      for (FemElement3dBase ele : ShellUtil.getAllElements(mFemModel)) {
         GrowElementBase gEle = (GrowElementBase)ele;
         
         GrowNode3d[] nodes = gEle.getNodes ();
         
         int numNodes = nodes.length;
         gEle.setElementGrowthTensor (new MatrixNd(numNodes, GrowChemical.NUM_TYPES));
         
         double[] parCol = new double[numNodes];
         double[] perCol = new double[numNodes];
         double[] norCol = new double[numNodes];
         for (int i = 0; i < numNodes; i++) {
            parCol[i] = nodes[i].getGrowChem (GrowChemical.PAR);
            perCol[i] = nodes[i].getGrowChem (GrowChemical.PER);
            norCol[i] = nodes[i].getGrowChem (GrowChemical.NOR);
         }
         gEle.getElementGrowthTensor ().setColumn (0, parCol);
         gEle.getElementGrowthTensor ().setColumn (1, perCol);
         gEle.getElementGrowthTensor ().setColumn (2, norCol);
      }
   }
   
   
   /**
    * For each element, rotate its growth tensor by its frame matrix.
    * 
    * Each rotated growth tensor is stored as a series of symmetric 6-vec
    * strain: each node has its own 6-vec strain.
    */
   protected void rotateElementGrowthTensors() {
      for (FemElement3dBase ele : ShellUtil.getAllElements(mFemModel)) {
         GrowElementBase gEle = (GrowElementBase)ele;
         
         int numNodes = ele.numNodes ();
         
         // Create a Nx6 matrix where each row (6-vector) corresponds to a 
         // 3x3 symmetrical strain of a node.
         // This Nx6 matrix is created by simply appending a zeroed 3x3 matrix
         // to the right of the the local growth tensor.
         // Local growth tensor is NxNumPrincipleAxes.
         MatrixNd strainVects = new MatrixNd(
            numNodes, GrowthTensorUtil.numStrainComp());
         strainVects.addSubMatrix (0, 0, gEle.getElementGrowthTensor ());
         
         for (int n = 0; n < numNodes; n++) {
            // Convert strain representation from 6-vector to 3x3 sym matrix
            
            // [ xx, yy, zz, yz, zx, xy ]
            //
            // into
            //
            // xx xy
            //    yy yz
            // zx    zz 
            
            double[] S = new double[GrowthTensorUtil.numStrainComp()];
            strainVects.getRow (n,S); 
            
            Matrix3d strain = GrowthTensorUtil.vecToMtx3d(S);
            
            // Rotate strain to be aligned with element's frame of principle 
            // directions.
            
            Matrix3d globalStrain = new Matrix3d();
            
            Matrix3d frame = gEle.getFrame();
            
            // R M R' is standard formula for rotating matrix to a frame
            globalStrain.set(frame);
            globalStrain.mul (strain);
            globalStrain.mulTranspose (frame);      

            // Now convert global strain representation from 3x3 sym matrix
            // to 6-vector.
            
            strainVects.setRow (n, GrowthTensorUtil.mtx3dToVec(globalStrain));
         }
         gEle.setRotatedElementGrowthStrains ( strainVects );
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
      for (FemElement3dBase ele : ShellUtil.getAllElements(mFemModel)) {
         GrowElementBase gEle = (GrowElementBase)ele;
         
         // eps0
         MatrixNd strainCols = new MatrixNd(gEle.getRotatedElementGrowthStrains ());
         strainCols.transpose ();
         
         // The value of shape function at every vertex index and 
         // integration coordinate. N-by-Q matrix.
         MatrixNd shapeMtx = getIntegExtrapolationMatrix();
         
         gEle.setStrainAtIntegPts ( new MatrixNd() );
         
         // (StrainLength by N) * (N by Q)
         gEle.getStrainAtIntegPts ().mul (strainCols, shapeMtx);
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
      for (FemElement3dBase ele : ShellUtil.getAllElements(mFemModel)) {
         GrowElementBase gEle = (GrowElementBase)ele;
         
         if (mFemModel.myThinShellAux != null) {
            mFemModel.myThinShellAux.applyGrowthTensorToEle (
               ele, isBendingMorphogenHack, fixedBendingStrain);
            continue; 
         }
         
         // Matrix3d prevF = gEle.getFramePrev ();
         // Matrix3d prevFT = new Matrix3d(prevF); 
         // prevFT.transpose ();
         
         // Matrix3d invPrevF = new Matrix3d(prevF);
         // boolean isOk = invPrevF.invert ();
         // if (!isOk) {
         //    throw new RuntimeException("Failed to invert");
         // }
         
         // Matrix3d invPrevFT = new Matrix3d(prevFT);
         // isOk = invPrevFT.invert ();
         // if (!isOk) {
         //    throw new RuntimeException("Failed to invert");
         // }
         
         // Matrix3d F = new Matrix3d(gEle.getFrame ());
         // Matrix3d FT = new Matrix3d(F);
         // FT.transpose();
         
         GrowIntegrationData3d[] idata = gEle.getIntegrationData ();
         for (int k = 0; k < idata.length; k++) {
            // Convert stored strain 6-vector into 3x3 sym strain matrix.
    
            double[] strainVect = new double[GrowthTensorUtil.numStrainComp()];
            gEle.getStrainAtIntegPts ().getColumn (k, strainVect);
            
            Matrix3d strainMtx = 
               (this.fixedBendingStrain == null) ?
               GrowthTensorUtil.vecToMtx3d(strainVect) : 
               new Matrix3d(this.fixedBendingStrain);

            if (ele.getElementClass () == ElementClass.SHELL) {
               if (isBendingMorphogenHack && k < 3) {
                  // Apply strain as usual to back-surface.
               } 
               else if (isBendingMorphogenHack && k < 6) {
                  // Do not apply strain to mid-surface.
                  strainMtx.setZero ();
               } 
               else if (isBendingMorphogenHack) {
                  // Leave the top-surface untouched.
                  if (zeroStrainAtBottom)
                     strainMtx.setZero ();
                  else
                     strainMtx.negate ();   // Don't use.
               }
            } else { // Element case.
               if (isBendingMorphogenHack && k < 3) {
                  // Apply strain as usual to bottom-surface.
               } 
               else if (isBendingMorphogenHack && k < 6) {
               // Leave the top-surface untouched.
                  if (zeroStrainAtBottom)
                     strainMtx.setZero ();  
                  else
                     strainMtx.negate ();    // Don't use.
               } 
            }
            
            if (this.fixedBendingStrain == null ) {
               // Before incrementing Fp, take the existing Fp, undo its rotation
               // and apply current rotation.
//               Matrix3d curFp = idata[k].getFp ();
//               curFp.sub (Matrix3d.IDENTITY);
//               curFp.mul (invPrevFT);
//               curFp.mul (invPrevF, curFp); 
//               // Undo done. Apply current.
//               curFp.mul (F, curFp);
//               curFp.mul (FT);
//               curFp.add (Matrix3d.IDENTITY);
               
               idata[k].addFp (strainMtx);
            } else {
               strainMtx.add (Matrix3d.IDENTITY);
               idata[k].setFp (strainMtx);
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
      if (mFemModel.myThinShellAux != null) {
         mFemModel.myThinShellAux.unapplyGrowthTensors ();
      } else {
         for (FemElement3dBase ele : ShellUtil.getAllElements(mFemModel)) {
            GrowElementBase gEle = (GrowElementBase)ele;
            
            GrowIntegrationData3d[] idata = gEle.getIntegrationData();
            for (int k = 0; k < idata.length; k++) {
               idata[k].setFp (new Matrix3d());
               idata[k].getFp ().setIdentity ();
            }
         }
      }
      
      ShellUtil.invalidateFem (mFemModel);
   }
   
   
   /* --- Util --- */
   
   protected MatrixNd mIntegExtrapolationMatrixCache;
   protected MatrixNd getIntegExtrapolationMatrix() {
      if (mIntegExtrapolationMatrixCache == null ) {
         
         if (mFemModel.numShellElements () > 0) {
            mIntegExtrapolationMatrixCache =
               mFemModel.getShellElement (0).getShapeMatrix ();
         } else {
            mIntegExtrapolationMatrixCache =
               mFemModel.getElement (0).getShapeMatrix ();
         }
      }
      
      return mIntegExtrapolationMatrixCache;
   }
}
