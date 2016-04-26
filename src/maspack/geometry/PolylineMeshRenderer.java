/**
 * Copyright (c) 2014, by the Authors: John E Lloyd (UBC), Antonio Sanchez (UBC)
 *
 * This software is freely available under a 2-clause BSD license. Please see
 * the LICENSE file in the ArtiSynth distribution directory for details.
 */
package maspack.geometry;

import java.util.ArrayList;

import maspack.render.RenderObject;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.VertexIndexArray;
import maspack.render.Renderer.ColorInterpolation;
import maspack.render.Renderer.LineStyle;
import maspack.render.Renderer.Shading;

/**
 * Utility class for rendering {@link PolylineMesh} objects.
 */
public class PolylineMeshRenderer  extends MeshRendererBase {

   private static PolylineMeshRenderer instance = null;
   
   public static PolylineMeshRenderer getInstance() {
      if (instance == null) {
         instance = new PolylineMeshRenderer ();
      }
      return instance;
   }
   
   protected class PolylineRobSignature extends RobSignature {
      
      // Don't need anything extra at the moment ...

      public PolylineRobSignature (
         PolylineMesh mesh, RenderProps props) {
         super (mesh, props);
      }

      public boolean equals (RobSignature other) {
         if (other instanceof PolylineRobSignature) {
            PolylineRobSignature pother = (PolylineRobSignature)other;
            return (super.equals (pother));
         }
         else {
            return false;
         }
      }
   }

   public PolylineMeshRenderer() {
   }

   protected RobSignature createSignature (
      MeshBase mesh, RenderProps props) {
      return new PolylineRobSignature ((PolylineMesh)mesh, props);
   }

   @Override
   protected RenderObject buildRenderObject (MeshBase mesh, RenderProps props) {
      RenderObject r = super.buildRenderObject (mesh, props);
      PolylineMesh pmesh = (PolylineMesh)mesh;

      int[] nidxs = pmesh.hasNormals() ? pmesh.getNormalIndices() : null;
      int[] cidxs = pmesh.hasColors() ? pmesh.getColorIndices() : null;
      int[] tidxs = pmesh.hasTextureCoords () ? pmesh.getTextureIndices () : null;

      int[] indexOffs = mesh.getFeatureIndexOffsets();
      int[] pidxs = mesh.createVertexIndices();
      ArrayList<Polyline> lines = pmesh.getLines();
      for (int i=0; i<pmesh.numLines(); i++) {
         Polyline line = lines.get(i); // XXX is this needed? or computed from index offsets?
         
         int loff = indexOffs[i];
         int numv = indexOffs[i+1] - loff;

         int[] vidxs = new int[numv]; 
         for (int j=0; j<numv; j++) {
            vidxs[j] = r.addVertex(
               pidxs[loff + j],
               nidxs != null ? nidxs[loff + j] : i,
               cidxs != null ? cidxs[loff + j] : -1,
               tidxs != null ? tidxs[loff + j] : -1);
         }
         // triangle fan for faces, line loop for edges
         r.addLineStrip(vidxs);
      }
      
      return r;
   }

   @Override
   protected void updateRenderObject (MeshBase mesh, RenderProps props, RenderObject robj) {
      super.updateRenderObject (mesh, props, robj);
   }

   public MeshRenderInfo prerender (PointMesh mesh, RenderProps props, MeshRenderInfo renderInfo) {
      return super.prerender (mesh, props, renderInfo);
   }

   public void render (
      Renderer renderer, PolylineMesh mesh, RenderProps props, 
      int flags, MeshRenderInfo renderInfo) {

      if (mesh.numVertices() == 0) {
         return;
      }
      boolean highlight = ((flags & Renderer.HIGHLIGHT) != 0);

      renderer.pushModelMatrix();
      if (mesh.isRenderBuffered()) {
         renderer.mulModelMatrix (mesh.getXMeshToWorldRender());
      }
      else {
         renderer.mulModelMatrix (mesh.XMeshToWorld);
      }

      float savedLineWidth = renderer.getLineWidth();
      Shading savedShadeModel = renderer.getShading();

//      Shading shading = props.getShading();
//      if (renderer.isSelecting()) {
//         shading = Shading.NONE;
//      }

      LineStyle lineStyle = props.getLineStyle();
      Shading savedShading = renderer.getShading();
      if (lineStyle == LineStyle.LINE && !mesh.hasNormals()) {
         renderer.setShading (Shading.NONE);
      }
      else {
         renderer.setShading (props.getShading());
      }
      ColorInterpolation savedColorInterp = null;
      if (usingHSV(mesh)) {
         savedColorInterp =
            renderer.setColorInterpolation (ColorInterpolation.HSV);
      }
      renderer.setLineColoring (props, highlight);
      switch (lineStyle) {
         case LINE: {
            int width = props.getLineWidth();
            if (width > 0) {
               renderer.drawLines (renderInfo.getRenderObject (), LineStyle.LINE, width);
            }
            break;
         }
         case SPINDLE:
         case SOLID_ARROW:
         case CYLINDER: {
            double rad = props.getLineRadius();
            if (rad > 0) {
               renderer.drawLines (renderInfo.getRenderObject (), props.getLineStyle(), rad);
            }
            break;
         }
      }
      if (savedColorInterp != null) {
         renderer.setColorInterpolation (savedColorInterp);
      }
      renderer.setShading (savedShading);
      renderer.setLineWidth (savedLineWidth);
      renderer.setShading (savedShadeModel);

      renderer.popModelMatrix();
   }
   
   /**
    * Appends Polyline line segments to an index array associated with the 
    * provided rendering context.  This is most useful for rendering
    * a selection of polylines from a PolylineMesh.
    * @param lines list of lines of which to obtain appropriate rendering indices
    * @param rinfo rendering information to be used along with the polylines
    * @param out list of indices to populate
    * @return number of indices added to <code>out</code>
    */
   public static int getPolylineLines(Iterable<? extends Polyline> lines,
      MeshRenderInfo renderInfo, VertexIndexArray out) {
      
      MeshBase mesh = renderInfo.getMesh ();
      int[] indexOffs = mesh.getFeatureIndexOffsets();
      
      int ni = out.size ();
      for (Polyline line : lines) {
         int polyIdx = line.getIndex ();
         int vidx = indexOffs[polyIdx];
         int numv = line.numVertices ();
         
         out.ensureCapacity (out.size ()+2*numv);
         for (int i=0; i<numv-1; ++i) {
            out.add (vidx);
            out.add (++vidx);
         }
      }
      
      return out.size ()-ni;
   
   }

}
