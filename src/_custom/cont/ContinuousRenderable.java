package _custom.cont;

import java.awt.Color;
import java.util.ArrayList;

import artisynth.core.femmodels.FemMeshComp;
import maspack.collision.ContactInfo;
import maspack.collision.EdgeEdgeContact;
import maspack.collision.PenetratingPoint;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.render.Renderer;
import maspack.render.color.ColorMap;
import maspack.render.color.RainbowColorMap;

public class ContinuousRenderable {
   
   protected ContactInfo cinfo;
   protected double vecLenScale = 5;
   
   public ContinuousRenderable(ContactInfo cinfo) {
      this.cinfo = cinfo;
   }
   
   public void render(Renderer renderer, int flags) {
      double a = 0;
      ColorMap colorMap = new RainbowColorMap();
      Color curColor = null;
      
      for (EdgeEdgeContact eeCt : cinfo.getEdgeEdgeContacts ()) {
         
//         a += 0.05;
//         curColor = colorMap.getColor (a);
//         
//         renderer.drawArrow (eeCt.edge0.tail.pnt, eeCt.edge0.head.pnt, 0.003, true);
//         renderer.drawArrow (eeCt.edge1.tail.pnt, eeCt.edge1.head.pnt, 0.003, true);
         
         // Repulsion normal
         renderer.setColor (Color.BLUE);
         Vector3d nrmVec = new Vector3d(eeCt.point1ToPoint0Normal);
         nrmVec.scale ( eeCt.displacement*vecLenScale );
         
         Point3d nrmTail = eeCt.point0;
         Point3d nrmHead = (Point3d)
            new Point3d(nrmTail).add (nrmVec);
         
         renderer.drawArrow (nrmTail, nrmHead, 0.001, true);
      }
      
      renderer.setColor (Color.RED);
      for (PenetratingPoint pentPt : cinfo.getPenetratingPoints (0)) {
         Vector3d nrmVec = new Vector3d(pentPt.normal);
         nrmVec.scale ( pentPt.distance*vecLenScale );
         
         Point3d nrmTail = pentPt.position;
         Point3d nrmHead = (Point3d)
            new Point3d(nrmTail).add (nrmVec);
         
         renderer.drawArrow (nrmTail, nrmHead, 0.001, true);
      }
   }
}
