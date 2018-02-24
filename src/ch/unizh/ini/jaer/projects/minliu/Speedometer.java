/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ch.unizh.ini.jaer.projects.minliu;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import com.jogamp.opengl.util.awt.TextRenderer;
import java.awt.Font;
import java.awt.Point;
import java.awt.event.MouseEvent;
import java.awt.geom.Rectangle2D;
import java.io.File;
import javax.swing.JFileChooser;
import net.sf.jaer.Description;
import net.sf.jaer.DevelopmentStatus;
import net.sf.jaer.chip.AEChip;
import net.sf.jaer.event.EventPacket;
import static net.sf.jaer.eventprocessing.EventFilter.log;
import net.sf.jaer.eventprocessing.EventFilter2DMouseAdaptor;
import net.sf.jaer.graphics.FrameAnnotater;
import net.sf.jaer.util.EngineeringFormat;
import net.sf.jaer.util.TobiLogger;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import net.sf.jaer.event.BasicEvent;
import net.sf.jaer.event.orientation.MotionOrientationEventInterface;


/**
 * A simple utility to do hand measurements of velocity using mouse
 *
 * @author Tobi Delbruck
 */
@Description("A simple utility to do hand measurements of velocity using mouse")
@DevelopmentStatus(DevelopmentStatus.Status.Stable)
public class Speedometer extends EventFilter2DMouseAdaptor implements FrameAnnotater {

    private int currentTimestamp = 0;
    private TimePoint startPoint, endPoint;
    private boolean firstPoint = true;
    private float speed = 0, distance, deltaTimestamp;
    EngineeringFormat engFmt = new EngineeringFormat();
    TextRenderer textRenderer = null;
    protected TobiLogger motionVectorEventLogger = null;
    public Speedometer(AEChip chip) {
        super(chip);
    }

    @Override
    public EventPacket<?> filterPacket(EventPacket<?> in) {
        if (in.getSize() > 2) {
            currentTimestamp = (in.getLastTimestamp() + in.getFirstTimestamp()) / 2;
        }
        return in;
    }

    @Override
    public void resetFilter() {
        firstPoint = true;
    }

    @Override
    public void initFilter() {
    }

    @Override
    public void mouseClicked(MouseEvent e) {
        if (firstPoint) {
            startPoint = new TimePoint(getMousePixel(e), currentTimestamp);
            endPoint = null;
        } else {
            endPoint = new TimePoint(getMousePixel(e), currentTimestamp);
        }
        if (startPoint != null && endPoint != null) {
            distance = (float) (endPoint.distance(startPoint));
            deltaTimestamp = endPoint.t - startPoint.t;
            speed = 1e6f * distance / deltaTimestamp;
            log.info(String.format("%s pps", engFmt.format(speed)));
        }
        firstPoint = !firstPoint;
    }

    private class TimePoint extends Point {

        int t;

        public TimePoint(int t, int xx, int yy) {
            super(xx, yy);
            this.t = t;
        }

        public TimePoint(Point p, int t) {
            super(p.x, p.y);
            this.t = t;
        }

    }

    @Override
    public void annotate(GLAutoDrawable drawable) {
        super.annotate(drawable); //To change body of generated methods, choose Tools | Templates.
        GL2 gl = drawable.getGL().getGL2();
        drawCursor(gl, startPoint, new float[]{0, 1, 0, 1});
        drawCursor(gl, endPoint, new float[]{1, 0, 0, 1});
        if (startPoint != null && endPoint != null) {
            gl.glColor3f(1, 1, 0);
            gl.glBegin(GL.GL_LINES);
            gl.glVertex2f(startPoint.x, startPoint.y);
            gl.glVertex2f(endPoint.x, endPoint.y);
            gl.glEnd();

            if (textRenderer == null) {
                textRenderer = new TextRenderer(new Font("SansSerif", Font.PLAIN, 24), true, false);
            }
            textRenderer.setColor(1, 1, 0, 1);
            String s = String.format("%s pps (%.0fpix /%ss)", engFmt.format(speed), distance, engFmt.format(1e-6f * deltaTimestamp));
            String l = String.format("%s %s %s %s", currentTimestamp, engFmt.format(speed), distance, engFmt.format(1e-6f * deltaTimestamp));
            
            
            textRenderer.beginRendering(drawable.getSurfaceWidth(), drawable.getSurfaceHeight());
//            Rectangle2D r = textRenderer.getBounds(s);
            textRenderer.draw(s, (startPoint.x+endPoint.x)/2,(startPoint.y+endPoint.y)/2 );
            textRenderer.endRendering();
            if (motionVectorEventLogger != null) motionVectorEventLogger.log(l);
        }
    }

    private void drawCursor(GL2 gl, Point p, float[] color) {
        if (p == null) {
            return;
        }
        checkBlend(gl);
        gl.glColor4fv(color, 0);
        gl.glLineWidth(3f);
        gl.glPushMatrix();
        gl.glTranslatef(p.x, p.y, 0);
        gl.glBegin(GL2.GL_LINES);
        gl.glVertex2f(0, -CURSOR_SIZE_CHIP_PIXELS / 2);
        gl.glVertex2f(0, +CURSOR_SIZE_CHIP_PIXELS / 2);
        gl.glVertex2f(-CURSOR_SIZE_CHIP_PIXELS / 2, 0);
        gl.glVertex2f(+CURSOR_SIZE_CHIP_PIXELS / 2, 0);
        gl.glEnd();
        gl.glTranslatef(.5f, -.5f, 0);
        gl.glColor4f(0, 0, 0, 1);
        gl.glBegin(GL2.GL_LINES);
        gl.glVertex2f(0, -CURSOR_SIZE_CHIP_PIXELS / 2);
        gl.glVertex2f(0, +CURSOR_SIZE_CHIP_PIXELS / 2);
        gl.glVertex2f(-CURSOR_SIZE_CHIP_PIXELS / 2, 0);
        gl.glVertex2f(+CURSOR_SIZE_CHIP_PIXELS / 2, 0);
        gl.glEnd();
        gl.glPopMatrix();
    }
    
    synchronized public void doSaveSingleSpeed() {

    
      //String s = String.format("%d %d %d %d %.3g %.3g %.3g %d", eout.timestamp, eout.x, eout.y, eout.type, eout.velocity.x, eout.velocity.y, eout.speed, eout.hasDirection ? 1 : 0);
      String s = String.format("%d %d %d", speed, distance, (1e-6f * deltaTimestamp));
      motionVectorEventLogger.log(s);
      motionVectorEventLogger.log("hellojjj");
      log.info("button pressed");
       

//Add your code here that is exectued
 // upon button press.
}
    
    
    synchronized public void doStartLoggingMotionVectorEvents() {
        if (motionVectorEventLogger != null && motionVectorEventLogger.isEnabled()) {
            log.info("logging already started");
            return;
        }
        String filename = null, filepath = null;
        final JFileChooser fc = new JFileChooser();
        fc.setCurrentDirectory(new File(getString("lastFile", System.getProperty("user.dir"))));  // defaults to startup runtime folder
        fc.setFileSelectionMode(JFileChooser.FILES_ONLY);
        fc.setSelectedFile(new File(getString("lastFile", System.getProperty("user.dir"))));
        fc.setDialogTitle("Select folder and base file name for the logged motion vector event data");
        int ret = fc.showOpenDialog(chip.getAeViewer() != null && chip.getAeViewer().getFilterFrame() != null ? chip.getAeViewer().getFilterFrame() : null);
        if (ret == JFileChooser.APPROVE_OPTION) {
            File file = fc.getSelectedFile();
            putString("lastFile", file.toString());
            motionVectorEventLogger = new TobiLogger(file.getPath(), "Motion vector events output from normal optical flow method");
            motionVectorEventLogger.setNanotimeEnabled(false);
            motionVectorEventLogger.setHeaderLine("speed_pps distance_pix deltatimestamp_ms");
            motionVectorEventLogger.setEnabled(true);
            motionVectorEventLogger.log("it works");
      
        } else {
            log.info("Cancelled logging motion vectors");
        }    
    }
    
        synchronized public void doStopLoggingMotionVectorEvents() {
        if (motionVectorEventLogger == null) {
            return;
        }
        motionVectorEventLogger.setEnabled(false);
        motionVectorEventLogger = null;
    }
    
    
    
}
