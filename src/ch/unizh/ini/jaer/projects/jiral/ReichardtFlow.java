/*
 * Copyright (C) 2018 Luis Jira.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301  USA
 */
package ch.unizh.ini.jaer.projects.jiral;

import ch.unizh.ini.jaer.projects.rbodo.opticalflow.AbstractMotionFlow;
import ch.unizh.ini.jaer.projects.rbodo.opticalflow.AbstractMotionFlow;
import java.awt.Point;
import java.util.*;
import net.sf.jaer.Description;
import net.sf.jaer.DevelopmentStatus;
import net.sf.jaer.chip.AEChip;
import net.sf.jaer.event.ApsDvsEvent;
import net.sf.jaer.event.ApsDvsEventPacket;
import net.sf.jaer.event.EventPacket;
import net.sf.jaer.event.PolarityEvent;
import static net.sf.jaer.eventprocessing.EventFilter.log;

/**
 * Supposed to draw individual motion vectors and compute global flow. Algorithm
 * based on Ridwan I., Cheng H. "An Event-Based Optical Flow Algorithm for
 * Dynamic Vision Sensors", Image Analysis and Recognition (2017).
 *
 * Class design based on LucasKanadeFlow by rbodo. 
 *
 * @author jiral
 */
@Description("Class for orientation of local motion optical flow based on the Reichardt detector.")
@DevelopmentStatus(DevelopmentStatus.Status.InDevelopment)
public class ReichardtFlow extends AbstractMotionFlow {

//Array to store most recent Timestamp per pixel
    private int[][][] timeMap;

// Additional spacing to border of chip. Needed for finite difference methods.    
    private int d;

//List with possible direction vectors
    private List<Point> vectors;
    
    public ReichardtFlow(AEChip chip) {
        super(chip);
        
        numInputTypes = 2;
        setup();
        resetFilter();  //unsure why, but this seems to not do anything
    }
 
    @Override
    synchronized public EventPacket filterPacket(EventPacket in) {
        setupFilter(in);
        motionField.checkArrays();

        // following awkward block needed to deal with DVS/DAVIS and IMU/APS events
        // block STARTS
        Iterator i;
        if (in instanceof ApsDvsEventPacket) {
            i = ((ApsDvsEventPacket) in).fullIterator();
        } else {
            i = ((EventPacket) in).inputIterator();
        }

        while (i.hasNext()) {
            Object o = i.next();
            if (o == null) {
                log.warning("null event passed in, returning input packet");
                return in;
            }
            if ((o instanceof ApsDvsEvent) && ((ApsDvsEvent) o).isApsData()) {
                continue;
            }
            PolarityEvent ein = (PolarityEvent) o;

            if (!extractEventInfo(o)) {
                continue;
            }
            if (measureAccuracy || discardOutliersForStatisticalMeasurementEnabled) {
                if (imuFlowEstimator.calculateImuFlow(o)) {
                    continue;
                }
            }
            // block ENDS
            if (isInvalidAddress(searchDistance + d)) {
                continue;
            }
//        timestamps[x][y][type].add(ts); // Add most recent event to queue.
//        timestamps2[x][y][type].add(ts);
            
            timeMap = lastTimesMap; //save last ts
            if (isInvalidTimestamp()) {
                continue;
            }
            if (xyFilter()) {
                continue;
            }
            countIn++;
            
            int xTemp, yTemp, tTemp, pTemp;
            
            for(Point vector : vectors){
              xTemp = x-vector.x;
              yTemp = y-vector.y;
              tTemp = timeMap[xTemp][yTemp][type];
              if(0 < ts - tTemp && ts-tTemp <= maxDtThreshold){
                  vx = vector.x;
                  vy = vector.y;
                processGoodEvent();
              }
            }
            
        }
        getMotionFlowStatistics().updatePacket(countIn, countOut);
        return isDisplayRawInput() ? in : dirPacket;
    }
    
    
    public void setup(){
        vectors = new ArrayList<>();
        //search block size is 2*searchDistance+1
        for(int xP = searchDistance * -1; xP <= searchDistance; xP++){
            for( int yP = searchDistance * -1; yP <= searchDistance; yP++){
                vectors.add(new Point(xP, yP));
            }
        }

    }

}
