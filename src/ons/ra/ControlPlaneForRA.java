/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package ons.ra;

import java.util.ArrayList;
import ons.EONLightPath;
import ons.LightPath;
import ons.Flow;
import ons.Path;
import ons.PhysicalTopology;
import ons.VirtualTopology;
import ons.WDMLightPath;
import java.util.Map;
import ons.DisasterArea;

/**
 * This is the interface that provides several methods for the
 * RWA Class within the Control Plane.
 * 
 * @author onsteam
 */
public interface ControlPlaneForRA {

    public boolean acceptFlow(long id, LightPath[] lightpaths);

    public boolean blockFlow(long id);

    public boolean rerouteFlow(long id, LightPath[] lightpaths);   
    
    public boolean canAddLightPath(Flow flow, LightPath lightpath);
    
    public Flow getFlow(long id);
    
    public Path getPath(Flow flow);
    
    public int getLightpathFlowCount(long id);

    public Map<Flow, Path> getMappedFlows();

    public PhysicalTopology getPT();
    
    public VirtualTopology getVT();
    
    public WDMLightPath createCandidateWDMLightPath(int src, int dst, int[] links, int[] wavelengths);
    
    public EONLightPath createCandidateEONLightPath(int src, int dst, int[] links, int firstSlot, int lastSlot, int modulation);
   
    public ArrayList<Flow> getInteruptedFlows();  
    
    public ArrayList<Flow> getActiveFlows();
    
    public ArrayList<Flow> getMappedFlowsAsList();
    
    public boolean upgradeFlow(Flow f, LightPath[] lps);
        
    public void degradeFlow(Flow f,int waveUnit);
    
    public void disasterDeparture(DisasterArea area);
    
    public Path getFlowPath(Flow f);
    
    public ArrayList<Flow> getDelayedFlows();
    
    public void setDelayedFlows(ArrayList<Flow> delayedFlows);
    
    public void removeActiveFlow(long id);
    
    //public void delayedFlowArrival(Flow f);
    
    public void delayedFlowDeparture(Flow f);  
    
    public ArrayList<EONLightPath> getDegradationFriends();
    
    public ArrayList<Flow> getDroppedFlows();
    
    public ArrayList<Flow> getRestoredFlows();
    
    public void dropFlow(Flow f);
    
    public void updateData();
    
    public void delayedFlowsTreatment(Flow f);
    public void restoreFlow(Flow f);
    public void delayFlow(Flow f);
    
    
}
