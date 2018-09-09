/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ons.ra;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;
import java.util.TreeSet;
import ons.DisasterArea;
import ons.EONLightPath;
import ons.EONLink;
import ons.EONPhysicalTopology;
import ons.Flow;
import ons.LightPath;
import ons.Modulation;
import ons.PhysicalTopology;
import ons.WDMLightPath;
import ons.WDMPhysicalTopology;
import ons.util.Dijkstra;
import ons.util.WeightedGraph;
import ons.util.YenKSP;

/**
 *
 * @author Gab
 */
public class EON_FDM implements RA{

    private ControlPlaneForRA cp;
    private WeightedGraph graph;    
    
    @Override
    public void simulationInterface(ControlPlaneForRA cp) {
        this.cp = cp;
        this.graph = cp.getPT().getWeightedGraph();           
    }
    
    private WeightedGraph getPostDisasterGraph(PhysicalTopology pt){
        
        int nodes = pt.getNumNodes();
        WeightedGraph g = new WeightedGraph(nodes);
        for (int i = 0; i < nodes; i++) {
            for (int j = 0; j < nodes; j++) {
                if (pt.hasLink(i, j)) {
                    if(!pt.getLink(i, j).isIsInterupted()){
                        g.addEdge(i, j, pt.getLink(i, j).getWeight());
                    }else{                        
                        g.addEdge(i, j, Integer.MAX_VALUE);
                    }                        
                }
            }
        }        
        
        return g;
    }

    public static int[] convertIntegers(List<Integer> integers) {
        int[] ret = new int[integers.size()];
        Iterator<Integer> iterator = integers.iterator();
        for (int i = 0; i < ret.length; i++) {
            ret[i] = iterator.next().intValue();
        }
        return ret;
    }
    
    @Override   
    public void flowArrival(Flow flow) {
        int[] nodes;
        int[] links;
        long id;
        LightPath[] lps = new LightPath[1];          
       /* ArrayList<Integer>[] paths = YenKSP.kShortestPaths(graph, flow.getSource(), flow.getDestination(), 3);
        flow.setPaths(paths);   */    
       ArrayList<Integer>[] paths = YenKSP.kDisruptedShortestPaths(cp.getPT().getWeightedGraph(), flow.getSource(), flow.getDestination(), 3);
       flow.setPaths(paths);  
        
        //this.graph = this.getPostDisasterGraph(cp.getPT());
        
        OUTER:
        for (ArrayList<Integer> path : paths) {

            nodes = convertIntegers(path);
            // If no possible path found, block the call
            if (nodes.length == 0) {
                cp.blockFlow(flow.getID());
                return;
            }

            // Create the links vector
            links = new int[nodes.length - 1];
            for (int j = 0; j < nodes.length - 1; j++) {
                links[j] = cp.getPT().getLink(nodes[j], nodes[j + 1]).getID();
            }

            // Get the size of the route in km
            double sizeRoute = 0;
            for (int i = 0; i < links.length; i++) {
                sizeRoute += ((EONLink) cp.getPT().getLink(links[i])).getWeight();
            }
            // Adaptative modulation:
            int modulation = Modulation.getBestModulation(sizeRoute);
            //System.out.println(modulation);
            flow.setModulation(modulation);
            // Calculates the required slots
            int requiredSlots = Modulation.convertRateToSlot(flow.getBwReq(), EONPhysicalTopology.getSlotSize(), modulation);
            if(requiredSlots>=100000)
                    continue OUTER;
            // Evaluate if each link have space to the required slots
            for (int i = 0; i < links.length; i++) {
                if (!((EONLink) cp.getPT().getLink(links[i])).hasSlotsAvaiable(requiredSlots)) {
                    cp.blockFlow(flow.getID());
                    return;
                }
            }

            // First-Fit spectrum assignment in some modulation 
            int[] firstSlot;
            for (int i = 0; i < links.length; i++) {
                // Try the slots available in each link
                firstSlot = ((EONLink) cp.getPT().getLink(links[i])).getSlotsAvailableToArray(requiredSlots);
                for (int j = 0; j < firstSlot.length; j++) {
                    // Now you create the lightpath to use the createLightpath VT
                    //Relative index modulation: BPSK = 0; QPSK = 1; 8QAM = 2; 16QAM = 3;
                    EONLightPath lp = cp.createCandidateEONLightPath(flow.getSource(), flow.getDestination(), links,
                            firstSlot[j], (firstSlot[j] + requiredSlots - 1), modulation);
                    // Now you try to establish the new lightpath, accept the call
                    if ((id = cp.getVT().createLightpath(lp)) >= 0) {
                        // Single-hop routing (end-to-end lightpath)
                        lps[0] = cp.getVT().getLightpath(id);
                        if (cp.acceptFlow(flow.getID(), lps)) {
                            return;
                        } else {
                            // Something wrong
                            // Dealocates the lightpath in VT and try again                        
                            cp.getVT().deallocatedLightpath(id);
                        }
                    }
                }
            }           

        }
        
        // Block the call
        //System.out.println("Block Arrival");
        cp.blockFlow(flow.getID());        
        
    }
    
    private boolean rerouteFlow(Flow flow) {      
                
        ArrayList<Integer> lastPath = null;
        long id;
        OUTER:
        for (ArrayList<Integer> path : flow.getPaths()) {

            lastPath = path;
            if (path != null) {

                int[] links = new int[path.size() - 1];
                LightPath[] lps = new LightPath[1];

                for (int j = 0; j < path.size() - 1; j++) {

                    if (cp.getPT().getLink(path.get(j), path.get(j + 1)).isIsInterupted()) {

                        path = null;
                        lastPath = path;
                        continue OUTER;
                    }

                    links[j] = cp.getPT().getLink(path.get(j), path.get(j + 1)).getID();

                }

                // Get the size of the route in km
                double sizeRoute = 0;
                for (int i = 0; i < links.length; i++) {
                    sizeRoute += ((EONLink) cp.getPT().getLink(links[i])).getWeight();
                }
                // Adaptative modulation:
                int modulation = Modulation.getBestModulation(sizeRoute);

                int requiredSlots = Modulation.convertRateToSlot((int) flow.getMaxRate(), EONPhysicalTopology.getSlotSize(), modulation);

                int[] firstSlot;
                for (int i = 0; i < links.length; i++) {
                    // Try the slots available in each link
                    firstSlot = ((EONLink) cp.getPT().getLink(links[i])).getSlotsAvailableToArray(requiredSlots);
                    for (int j = 0; j < firstSlot.length; j++) {
                        // Now you create the lightpath to use the createLightpath VT
                        //Relative index modulation: BPSK = 0; QPSK = 1; 8QAM = 2; 16QAM = 3;
                        EONLightPath lp = cp.createCandidateEONLightPath(flow.getSource(), flow.getDestination(), links,
                                firstSlot[j], (firstSlot[j] + requiredSlots - 1), modulation);
                        // Now you try to establish the new lightpath, accept the call
                        if ((id = cp.getVT().createLightpath(lp)) >= 0) {
                            // Single-hop routing (end-to-end lightpath)
                            lps[0] = cp.getVT().getLightpath(id);
                            if (cp.upgradeFlow(flow, lps)) {
                                return true;
                            } else {
                                // Something wrong
                                // Dealocates the lightpath in VT and try again
                                cp.getVT().deallocatedLightpath(id);
                            }
                        }
                    }
                }
            }
        }
        return false;
    }
    

    public boolean addLightPath(Flow flow){
        
        ArrayList<Integer> nodes = new ArrayList<Integer>();
        int[] links;
        long id;
        LightPath[] lps = new LightPath[1];  
             
        OUTER:
        for (ArrayList<Integer> path : flow.getPaths()) {
            nodes = path;
            if (nodes.size() == 0) {
                continue;
            }
            // Create the links vector
            links = new int[nodes.size() - 1];
            for (int j = 0; j < nodes.size() - 1; j++) {
                links[j] = cp.getPT().getLink(nodes.get(j), nodes.get(j + 1)).getID();
            }
            // Get the size of the route in km
            double sizeRoute = 0;
            for (int i = 0; i < links.length; i++) {               
                sizeRoute += ((EONLink) cp.getPT().getLink(links[i])).getWeight();
            }
            // Adaptative modulation:
            int modulation = Modulation.getBestModulation(sizeRoute);

            // Calculates the required slots
            int requiredSlots = Modulation.convertRateToSlot((int) flow.getMaxRate(), EONPhysicalTopology.getSlotSize(), modulation);
            if(requiredSlots>=100000)
                    continue OUTER;
            
            // Evaluate if each link have space to the required slots
            for (int i = 0; i < links.length; i++) {
                if (((EONLink) cp.getPT().getLink(links[i])).isIsInterupted()||!((EONLink) cp.getPT().getLink(links[i])).hasSlotsAvaiable(requiredSlots)) {
                    continue;
                }
            }
            // First-Fit spectrum assignment in some modulation 
            int[] firstSlot;
            for (int i = 0; i < links.length; i++) {
                // Try the slots available in each link
                firstSlot = ((EONLink) cp.getPT().getLink(links[i])).getSlotsAvailableToArray(requiredSlots);
                for (int j = 0; j < firstSlot.length; j++) {
                    // Now you create the lightpath to use the createLightpath VT
                    //Relative index modulation: BPSK = 0; QPSK = 1; 8QAM = 2; 16QAM = 3;
                    EONLightPath lp = cp.createCandidateEONLightPath(flow.getSource(), flow.getDestination(), links,
                            firstSlot[j], (firstSlot[j] + requiredSlots - 1), modulation);
                    // Now you try to establish the new lightpath, accept the call
                    if ((id = cp.getVT().createLightpath(lp)) >= 0) {
                        // Single-hop routing (end-to-end lightpath)
                        lps[0] = cp.getVT().getLightpath(id);
                        if (cp.upgradeFlow(flow, lps)) {
                            return true;
                        } else {
                            // Something wrong
                            // Dealocates the lightpath in VT and try again
                            cp.getVT().deallocatedLightpath(id);
                        }
                    }
                }
            }
        }

        return false;
        
    }    
    
    
    @Override
    public void flowDeparture(long id) {
        
    }

    @Override
    public void disasterArrival(DisasterArea area) {
        
        ArrayList<Flow> survivedFlows = cp.getMappedFlowsAsList();
        ///Step 1: For each existing/survived connection of set
        ///S(⊂C), degrade the bandwidth to one unit. 

        for (Flow f : survivedFlows) {

            cp.degradeFlow(f, 1);

        } 
        
        //Step 2: For each disrupted connection of set D(⊂C), reprovision
        //it on the shortest available candidate
        //path P(c,k) with one bandwidth unit.
        /*for (Flow flow : cp.getInteruptedFlows()) {

            rerouteFlow(flow);

        }*/
        
        ArrayList<Flow> interuptedFlows = new ArrayList<Flow>(cp.getInteruptedFlows());
        /*for (Iterator<Flow> i = interuptedFlows.iterator(); i.hasNext();) {
            Flow flow = i.next();
            if (flow.calcDegradation() == 0.0f || Double.isNaN(flow.calcDegradation())) {                 
                cp.dropFlow(flow);
                flow.updateTransmittedBw();
                i.remove();

            }

        }*/

        //Step 3: Sort all connections of set H=(S∪D) in ascending
        //order of αc.        
        ArrayList<Flow> allFlows = new ArrayList<Flow>();
        allFlows.addAll(interuptedFlows);
        allFlows.addAll(survivedFlows);

        Comparator<Flow> comparator = new Comparator<Flow>() {
            @Override
            public int compare(Flow t, Flow t1) {
                return Double.compare(t.calcDegradation(), t1.calcDegradation());
            }
        };

        //Step 4: For the first connection c in set H, if αc ≥ 1, remove
        //this connection, go to Step 4; otherwise,
        //upgrade connection c by 1 bandwidth unit; if
        //successful, go to Step 3; otherwise, go to Step 5.        
        while (allFlows.size() > 0) {

            Collections.sort(allFlows, comparator);
            
            /*System.out.println();
            for(Flow f:allFlows){
                
                System.out.println(f.calcDegradation());
                
            }
            System.out.println();*/
            
            Flow flow = allFlows.get(0);

            if (flow.calcDegradation() >= 1) { 
                
                if(interuptedFlows.contains(flow)){
                    
                    flow.updateTransmittedBw();
                    cp.restoreFlow(flow);
                }
                    
                allFlows.remove(flow);                
                continue;

            } else {

                if (!cp.upgradeFlow(flow, null)) {
                    
                    if(!addLightPath(flow)){
                                                
                        allFlows.remove(flow);
                        //System.out.println("Dropou " + flow.calcDegradation());
                        cp.dropFlow(flow);
                        flow.updateTransmittedBw();
                        
                    }  
                    
                }

            }

        }
        
    }

    @Override
    public void disasterDeparture() {
        //throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }

    @Override
    public void delayedFlowDeparture(Flow f) {
        
    }    

    @Override
    public void delayedFlowArrival(Flow f) {
        f.dropFlow();
    }
    
}
