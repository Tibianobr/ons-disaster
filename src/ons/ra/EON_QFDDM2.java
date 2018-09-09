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
import java.util.TreeSet;
import ons.DisasterArea;
import ons.EONLightPath;
import ons.EONLink;
import ons.EONPhysicalTopology;
import ons.Flow;
import ons.LightPath;
import ons.Modulation;
import ons.Path;
import ons.PhysicalTopology;
import ons.util.Dijkstra;
import ons.util.WeightedGraph;
import ons.util.YenKSP;

/**
 *
 * @author Gab
 */
public class EON_QFDDM2 implements RA {

    private ControlPlaneForRA cp;
    private WeightedGraph graph;

    @Override
    public void simulationInterface(ControlPlaneForRA cp) {
        this.cp = cp;
        this.graph = cp.getPT().getWeightedGraph();
    }

    @Override
    public void flowArrival(Flow flow) {
        int[] nodes;
        int[] links;
        long id;
        LightPath[] lps = new LightPath[1];          
        /*ArrayList<Integer>[] paths = YenKSP.kShortestPaths(graph, flow.getSource(), flow.getDestination(), 3);
        flow.setPaths(paths);   */    
        
        //this.graph = this.getPostDisasterGraph(cp.getPT());

        // Shortest-Path routing        
        nodes = Dijkstra.getShortestPath(graph, flow.getSource(), flow.getDestination());

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
        // Block the call
        //System.out.println("Block Arrival");
        cp.blockFlow(flow.getID());
    }

    private LightPath getLeastLoadedLightpath(Flow flow) {
        long abw_aux, abw = 0;
        LightPath lp_aux, lp = null;

        // Get the available lightpaths
        TreeSet<LightPath> lps = cp.getVT().getAvailableLightpaths(flow.getSource(),
                flow.getDestination(), flow.getRate());

        if (lps != null && !lps.isEmpty()) {
            while (!lps.isEmpty()) {
                lp_aux = lps.pollFirst();
                // Get the available bandwidth
                for (int l : lp_aux.getLinks()) {

                    if (cp.getPT().getLink(l).isIsInterupted()) {
                        return null;
                    }

                }

            }
        }

        if (lps != null && !lps.isEmpty()) {
            while (!lps.isEmpty()) {
                lp_aux = lps.pollFirst();
                // Get the available bandwidth
                abw_aux = cp.getVT().getLightpathBWAvailable(lp_aux.getID());
                if (abw_aux > flow.getBwReq()) {
                    abw = abw_aux;
                    lp = lp_aux;
                }
            }
        }
        return lp;
    }

    private LightPath getLeastLoadedLightpathCustomRate(Flow flow, int rate) {
        long abw_aux, abw = 0;
        LightPath lp_aux, lp = null;

        // Get the available lightpaths
        TreeSet<LightPath> lps = cp.getVT().getAvailableLightpaths(flow.getSource(),
                flow.getDestination(), rate);

        if (lps != null && !lps.isEmpty()) {
            while (!lps.isEmpty()) {
                lp_aux = lps.pollFirst();
                // Get the available bandwidth
                for (int l : lp_aux.getLinks()) {

                    if (cp.getPT().getLink(l).isIsInterupted()) {
                        return null;
                    }

                }

            }
        }

        if (lps != null && !lps.isEmpty()) {
            while (!lps.isEmpty()) {
                lp_aux = lps.pollFirst();
                // Get the available bandwidth
                abw_aux = cp.getVT().getLightpathBWAvailable(lp_aux.getID());
                if (abw_aux > flow.getBwReq()) {
                    abw = abw_aux;
                    lp = lp_aux;
                }
            }
        }
        return lp;
    }

    private WeightedGraph getPostDisasterGraph(PhysicalTopology pt) {

        int nodes = pt.getNumNodes();
        WeightedGraph g = new WeightedGraph(nodes);
        for (int i = 0; i < nodes; i++) {
            for (int j = 0; j < nodes; j++) {
                if (pt.hasLink(i, j)) {
                    if (!pt.getLink(i, j).isIsInterupted()) {
                        g.addEdge(i, j, pt.getLink(i, j).getWeight());
                    } else {
                        g.addEdge(i, j, Integer.MAX_VALUE);
                    }
                }
            }
        }

        return g;
    }

    private boolean restoreFlow(Flow flow) {

        ArrayList<Integer> nodes = new ArrayList<Integer>();
        int[] links;
        long id;
        LightPath[] lps = new LightPath[1];
        ArrayList<Integer>[] paths = YenKSP.kShortestPaths(getPostDisasterGraph(cp.getPT()), flow.getSource(), flow.getDestination(), 3);
        flow.setPaths(paths);
        
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

            if (modulation == -1) {
                continue;
            }

            // Calculates the required slots
            int requiredSlots = Modulation.convertRateToSlot((int) flow.getMaxRate(), EONPhysicalTopology.getSlotSize(), modulation);

            // Evaluate if each link have space to the required slots
            for (int i = 0; i < links.length; i++) {
                if (!((EONLink) cp.getPT().getLink(links[i])).hasSlotsAvaiable(requiredSlots) || ((EONLink) cp.getPT().getLink(links[i])).isIsInterupted()) {
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

    public boolean addLightPath(Flow flow) {

        ArrayList<Integer> nodes = new ArrayList<Integer>();
        int[] links;
        long id;
        LightPath[] lps = new LightPath[1];

        /*lps[0] = getLeastLoadedLightpathCustomRate(flow, 1000);
        if (lps[0] instanceof LightPath) {
            if (cp.upgradeFlow(flow, lps)) {
                return true;
            }
        }*/

        /*System.out.println();
        for(ArrayList<Integer> lp :flow.getPaths()){
        
            System.out.println(lp);
                
        }
        System.out.println();*/
        //Caso a conexão seja sobrevivente
        if (flow.getPaths() == null) {

            ArrayList<Integer>[] paths = YenKSP.kShortestPaths(getPostDisasterGraph(cp.getPT()), flow.getSource(), flow.getDestination(), 3);
            flow.setPaths(paths);

        }

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

                //System.out.println(((EONLink) cp.getPT().getLink(links[i])).getWeight());
                sizeRoute += ((EONLink) cp.getPT().getLink(links[i])).getWeight();
            }

            //System.out.println("Size Route: " + sizeRoute);
            // Adaptative modulation:
            int modulation = Modulation.getBestModulation(sizeRoute);
            //System.out.println("Size route " + sizeRoute);
            if (modulation == -1) {
                continue;
            }

            // Calculates the required slots
            int requiredSlots = Modulation.convertRateToSlot((int) flow.getMaxRate(), EONPhysicalTopology.getSlotSize(), modulation);
            //System.out.println("Slot size: " + EONPhysicalTopology.getSlotSize() +" Required slots: " + requiredSlots + " Modulation: " + modulation);
            // Evaluate if each link have space to the required slots
            for (int i = 0; i < links.length; i++) {
                if (!((EONLink) cp.getPT().getLink(links[i])).hasSlotsAvaiable(requiredSlots) || cp.getPT().getLink(links[i]).isIsInterupted()) {
                    //System.out.println("DEu ruim aqui");
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
                            //System.out.println("Peguei: " + nodes);
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
    public void disasterArrival(DisasterArea area) {

        ArrayList<Flow> survivedFlows = cp.getMappedFlowsAsList();
        ArrayList<EONLightPath> degradationFriends = new ArrayList<EONLightPath>();

        Comparator<Flow> comparator = new Comparator<Flow>() {
            @Override
            public int compare(Flow t, Flow t1) {

                int t1Deg = t.getServiceInfo().getServiceInfo();
                int t2Deg = t1.getServiceInfo().getServiceInfo();
                int sComp = Integer.compare(t1Deg, t2Deg);

                if (sComp != 0) {

                    return sComp;

                } else {

                    return Double.compare(t.calcDegradation(), t1.calcDegradation());

                }

            }
        };

        ///Step 1: For each existing/survived connection of set
        ///S(⊂C), degrade the bandwidth to one unit.    
        int max = 0;
        for (Flow f : survivedFlows) {

            if (f.isDegradeTolerant()) {

                cp.degradeFlow(f, /*f.getMaxDegradationNumber()*/ f.getMaxDegradationNumberEon());

            }

        }

        //System.out.println(cp.getInteruptedFlows().size());
        //Step 2: For each disrupted connection of set D(⊂C), reprovision
        //it on the shortest available candidate
        //path P(c,k) with one bandwidth unit.
        /*Collections.sort(cp.getInteruptedFlows(), comparator);
        for (Flow flow : cp.getInteruptedFlows()) {

            restoreFlow(flow);

        }*/
        
        ArrayList<Flow> interuptedFlows = new ArrayList<Flow>(cp.getInteruptedFlows());
        /*for (Iterator<Flow> i = interuptedFlows.iterator(); i.hasNext();) {
            Flow flow = i.next();
            if (flow.calcDegradation() == 0.0f || Double.isNaN(flow.calcDegradation())) {

                i.remove();

                if (flow.isDelayTolerant()) {

                    //Delay tolerant
                    cp.getDelayedFlows().add(flow);

                } else {

                    //Drop Flow
                    cp.dropFlow(flow);  

                }
                flow.updateTransmittedBw();

            }

        }*/
        //System.out.println(interuptedFlows.size());

        //Step 3: Sort all connections of set H=(S∪D) in ascending
        //order of αc.        
        ArrayList<Flow> allFlows = new ArrayList<Flow>();
        allFlows.addAll(interuptedFlows);
        allFlows.addAll(survivedFlows);

        //Step 4: For the first connection c in set H, if αc ≥ 1, remove
        //this connection, go to Step 4; otherwise,
        //upgrade connection c by 1 bandwidth unit; if
        //successful, go to Step 3; otherwise, go to Step 5.        
        while (interuptedFlows.size() > 0) {

            Collections.sort(interuptedFlows, comparator);
            Flow flow = interuptedFlows.get(0);

            /*System.out.println();
            for(Flow f:allFlows){
                
                System.out.println("Classe de serviço: " + f.getServiceInfo().getServiceInfo() + " Degradation Rate: " + f.calcDegradation());
                
            }
            System.out.println();*/
            float aux_degr = 1;
            if (flow.isDegradeTolerant()) {
                aux_degr = (flow.getMaxDegradationNumberEon() / 2) / (float) flow.getRequiredSlots();
            }

            if (flow.calcDegradation() >= aux_degr) {
                if(interuptedFlows.contains(flow))
                    cp.getRestoredFlows().add(flow);
                interuptedFlows.remove(flow);
                continue;

            } else {

                if (!cp.upgradeFlow(flow, null)) {

                    if (!addLightPath(flow)) {

                        interuptedFlows.remove(flow);

                        if (flow.isDelayTolerant()) {

                            //Delay tolerant
                            cp.getDelayedFlows().add(flow);

                        } else {

                            //Drop Flow
                            cp.dropFlow(flow);                           

                        }
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
        //f.updateTransmittedBw();
    }

    /*@Override
    public void delayedFlowsArrival() {

        cp.delayedFlowsArrival();

        /*for(Flow f:cp.getDelayedFlows())
            cp.delayedFlowDeparture(f);*/
    //}

    @Override
    public void delayedFlowArrival(Flow f) {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }

}
