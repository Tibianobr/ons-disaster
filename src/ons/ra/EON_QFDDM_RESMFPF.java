/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ons.ra;

import ons.*;
import ons.util.WeightedGraph;
import ons.util.YenKSP;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import static ons.ra.EON_FDM.convertIntegers;

/**
 * @author Gab
 */
public class EON_QFDDM_RESMFPF implements RA {

    private ControlPlaneForRA cp;
    private WeightedGraph graph;

    @Override
    public void simulationInterface(ControlPlaneForRA cp) {
        this.cp = cp;
        this.graph = cp.getPT().getWeightedGraph();
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

    @Override
    public void flowArrival(Flow flow) {

        int[] nodes;
        int[] links;
        long id;
        LightPath[] lps = new LightPath[1];
       /* ArrayList<Integer>[] paths = Ye nKSP.kShortestPaths(graph, flow.getSource(), flow.getDestination(), 3);
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
            if (requiredSlots >= 100000)
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
                            //  System.out.println(path);
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

    public LightPath[] addLightPath(Flow flow) {
        long id;
        LightPath[] lps = new LightPath[1];
        ArrayList<Candidate> candidateArrayList = new ArrayList<>();


        if (flow.getPaths() == null) {

            ArrayList<Integer>[] paths = YenKSP.kDisruptedShortestPaths(getPostDisasterGraph(cp.getPT()), flow.getSource(), flow.getDestination(), 3);
            flow.setPaths(paths);

        }

        for (ArrayList<Integer> path : flow.getPaths())
            candidateArrayList.add(new Candidate(path));

        OUTER:
        for (Candidate candidate : candidateArrayList) {
            candidate.setNodes(convertIntegers(candidate.getPath()));

            // Se não há caminho disponível
            if (candidate.getNodes().length == 0) {
                candidate.setValid(false);
                continue;
            }

            // Cria o vetor de links do caminho
            candidate.setLinks(new int[candidate.getNodes().length - 1]);
            for (int j = 0; j < candidate.getNodes().length - 1; j++)
                candidate.getLinks()[j] = cp.getPT().getLink(candidate.getNodes()[j], candidate.getNodes()[j + 1]).getID();

            // Calcula o tamanho da rota em km
            for (int i = 0; i < candidate.getLinks().length; i++)
                candidate.setSizeRoute(candidate.getSizeRoute() + cp.getPT().getLink(candidate.getLinks()[i]).getWeight());

            candidate.setModulation(Modulation.getBestModulation(candidate.getSizeRoute()));
            candidate.setRequiredSlots(Modulation.convertRateToSlot(flow.getBwReq(), EONPhysicalTopology.getSlotSize(), candidate.getModulation()));

            if (candidate.getRequiredSlots() >= 100000) continue OUTER;

            // Verificar se cada link tem capacidade de slots para a chamada
            for (int i = 0; i < candidate.getLinks().length; i++) {
                if (!((EONLink) cp.getPT().getLink(candidate.getLinks()[i])).hasSlotsAvaiable(candidate.getRequiredSlots())) {
                    candidate.setValid(false);
                    continue;
                }
            }
        }

        List<Candidate> remove = new ArrayList<>();

        // Se ficou inválido o caminho ele exclui da lista
        for (Candidate candidate : candidateArrayList) {
            if (candidate.isValid())
                for (int i = 0; i < candidate.getLinks().length; i++) {
                    candidate.setSlots(((EONLink) cp.getPT().getLink(candidate.getLinks()[i])).getSlotsAvailableToArray(candidate.getRequiredSlots()));
                    candidate.setLargerSlotBlock(((EONLink) cp.getPT().getLink(candidate.getLinks()[i])).maxSizeAvaiable());
                    candidate.setFreeSlots(((EONLink) cp.getPT().getLink(candidate.getLinks()[i])).getAvaiableSlots());
                    if (1 - ((double) candidate.getLargerSlotBlock() / (double) candidate.getFreeSlots()) < candidate.getFragmentation())
                    {
                        candidate.setFragmentation(1 - ((double) candidate.getLargerSlotBlock() / (double) candidate.getFreeSlots()));
                        //   link_frag = i;
                    }
                }
            else {
                remove.add(candidate);
            }
        }

        for (Candidate c:remove) {
            candidateArrayList.remove(c); }

        Collections.sort(candidateArrayList, Comparator.comparingDouble(Candidate::getFragmentation).reversed());
        for (int k = 0; k < candidateArrayList.size(); k++) {
            Candidate smallestFrag = getSmallestFrag(candidateArrayList, k);
              System.out.println(smallestFrag.getFragmentation());

            for (int i = 0; i < smallestFrag.getLinks().length; i++) {
                smallestFrag.setSlots(((EONLink) cp.getPT().getLink(smallestFrag.getLinks()[i]))
                        .getSlotsAvailableToArray(smallestFrag.getRequiredSlots()));

                for (int j = 0; j < smallestFrag.getSlots().length; j++) {
                    smallestFrag.setLp(cp.createCandidateEONLightPath(
                            flow.getSource(),
                            flow.getDestination(),
                            smallestFrag.getLinks(),
                            smallestFrag.getSlots()[j],
                            (smallestFrag.getSlots()[j] + smallestFrag.getRequiredSlots() - 1),
                            smallestFrag.getModulation()));
                    if ((id = cp.getVT().createLightpath(smallestFrag.getLp())) >= 0) {
                        lps[0] = cp.getVT().getLightpath(id);
                        if (cp.upgradeFlow(flow, lps)) {
                            return lps;
                        } else {
                            cp.getVT().deallocatedLightpath(id);
                        }
                    }
                }

            }
        }
        return null;

    }


    private Candidate getSmallestFrag(List<Candidate> candidateList, Integer k) {
        return candidateList.get(k);
    }

    @Override
    public void flowDeparture(long id) {
    }

    @Override
    public void disasterArrival(DisasterArea area) {
        ArrayList<Flow> survivedFlows = cp.getMappedFlowsAsList();
        // Degrada as conexões sobreviventes
        for (Flow f : survivedFlows) {
            if (f.isDegradeTolerant()) {
                cp.degradeFlow(f, f.getMaxDegradationNumberEon());
            }
        }
        ArrayList<Flow> interuptedFlows = new ArrayList<Flow>(cp.getInteruptedFlows());
        ArrayList<Flow> allFlows = new ArrayList<Flow>();
        allFlows.addAll(interuptedFlows);
        while (allFlows.size() > 0) {
            Flow flow = allFlows.get(0);
            if (addLightPath(flow) != null) {
                flow.setRate(0);
                flow.dropFlow();
                flowArrival(flow);
                cp.getRestoredFlows().add(flow);
                flow.updateTransmittedBw();
            }
            else {
                if (flow.isDelayTolerant()) {
                    //Delay tolerant
                    cp.delayFlow(flow);
                } else {
                    //Drop Flow
                    cp.dropFlow(flow);
                }
                flow.updateTransmittedBw();
            }
            allFlows.remove(flow);

        }

    }

    @Override
    public void disasterDeparture() {
        //throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
        //System.out.println("Acabou " + SimulationRunner.timer);
    }

    @Override
    public void delayedFlowDeparture(Flow f) {
        //f.updateTransmittedBw();
    }

    public void delayedFlowArrival(Flow f) {


        //System.out.println("Holding time: " + (f.getDepartureEvent().getTime() - f.getArrivalEvent().getTime()) + " Tempo: " + (SimulationRunner.timer - TrafficGenerator.disasterArrival[TrafficGenerator.globalCount]));
        ArrayList<Integer>[] paths = YenKSP.kShortestPaths(this.getPostDisasterGraph(cp.getPT())/*cp.getPT().getWeightedGraph()*/, f.getSource(), f.getDestination(), 3);
        f.setPaths(paths);
        //System.out.println("Tratamento iniciado " + SimulationRunner.timer);

        while (f.calcDegradation() < 1 - f.getServiceInfo().getDegradationTolerance()) {

            if (!cp.upgradeFlow(f, null)) {

                if (addLightPath(f) == null) {

                    break;

                }

            }

        }

        if (f.calcDegradation() < 1 - f.getServiceInfo().getDegradationTolerance()) {
            cp.dropFlow(f);
        } else {
            cp.restoreFlow(f);

        }

    }


}
