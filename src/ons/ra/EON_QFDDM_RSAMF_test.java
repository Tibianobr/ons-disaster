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

import static ons.ra.EON_FDM.convertIntegers;

/**
 * @author Gab
 */
public class EON_QFDDM_RSAMF_test implements RA {

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
        ArrayList<Candidate> candidates = new ArrayList<>();
        LightPath[] lps = new LightPath[1];          
       /* ArrayList<Integer>[] paths = Ye nKSP.kShortestPaths(graph, flow.getSource(), flow.getDestination(), 3);
        flow.setPaths(paths);   */
        System.out.println("Arrival!");
        System.out.println("Chegou Flow = " + flow.getSource() + " até " + flow.getDestination());
        ArrayList<Integer>[] paths = YenKSP.kDisruptedShortestPaths(cp.getPT().getWeightedGraph(), flow.getSource(), flow.getDestination(), 3);
        System.out.println("Possíveis caminhos:");
        for (int i = 0; i < paths.length; i++)
            System.out.println(paths[i]);
        flow.setPaths(paths);
        //this.graph = this.getPostDisasterGraph(cp.getPT());

        int currentCandidate = -1;
        int lessFragPathId = -1;
        Double lessFragPathValue = Double.POSITIVE_INFINITY;
        for (ArrayList<Integer> path : paths)
            candidates.add(new Candidate(path));
        OUTER:
        for (Candidate candidate : candidates) {
            candidate.setNodes(convertIntegers(candidate.getPath()));

            // Se não há caminho disponível
            if (candidate.getNodes().length == 0) {
                cp.blockFlow(flow.getID());
                candidate.setValid(false);
                return;
            }

            // Cria o vetor de links do caminho
            candidate.setLinks(new int[candidate.getNodes().length]);
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
                    cp.blockFlow(flow.getID());
                    candidate.setValid(false);
                    return;
                }
            }

            // Calcular os blocos livres de cada candidato, maiores slots, fragmentação e slots que usará
            for (int i = 0; i < candidate.getLinks().length; i++)
            {
                candidate.setSlots(((EONLink) cp.getPT().getLink(candidate.getLinks()[i])).getSlotsAvailableToArray(candidate.getRequiredSlots()));
                candidate.setLargerSlotBlock(candidate.getLargerSlotBlock() + ((EONLink) cp.getPT().getLink(candidate.getLinks()[i])).maxSizeAvaiable());
                candidate.setFreeSlots(candidate.getFreeSlots() + ((EONLink) cp.getPT().getLink(candidate.getLinks()[i])).getAvaiableSlots());


                // Cria o lightpath a cada link
                for (int j = 0 ; j < candidate.getSlots().length; j++)
                {
                    candidate.setLp(cp.createCandidateEONLightPath(
                            flow.getSource(),
                            flow.getDestination(),
                            candidate.getLinks(),
                            candidate.getSlots()[j],
                            (candidate.getSlots()[j] + candidate.getRequiredSlots() - 1),
                            candidate.getModulation()));
                }
            }
            candidate.setFragmentation((double) candidate.getLargerSlotBlock()/ (double) candidate.getFreeSlots());
        }


        for (Candidate candidate:candidates) {
            currentCandidate++;
            if (candidate.getFragmentation() < lessFragPathValue) {
                lessFragPathId = currentCandidate;
                lessFragPathValue = candidate.getFragmentation();
            }
        }

        Candidate smallestFrag = candidates.get(lessFragPathId);

        System.out.println(smallestFrag.getLp());
        id = cp.getVT().createLightpath(smallestFrag.getLp());
        lps[0] = cp.getVT().getLightpath(id);
        cp.acceptFlow(flow.getID(),lps);
        return;




        // antigo first fit
//        if ((id = cp.getVT().createLightpath(lp)) >= 0) {
//            // Single-hop routing (end-to-end lightpath)
//            lps[0] = cp.getVT().getLightpath(id);
//            if (cp.acceptFlow(flow.getID(), lps)) {
//                //  System.out.println("Aceito! o caminho = " + links[i]);
//                // return;
//            } else {
//                // Something wrong
//                // Dealocates the lightpath in VT and try again
//                cp.getVT().deallocatedLightpath(id);
//            }
//        }

        // Block the call
        //System.out.println("Block Arrival");
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

    public boolean addLightPath(Flow flow) {

        ArrayList<Integer> nodes = new ArrayList<Integer>();
        int[] links;
        long id;
        LightPath[] lps = new LightPath[1];

        if (flow.getPaths() == null) {

            ArrayList<Integer>[] paths = YenKSP.kDisruptedShortestPaths(getPostDisasterGraph(cp.getPT()), flow.getSource(), flow.getDestination(), 3);
            flow.setPaths(paths);

        }

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
            if (requiredSlots >= 100000)
                continue OUTER;

            // Evaluate if each link have space to the required slots
            for (int i = 0; i < links.length; i++) {
                if (((EONLink) cp.getPT().getLink(links[i])).isIsInterupted() || !((EONLink) cp.getPT().getLink(links[i])).hasSlotsAvaiable(requiredSlots)) {
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

            if (f.isDegradeTolerant()) {

                cp.degradeFlow(f, /*f.getMaxDegradationNumber()*/ f.getMaxDegradationNumberEon());

            }

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

                int t1Deg = t.getServiceInfo().getServiceInfo();
                int t2Deg = t1.getServiceInfo().getServiceInfo();
                int sComp = Integer.compare(t1Deg, t2Deg);

                if (sComp != 0) {

                    return sComp;

                } else {

                    return Double.compare(t.getServiceInfo().getDelayTolerance(), t1.getServiceInfo().getDelayTolerance());

                }

            }
        };


        //Step 4: For the first connection c in set H, if αc ≥ 1, remove
        //this connection, go to Step 4; otherwise,
        //upgrade connection c by 1 bandwidth unit; if
        //successful, go to Step 3; otherwise, go to Step 5.        
        while (allFlows.size() > 0) {

            Collections.sort(allFlows, comparator);
            Flow flow = allFlows.get(0);

            /*System.out.println();
            for(Flow f:allFlows){
                
                System.out.println("Classe de serviço: " + f.getServiceInfo().getServiceInfo() + " Degradation Rate: " + f.calcDegradation());
                
            }
            System.out.println();*/
            /*float aux_degr = 1;
            if (flow.isDegradeTolerant()) {
                aux_degr = (flow.getMaxDegradationNumberEon()) / (float) flow.getRequiredSlots();
            }*/

            if (flow.calcDegradation() >= 1 - flow.getServiceInfo().getDegradationTolerance()) {
                if (interuptedFlows.contains(flow)) {

                    flow.updateTransmittedBw();
                    cp.restoreFlow(flow);

                }

                allFlows.remove(flow);
                continue;

            } else {

                if (!cp.upgradeFlow(flow, null)) {

                    if (!addLightPath(flow)) {

                        allFlows.remove(flow);

                        if (flow.isDelayTolerant()) {

                            //Delay tolerant
                            cp.delayFlow(flow);

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

                if (!addLightPath(f)) {

                    break;

                }

            }

        }

        if (f.calcDegradation() < 1 - f.getServiceInfo().getDegradationTolerance()) {
            //System.out.println("Dropou com " + f.calcDegradation() + " precisava de: " + (1- f.getServiceInfo().getDegradationTolerance()));
            cp.dropFlow(f);
        } else {
            //System.out.println("Restaurou " + f.calcDegradation());
            cp.restoreFlow(f);

        }

    }


}
