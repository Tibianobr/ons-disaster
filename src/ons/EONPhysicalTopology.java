package ons;

import ons.util.WeightedGraph;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

/**
 * The EON Physical Topology object
 * @author onsteam
 */
public class EONPhysicalTopology extends PhysicalTopology {

    private static int slotSize;
    private static int maxModulation;
    
    public EONPhysicalTopology(Element xml) {
        super(xml);

        int id;
        int groomingInPorts, groomingOutPorts, capacity = 0;
        int[] modulations = new int[Modulation.N_MOD];
        boolean generalCapacity = false, generalModulation = false;
        double delay, weight;
        String[] parts;

        try {
            // Checking the atributtes of <nodes> tag for general values
            NodeList nodesEntities = xml.getElementsByTagName("nodes");
            if (((Element) nodesEntities.item(0)).hasAttribute("modulations")) {
                generalModulation = true;
                for (int i = 0; i < modulations.length; i++) {
                    modulations[i] = 0;
                }
                parts = (((Element) nodesEntities.item(0)).getAttribute("modulations").split(",[ ]*"));
                for (String part : parts) {
                    modulations[Modulation.convertModulationTypeToInteger(part)] = 1;
                }
                for (int i = modulations.length - 1; i >= 0; i--) {
                    if(modulations[i] == 1){
                        maxModulation = i;
                        break;
                    }
                }
            }
            if (((Element) nodesEntities.item(0)).hasAttribute("capacity")) {
                generalCapacity = true;
                capacity = Integer.parseInt(((Element) nodesEntities.item(0)).getAttribute("capacity"));
            }
            // Process nodes
            NodeList nodelist = xml.getElementsByTagName("node");
            nodes = nodelist.getLength();
            if (Simulator.verbose) {
                System.out.println(Integer.toString(nodes) + " nodes");
            }
            nodeVector = new EONOXC[nodes];
            for (int i = 0; i < nodes; i++) {
                id = Integer.parseInt(((Element) nodelist.item(i)).getAttribute("id"));
                groomingInPorts = Integer.parseInt(((Element) nodelist.item(i)).getAttribute("grooming-in-ports"));
                groomingOutPorts = Integer.parseInt(((Element) nodelist.item(i)).getAttribute("grooming-out-ports"));
                if (!generalModulation) {
                    for (int j = 0; j < modulations.length; j++) {
                        modulations[j] = 0;
                    }
                    parts = (((Element) nodelist.item(i)).getAttribute("modulations").split(",[ ]*"));
                    //FIXME: Making exception handling if the user does not put the 'modulations' in all nodes
                    for (String part : parts) {
                        modulations[Modulation.convertModulationTypeToInteger(part)] = 1;
                    }
                    for (int m = modulations.length - 1; m >= 0; m--) {
                        if (modulations[m] == 1) {
                            maxModulation = m;
                            break;
                        }
                    }
                }
                if (!generalCapacity) {
                    //FIXME: Making exception handling if the user does not put the 'capacity' in all nodes
                    capacity = Integer.parseInt(((Element) nodelist.item(i)).getAttribute("capacity"));
                }
                nodeVector[id] = new EONOXC(id, groomingInPorts, groomingOutPorts, capacity, modulations);
            }

            int src, dst, slots = 0, guardband = 0;
            EONPhysicalTopology.slotSize = 0;
            boolean generalSlots = false, generalGuardband = false, generalSlotSize = false;
            
            // Checking the atributtes of <links> tag for general values
            NodeList linksEntities = xml.getElementsByTagName("links");
            if (((Element) linksEntities.item(0)).hasAttribute("slots")) {
                generalSlots = true;
                slots = Integer.parseInt(((Element) linksEntities.item(0)).getAttribute("slots"));
            }
            if (((Element) linksEntities.item(0)).hasAttribute("guardband")) {
                generalGuardband = true;
                guardband = Integer.parseInt(((Element) linksEntities.item(0)).getAttribute("guardband"));
            }
            if (((Element) linksEntities.item(0)).hasAttribute("slot-size")) {
                generalSlotSize = true;
                EONPhysicalTopology.slotSize = Integer.parseInt(((Element) linksEntities.item(0)).getAttribute("slot-size"));
            }

            // Process links
            NodeList linklist = xml.getElementsByTagName("link");
            links = linklist.getLength();
            if (Simulator.verbose) {
                System.out.println(Integer.toString(links) + " links");
            }
            linkVector = new EONLink[links];
            adjMatrix = new EONLink[nodes][nodes];
            for (int i = 0; i < links; i++) {
                id = Integer.parseInt(((Element) linklist.item(i)).getAttribute("id"));
                src = Integer.parseInt(((Element) linklist.item(i)).getAttribute("source"));
                dst = Integer.parseInt(((Element) linklist.item(i)).getAttribute("destination"));
                delay = Double.parseDouble(((Element) linklist.item(i)).getAttribute("delay"));
                if (!generalSlots) {
                    // FIXME: Fazer o tratamento de execao caso o usuario nao coloque o 'slots' em todos os links
                    slots = Integer.parseInt(((Element) linklist.item(i)).getAttribute("slots"));
                }
                if (!generalGuardband) {
                    //FIXME: Making exception handling if the user does not put the 'guardband' in all links
                    guardband = Integer.parseInt(((Element) linklist.item(i)).getAttribute("guardband"));
                }
                if (!generalSlotSize) {
                    //FIXME: Making exception handling if the user does not put the 'slot-size' in all links
                    EONPhysicalTopology.slotSize = Integer.parseInt(((Element) linksEntities.item(0)).getAttribute("slot-size"));
                }
                weight = Double.parseDouble(((Element) linklist.item(i)).getAttribute("weight"));
                linkVector[id] = adjMatrix[src][dst] = new EONLink(id, src, dst, delay, weight, slots, guardband);
            }
        } catch (Throwable t) {
            t.printStackTrace();
        }

    }
    
    /**
     * Retrieves the slot size in MHz.
     * @return slot size in MHz
     */
    public static int getSlotSize() {
        return slotSize;
    }

    /**
     * Retrieves the max modulation format in xml schema.
     * @return the id of max modulation format allowed
     */
    public static int getMaxModulation() {
        return maxModulation;
    }
    
    /**
     * Allocates optical path network.
     * @param lightpath the lightpath will be alocated
     */
    @Override
    public void createPhysicalLightpath(LightPath lightpath) {
        for (int i = 0; i < lightpath.links.length; i++) {
            ((EONLink) linkVector[lightpath.links[i]]).reserveSlots(lightpath.id, ((EONLightPath) lightpath).getFirstSlot(), ((EONLightPath) lightpath).getLastSlot());
        }
        //Set the transponder used in this lp and Reserve ports
        lightpath.setTx(this.getNode(this.getLink(lightpath.links[0]).getSource()).reserveGroomingInputPort());
        lightpath.setRx(this.getNode(this.getLink(lightpath.links[lightpath.links.length - 1]).getDestination()).reserveGroomingOutputPort());
    }
    
    /**
     * Examine whether it is possible to allocate the supplied optical path.
     * @param lightpath the optical path
     * @return true if is possible, false otherwise
     */
    @Override
    public boolean canCreatePhysicalLightpath(LightPath lightpath) {
        //continuity test
        if (!checkLinkPath(lightpath.links)) {
            return false; 
        }
        //modulation test
        if (!((((EONOXC) nodeVector[lightpath.getSource()]).hasModulation(((EONLightPath) lightpath).getModulation()))
                && (((EONOXC) nodeVector[lightpath.getDestination()]).hasModulation(((EONLightPath) lightpath).getModulation())))) {            
            return false;
        }
        //Available transceivers test
        if (!getNode(getLink(lightpath.links[0]).getSource()).hasFreeGroomingInputPort()) {
            return false;
        }
        if (!getNode(getLink(lightpath.links[lightpath.links.length - 1]).getDestination()).hasFreeGroomingOutputPort()) {
            return false;
        }
        //transceiver capacity test
        if (((EONOXC) nodeVector[lightpath.getSource()]).getCapacity() < (((EONLightPath) lightpath).getLastSlot() - ((EONLightPath) lightpath).getFirstSlot() + 1)) {
            return false;
        }
        //RSA test
        for (int i = 0; i < lightpath.links.length; i++) {
            if (!(((EONLink) linkVector[lightpath.links[i]]).areSlotsAvaiable(((EONLightPath) lightpath).getFirstSlot(), ((EONLightPath) lightpath).getLastSlot()))) {
                return false;
            }
        }
        return true;
    }

    /**
     * Deallocates optical path provided.
     * @param lightpath the optical path provided
     */
    @Override
    public void removePhysicalLightpath(LightPath lightpath) {
        for (int i = 0; i < lightpath.links.length; i++) {
            ((EONLink) linkVector[lightpath.links[i]]).releaseSlots(((EONLightPath) lightpath).getFirstSlot(), ((EONLightPath) lightpath).getLastSlot());
        }
        // Release ports
        this.getNode(lightpath.getSource()).releaseGroomingInputPort(lightpath.Tx);
        this.getNode(lightpath.getDestination()).releaseGroomingOutputPort(lightpath.Rx);
    }
    
    /**
     * Retrieves the bandwidth available in Mbps in this lightpath
     * @param lightpath the lightpath to be examined
     * @return the bandwidth available
     */
    @Override
    public int getBWAvailable(LightPath lightpath) {
        return ((EONLightPath) lightpath).getBwAvailable();
    }
    
    /**
     * Add flow in this lightpath.
     * @param flow the flow to be add
     * @param lightpath the lightpath
     */
    @Override
    public void addFlow(Flow flow, LightPath lightpath) {
        ((EONLightPath) lightpath).addFlowOnLightPath(flow.getRate());
    }

    /**
     * Retrieves the total bandwidth this lightpath
     * @param lightpath the lightpath
     * @return the bandwidth in Mbps
     */
    @Override
    public int getBW(LightPath lightpath) {
        return ((EONLightPath) lightpath).getBw();               
    }
    
    /**
     * Returns a weighted graph with vertices representing the physical network
     * nodes, and the edges representing the physical links.
     *
     * The weight of each edge receives the same value of the original link
     * weight if the link has at least slots available. Otherwise it has no edges.
     *
     * @param slots the flow rate are available in slots
     * @return an WeightedGraph class object
     */
    public WeightedGraph getWeightedGraph(int slots) {
        EONLink link;
        WeightedGraph g = new WeightedGraph(nodes);
        for (int i = 0; i < nodes; i++) {
            for (int j = 0; j < nodes; j++) {
                if (hasLink(i, j)) {
                    link = (EONLink) getLink(i, j);
                    if (link.maxSizeAvaiable() >= slots) {
                        g.addEdge(i, j, link.getWeight());
                    }
                }
            }
        }
        return g;
    }
    
    /**
     * Examine whether it is possible to add flow in the lightpath.
     * @param flow the flow to be add
     * @param lightpath the lightpath
     * @return true if is possible, false otherwise
     */
    @Override
    public boolean canAddFlow(Flow flow, LightPath lightpath) {
        return ((EONLightPath) lightpath).getBwAvailable() >= flow.getRate();
    }
    
    /**
     * Remove the flow of this litghpath.
     * @param flow the flow to be removed
     * @param lightpath the lightpath
     */
    @Override
    public void removeFlow(Flow flow, LightPath lightpath) {
        ((EONLightPath) lightpath).removeFlowOnLightPath(flow.getRate());
    }
    
    /**
     * Retrieves the slots available in all physical topology links.
     * @return the number of slots available
     */
    public int getAvailableSlots(){
        int slots = 0;
        for(int i = 0; i < links; i++){
            slots = slots + ((EONLink) this.getLink(i)).getAvaiableSlots();
        }
        return slots;
    }

    @Override
    public void degradeFlow() {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }
}
