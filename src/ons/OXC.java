/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ons;

import java.util.TreeSet;

/**
 * The Optical Cross-Connects (OXCs) are present in nodes, for route data
 * traffic and has grooming input and output ports. Traffic grooming 
 * is the process of grouping many small data flows into larger units, 
 * so they can be processed as single units. Grooming in OXCs has the 
 * objective of minimizing the cost of the network.
 * 
 * @author onsteam
 */
public abstract class OXC {

    protected int id;
   protected int groomingInputPorts;
    protected int groomingOutputPorts;
    protected TreeSet<Integer> freeGroomingInputPorts;
    protected TreeSet<Integer> freeGroomingOutputPorts; 
    
    /**
     * Creates a new OXC object. All its attributes must be given
     * given by parameter, except for the free grooming input and output
     * ports, that, at the beginning of the simulation, are the same as 
     * the total number of grooming input and output ports.
     * 
     * @param id the OXC's unique identifier
     * @param groomingInputPorts total number of grooming input ports
     * @param groomingOutputPorts total number of grooming output ports
     */
    public OXC(int id, int groomingInputPorts, int groomingOutputPorts) {
        this.id = id;
        this.groomingInputPorts = groomingInputPorts;
        this.freeGroomingInputPorts = startGroomingPorts(groomingInputPorts);
        this.groomingOutputPorts = groomingOutputPorts;
        this.freeGroomingOutputPorts = startGroomingPorts(groomingOutputPorts);
    }
    
    /**
     * Retrieves the OXC's unique identifier.
     * 
     * @return the OXC's id attribute
     */
    public int getID() {
        return id;
    }
    
    /**
     * Says whether or not a given OXC has free
     * grooming input port(s).
     * 
     * @return true if the OXC has free grooming input port(s)
     */
    public boolean hasFreeGroomingInputPort() {
        return !freeGroomingInputPorts.isEmpty();
    }
    
    /**
     * Says whether or not a given OXC has all free
     * grooming input port(s).
     * 
     * @return true if the OXC has all free grooming input port(s)
     */
    public boolean allFreeGroomingInputPort() {
        return freeGroomingInputPorts.size() == groomingInputPorts;
    }
    
    /**
     * By decreasing the number of free grooming input ports,
     * this function "reserves" a grooming input port.
     * 
     * @return the number of free grooming input port, if the number is -1 is because has some error
     */
    public int reserveGroomingInputPort() {
        if (!freeGroomingInputPorts.isEmpty()) {
            return freeGroomingInputPorts.pollFirst();
        } else {
            return -1;//if some lightpath has transponder -1 is because has some error in simulator's code
        }
    }
    
    /**
     * By increasing the number of free grooming input ports,
     * this function "releases" a grooming input port.
     * The "groomingInputPort" can be -1 when the "VirtualTopology" remove "Lightpath" that are on a optical grooming.
     * 
     * @param groomingInputPort the grooming input Port to be released
     * @return false if there are no grooming input ports to be freed
     */
    public boolean releaseGroomingInputPort(int groomingInputPort) {
        if ((freeGroomingInputPorts.size() < groomingInputPorts) && (groomingInputPort >= 0)) {
            freeGroomingInputPorts.add(groomingInputPort);
            return true;
        }
        return false;
    }
    
/**
     * Says whether or not a given OXC has free
     * grooming output port(s).
     * 
     * @return true if the OXC has free grooming output port(s)
     */
    public boolean hasFreeGroomingOutputPort() {
        return !freeGroomingOutputPorts.isEmpty();
    }
    
    /**
     * Says whether or not a given OXC has all free
     * grooming output port(s).
     * 
     * @return true if the OXC has all free grooming output port(s)
     */
    public boolean allFreeGroomingOutputPort() {
        return freeGroomingOutputPorts.size() == groomingOutputPorts;
    }
    
    /**
     * By decreasing the number of free grooming output ports,
     * this function "reserves" a grooming output port.
     * 
     * @return the number of free grooming output port, if the number is -1 is because has some error
     */
    public int reserveGroomingOutputPort() {
        if (!freeGroomingOutputPorts.isEmpty()) {
            return freeGroomingOutputPorts.pollFirst();
        } else {
            return -1;
        }
    }
    
    /**
     * By increasing the number of free grooming output ports,
     * this function "releases" a grooming output port.
     * The "groomingOutputPort" can be -1 when the "VirtualTopology" remove "Lightpath" that are on a optical grooming.
     * 
     * @param groomingOutputPort the grooming output Port to be released
     * @return false if there are no grooming output ports to be freed
     */
    public boolean releaseGroomingOutputPort(int groomingOutputPort) {
        if ((freeGroomingOutputPorts.size() < groomingOutputPorts) && (groomingOutputPort >= 0)) {
            freeGroomingOutputPorts.add(groomingOutputPort);
            return true;
        }
        return false;
    }
    
    /**
     * Start grooming ports from the number of groomingPorts provided.
     * @param groomingPorts the groomingPorts provided
     * @return the TreeSet with the freeGroomingPorts
     */
    private TreeSet<Integer> startGroomingPorts(int groomingPorts){
        TreeSet<Integer> ports = new TreeSet<>();
        for(int i = 0; i < groomingPorts; i++){
            ports.add(i);
        }
        return ports;
    }
    
}
