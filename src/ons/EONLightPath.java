/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ons;

/**
 * The Lightpath for Elastic Optical Networks (EON) provides
 * an lightpath that joins traffic from various links with
 * different number of slots. Thus, the lightpath can have
 * variable bit rates, which can vary by modulation and number
 * of flows on it.
 *
 * @author onsteam
 */
public class EONLightPath extends LightPath {

    private final int firstSlot;
    private final int lastSlot;
    private final int modulation; //relative index modulation
    private final int bw; //bandwidth in Mbps 
    private int bwAvailable; //bandwidth available in Mbps 
   
    /**
     * Creates a new EONLightpath object. All its attributes must be
     * given by parameter, except for the avaiable bandwidth,
     * that will be calculated based on the numbers of slots 
     * it size and tha path's modulation
     * 
     * @param id the EONLightpath's unique identifier
     * @param src the source node ID of the Lightpath
     * @param dst the destination node ID of the Lightpath
     * @param links the array of EONLinks ID contained in the Lighpath
     * @param firstSlot the fisrt slot of the lightpath
     * @param lastSlot the last slot of the lightpath
     * @param modulation the modulation used on the Lightpath
     * @param slotSize the size of each slot
     */
    public EONLightPath(long id, int src, int dst, int[] links, int firstSlot, int lastSlot, int modulation, int slotSize) {
        super(id, src, dst, links);
        this.firstSlot = firstSlot;
        this.lastSlot = lastSlot;
        this.modulation = modulation;
        this.bw = Modulation.convertSlotToRate(((lastSlot - firstSlot) + 1), slotSize, modulation);
        this.bwAvailable = Modulation.convertSlotToRate(((lastSlot - firstSlot) + 1), slotSize, modulation);
    }
    
    /**
     * Retrieves the Lightpath's first slot position on the fiber
     * 
     * @return the first slot position
     */
    public int getFirstSlot() {
        return firstSlot;
    }

    /**
     * Retrieves the Lightpath's last slot position on the fiber
     * 
     * @return the last slot position
     */
    public int getLastSlot() {
        return lastSlot;
    }
    
    /**
     * Returns the number of slots containing in the Lightpath.
     * 
     * @return the number of slots in this Lightpath
     */
    public int getSlots(){
        return lastSlot - firstSlot + 1;
    }
    
    /**
     * Retrieves the Lightpath's modulation
     * 
     * @return the value of the Lightpath's modulation 
     */
    public int getModulation() {
        return modulation;
    }
    
    /**
     * Retrieves the Lightpath's avaiable bandwidth
     * 
     * @return the value of the Lightpath's avaiable bandwidth
     */
    public int getBwAvailable() {
        return bwAvailable;
    }
    
    /**
     * Retrieves the Lightpath's total bandwidth
     * 
     * @return the value of the Lightpath's total bandwidth
     */
    public int getBw() {
        return bw;
    }
    
    /**
     * Add a Flow in the Lightpath
     * 
     * @param bw the bandwidth used by the Flow
     */
    public void addFlowOnLightPath(int bw) {
        if (bw > this.bwAvailable){
            throw (new IllegalArgumentException());
        } else {
            this.bwAvailable = this.bwAvailable - bw;
        }
    }
    
    public boolean canAddFlowOnLightPath(int bw){
        //System.out.println("Id: "+ this.getID() + "Precisa de: " + bw + " tem: " + bwAvailable);
        if (bw > this.bwAvailable){
            return false;
        } 
        return true;
    }
    
    /**
     * Remove a Flow on the Lighpath
     * 
     * @param bw the bandwidth used by the Flow
     */
    public void removeFlowOnLightPath(int bw) {
        if (bw > this.bw){
            throw (new IllegalArgumentException());
        } else {
            this.bwAvailable = this.bwAvailable + bw;
        }
    }
    
    /**
     * Prints all information related to the EONLighpath object.
     * 
     * @return string containing all the values of the lighpaths's parameters.
     */
    @Override
    public String toString() {
        String lightpath = Long.toString(id) + "; " + Integer.toString(src) + " " + Integer.toString(dst) + " ";
        for (int i = 0; i < links.length; i++) {
            lightpath += Integer.toString(links[i]) + " (" + Integer.toString(firstSlot) + "->" + Integer.toString(lastSlot) + ") ";
        }
        return lightpath;
    }
    
    @Override
    public String toTrace() {
        String lightpath = Long.toString(id) + "; " + Integer.toString(src) + " " + Integer.toString(dst) + " ";
        for (int i = 0; i < links.length; i++) {
            lightpath += Integer.toString(links[i]) + "_" + Integer.toString(firstSlot) + "->" + Integer.toString(lastSlot) + ") ";
        }
        return lightpath;
    }

}
