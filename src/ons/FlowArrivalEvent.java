/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package ons;

/**
 * Methods to treat the incoming of a Flow object.
 * 
 * @author onsteam
 */
public class FlowArrivalEvent extends Event {
	
    private Flow flow;
    
    /**
     * Creates a new FlowArrivalEvent object.
     * 
     * @param flow the arriving flow
     */
    public FlowArrivalEvent(Flow flow) {
        this.flow = flow;
    }
    
    /**
     * Retrives the flow attribute of the FlowArrivalEvent object.
     * 
     * @return the FlowArrivalEvent's flow attribute
     */
    public Flow getFlow() {
        return this.flow;
    }
    
    /**
     * Prints all information related to the arriving flow.
     * 
     * @return string containing all the values of the flow's parameters
     */
    @Override
    public String toString() {
        return "Arrival: "+flow.toString();
    }

    /**
     * @param flow the flow to set
     */
    public void setFlow(Flow flow) {
        this.flow = flow;
    }
}
