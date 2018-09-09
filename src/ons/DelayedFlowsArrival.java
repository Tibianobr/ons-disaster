/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ons;

/**
 *
 * @author Gab
 */
public class DelayedFlowsArrival extends Event{
    
    private Flow flow;
    
    public DelayedFlowsArrival(Flow f) {
        flow = f;
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
