/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package ons;

/**
 * Methods to treat the outgoing of a Flow object.
 * 
 * @author onsteam
 */
public class FlowDepartureEvent extends Event{

    protected long id;
    private Flow f;
    
    /**
     * Creates a new FlowDepartureEvent object.
     * 
     * @param id unique identifier of the outgoing flow
     */
    public FlowDepartureEvent(long id, Flow flow) {
        this.id = id;
        f = flow;
    }
    
    /**
     * Retrieves the identifier of the FlowDepartureEvent object.
     * 
     * @return the FlowDepartureEvent's id attribute
     */
    public long getID() {
        return this.id;
    }
    
    /**
     * Prints all information related to the outgoing flow.
     * 
     * @return string containing all the values of the flow's parameters
     */
    @Override
    public String toString() {
        return "Departure: "+Long.toString(id);
    }

    /**
     * @return the f
     */
    public Flow getF() {
        return f;
    }

    /**
     * @param f the f to set
     */
    public void setF(Flow f) {
        this.f = f;
    }
}
