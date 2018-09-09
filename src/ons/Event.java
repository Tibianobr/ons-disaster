/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package ons;

/**
 * Event objects have only one attribute: scheduled time.
 * It is the time slot the event is set to happen.
 * @author onsteam
 */
public abstract class Event {
    
    protected double time;
   
    /**
     * Sets a new time for the Event to happen.
     * 
     * @param time new scheduled period
     */
    public void setTime(double time){
        this.time = time;
    }
    
    /**
     * Retrieves current scheduled time for a given Event. 
     * 
     * @return value of the Event's time attribute
     */
    public double getTime() {
        return this.time;
    }
}
