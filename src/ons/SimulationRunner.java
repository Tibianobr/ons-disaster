/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ons;

/**
 * Simply runs the simulation, as long as there are events
 * scheduled to happen.
 * 
 * @author onsteam
 */
public class SimulationRunner {

    public static float timer;
    public static EventScheduler events;
    /**
     * Creates a new SimulationRunner object.
     *
     * @param cp the the simulation's control plane
     * @param events the simulation's event scheduler
     */
    public SimulationRunner(ControlPlane cp, EventScheduler events) {
        Event event;
        timer = 0;
        this.events = events;
        Tracer tr = Tracer.getTracerObject();
        MyStatistics st = MyStatistics.getMyStatisticsObject();        
        while ((event = events.popEvent()) != null) {
            tr.add(event);
            timer = (float) event.getTime();
            if(cp.getPT() instanceof EONPhysicalTopology){
                //to calculate the available spectrum and available transponders statistics
                st.addEvent(event, ((EONPhysicalTopology) cp.getPT()).getAvailableSlots(), cp.getPT().getAllFreeGroomingInputPorts());
            } else{
                //to calculate the available transponders statistics in WDM Simulator
                st.addEvent(event, cp.getPT().getAllFreeGroomingInputPorts());
            }
            cp.newEvent(event);
        }
        int slots_total = 0;
        int slots_cont = 0;

        for(int i = 0; i<=85;i++)
        {
            System.out.println(((EONLink) cp.getPT().getLink(i)).maxSizeAvaiable() + "/" + ((EONLink) cp.getPT().getLink(i)).getAvaiableSlots());
            slots_total += ((EONLink) cp.getPT().getLink(i)).getAvaiableSlots();
            slots_cont += ((EONLink) cp.getPT().getLink(i)).maxSizeAvaiable();
        }

        System.out.println("FRAG = " +  ((double)slots_cont/(double)slots_total));
    }
}
