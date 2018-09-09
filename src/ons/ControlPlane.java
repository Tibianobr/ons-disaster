/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ons;

import java.text.DecimalFormat;
import ons.ra.RA;
import ons.ra.ControlPlaneForRA;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Stack;
import ons.util.Distribution;

/**
 * The Control Plane is responsible for managing resources and connection within
 * the network.
 *
 * @author onsteam
 */
public class ControlPlane implements ControlPlaneForRA { // RA is Routing Assignment Problem

    private RA ra;
    private PhysicalTopology pt;
    private VirtualTopology vt;
    private Map<Flow, Path> mappedFlows; // Flows that have been accepted into the network
    private Map<Long, Flow> activeFlows; // Flows that have been accepted or that are waiting for a RA decision 
    private ArrayList<Flow> interuptedFlows;
    private ArrayList<Flow> delayedFlows;
    private Tracer tr = Tracer.getTracerObject();
    private MyStatistics st = MyStatistics.getMyStatisticsObject();
    public static ArrayList<Flow> droppedFlows;
    public static ArrayList<Flow> restoredFlows;
    ArrayList<EONLightPath> degradationFriends;    
    
    /**
     * Creates a new ControlPlane object.
     *
     * @param raModule the name of the RA class
     * @param pt the network's physical topology
     * @param vt the network's virtual topology
     */
    public ControlPlane(String raModule, PhysicalTopology pt, VirtualTopology vt) {
        Class RAClass;

        mappedFlows = new HashMap<Flow, Path>();
        activeFlows = new HashMap<Long, Flow>();
        interuptedFlows = new ArrayList<Flow>();
        delayedFlows = new ArrayList<Flow>();
        droppedFlows = new ArrayList<Flow>();
        restoredFlows = new ArrayList<Flow>();
        degradationFriends = new ArrayList<EONLightPath>();        
        
        this.pt = pt;
        this.vt = vt;

        try {
            RAClass = Class.forName(raModule);
            ra = (RA) RAClass.newInstance();
            ra.simulationInterface(this);
        } catch (Throwable t) {
            t.printStackTrace();
        }

    }

    /**
     * Deals with an Event from the event queue. If it is of the
     * FlowArrivalEvent kind, adds it to the list of active flows. If it is from
     * the FlowDepartureEvent, removes it from the list.
     *
     * @param event the Event object taken from the queue
     */
    public void newEvent(Event event) {

        if (event instanceof FlowArrivalEvent) {
            newFlow(((FlowArrivalEvent) event).getFlow());
            /*if(((FlowArrivalEvent) event).getFlow().isIsDelayed()){
                
            }*/
                
            ra.flowArrival(((FlowArrivalEvent) event).getFlow());
        } else if (event instanceof FlowDepartureEvent) {            
            ra.flowDeparture(((FlowDepartureEvent) event).getID());            
            removeFlow(((FlowDepartureEvent) event).getID());
        } else if (event instanceof DisasterArrivalEvent) {
            DisasterArrivalEvent disaster = (DisasterArrivalEvent) event;
            applyDisaster(disaster.getArea());
            TrafficGenerator.globalCount++;
            ra.disasterArrival(disaster.getArea());
            //disasterDeparture(disaster.getArea());
            for(Flow f:this.getDelayedFlows()){                
                delayedFlowsTreatment(f);
            }
            this.getDelayedFlows().clear();
            
            
        } else if (event instanceof DisasterDepartureEvent) {
            DisasterDepartureEvent disaster = (DisasterDepartureEvent) event;
            disasterDeparture(disaster.getArea());
            ra.disasterDeparture();            
        } else if (event instanceof DelayedFlowDepartureEvent) {        
            ra.delayedFlowDeparture(((DelayedFlowDepartureEvent) event).getFlow()); 
            delayedFlowDeparture(((DelayedFlowDepartureEvent) event).getFlow());
        }else if (event instanceof DelayedFlowsArrival) {             
            DelayedFlowsArrival e = (DelayedFlowsArrival) event;
            ra.delayedFlowArrival(e.getFlow());
            
        }
    }

    /**
     * Adds a given active Flow object to a determined Physical Topology.
     *
     * @param id unique identifier of the Flow object
     * @param lightpaths the Path, or list of LighPath objects
     * @return true if operation was successful, or false if a problem occurred
     */
    
    @Override
    public void restoreFlow(Flow f){
        
         if(f.calcDegradation()<st.getMinDegr()&&f.calcDegradation()>0)
                st.setMinDegr(f.calcDegradation());
        
        if(f.isIsDelayed())
            delayedFlowConfirmation(f);
        
        //if(f.isDegradeTolerant() && f.calcDegradation()<1){            
            
            //System.out.println("Necessário: " + f.getBwReq() + " Transmitiu: " + f.getTransmittedBw());
            
            double flowAge = SimulationRunner.timer - TrafficGenerator.disasterArrival[TrafficGenerator.globalCount];            
            double holdingTime = f.getDepartureEvent().getTime() - f.getArrivalEvent().getTime();
            double totalBWReq = holdingTime*f.getBwReq();
            double remainingBw = totalBWReq - f.getTransmittedBw();
            double minimumHoldingTime = (remainingBw)/(f.getBwReq()*f.calcDegradation());            
            
            float time = (float) (SimulationRunner.timer + minimumHoldingTime);
            Event event; 
            event = new FlowDepartureEvent(f.getID(),f);
            event.setTime(time + minimumHoldingTime);
            SimulationRunner.events.replaceEvent(f.getDepartureEvent(), event);
            f.setDepartureEvent((FlowDepartureEvent) event);            
            
        //}
            
        
        getRestoredFlows().add(f);  
        
    }
    
    private void delayedFlowConfirmation(Flow f){
                
        st.setDelayedFlows(st.getDelayedFlows()+1); 
        if(SimulationRunner.timer - TrafficGenerator.disasterArrival[TrafficGenerator.globalCount]<100)
            st.setWaitMean(st.getWaitMean()+SimulationRunner.timer - TrafficGenerator.disasterArrival[TrafficGenerator.globalCount]);
        st.setNumFlowsDelayedRestored(st.getNumFlowsDelayedRestored() + 1);
        
    }
    
    @Override
    public boolean acceptFlow(long id, LightPath[] lightpaths) {
        Flow flow;

        if (id < 0 || lightpaths.length < 1) {
            throw (new IllegalArgumentException());
        } else {
            if (!activeFlows.containsKey(id)) {                 
                return false;
            }
            flow = activeFlows.get(id);
            if (!canAddFlowToPT(flow, lightpaths)) {                
                return false;
            }
            /*if (!checkLightpathContinuity(flow, lightpaths)) {                 
                return false;
            }*/
            int usedTransponders = 0;
            for (LightPath lightpath : lightpaths) {
                if (vt.isLightpathIdle(lightpath.getID())) {
                    usedTransponders++;
                }
            }
            addFlowToPT(flow, lightpaths);
            mappedFlows.put(flow, new Path(lightpaths));
            tr.acceptFlow(flow, lightpaths);
            st.userTransponder(usedTransponders);
            st.acceptFlow(flow, lightpaths);
            flow.updateHoldingTime();
            //if(flow.isIsDelayed())
                //System.out.println("Aceita"); 
            return true;
        }
    }

    /**
     * Removes a given Flow object from the list of active flows.
     *
     * @param id unique identifier of the Flow object
     * @return true if operation was successful, or false if a problem occurred
     */
    @Override
    public boolean blockFlow(long id) {
        Flow flow;

        if (id < 0) {
            throw (new IllegalArgumentException());
        } else {
            if (!activeFlows.containsKey(id)) {
                return false;
            }
            flow = activeFlows.get(id);
            if (mappedFlows.containsKey(flow)) {
                return false;
            }
                        
            //if(!flow.isIsDelayed()){
                
                tr.blockFlow(flow);
                st.blockFlow(flow);
                
            //}         
            
            activeFlows.remove(id);
            
            return true;
        }
    }

    /**
     * Removes a given Flow object from the Physical Topology and then puts it
     * back, but with a new route (set of LightPath objects).
     *
     * @param id unique identifier of the Flow object
     * @param lightpaths list of LightPath objects, which form a Path
     * @return true if operation was successful, or false if a problem occurred
     */
    @Override
    public boolean rerouteFlow(long id, LightPath[] lightpaths) {
        Flow flow;
        Path oldPath;

        if (id < 0 || lightpaths.length < 1) {
            throw (new IllegalArgumentException());
        } else {
            if (!activeFlows.containsKey(id)) {
                return false;
            }
            flow = activeFlows.get(id);
            if (!mappedFlows.containsKey(flow)) {
                return false;
            }
            oldPath = mappedFlows.get(flow);
            removeFlowFromPT(flow, lightpaths);
            if (!canAddFlowToPT(flow, lightpaths)) {
                addFlowToPT(flow, oldPath.getLightpaths());
                return false;
            }
            /*if (!checkLightpathContinuity(flow, lightpaths)) {
                return false;
            }*/
            addFlowToPT(flow, lightpaths);
            mappedFlows.put(flow, new Path(lightpaths));
            //tr.flowRequest(id, true);
            return true;
        }
    }
    
    @Override
    public void delayedFlowDeparture(Flow f){
                
        if(f.getTransmittedBw()<f.getBwReq()){
            
            dropFlow(f);
            //System.out.println("Dropou");
            
        }            
        this.removeFlow(f.getID());
        
    }
    
    
    public void delayedFlowsArrival(){
        
        
        /*Comparator<Flow> comparator = new Comparator<Flow>() {
            @Override
            public int compare(Flow t, Flow t1) {
                
                int t1Deg = t.getServiceInfo().getServiceInfo();
                int t2Deg = t1.getServiceInfo().getServiceInfo();
                int sComp = Integer.compare(t1Deg, t2Deg);
                
                if(sComp != 0){
                    
                    return sComp;
                    
                }else{
                    
                    return Double.compare(t.getServiceInfo().getDelayTolerance(), t1.getServiceInfo().getDelayTolerance());
                    
                }              
                                
            }
        };
        
        delayedFlows.sort(comparator);
        
        for(Flow f:delayedFlows){
            
            if(f.isIsDelayed())
                continue;
            
            double holdingTime = (f.getDepartureEvent().time - f.getArrivalEvent().time);
            
            float delayTolerance = f.getServiceInfo().getDelayTolerance();
            double remainingBw = f.getBwReq()-f.getTransmittedBw();  
            double flowAge = SimulationRunner.timer - TrafficGenerator.disasterArrival[TrafficGenerator.globalCount];
            double minimumHoldingTime = flowAge*remainingBw/f.getTransmittedBw();
            double durationTime = f.getDepartureEvent().getTime() - f.getArrivalEvent().getTime();
                        
            
            
            if(flowAge+minimumHoldingTime<=(delayTolerance*durationTime)){
                                
                double gap = (durationTime + delayTolerance*durationTime) - flowAge+minimumHoldingTime;
                Flow flow = f;
                flow.setBwReq((int) (f.getBwReq()-f.getTransmittedBw()));
                flow.setIsDelayed(true);
                float time = (float) (SimulationRunner.timer+gap);                
                Event event = new FlowArrivalEvent(flow);
                event.setTime(time);
                flow.setArrivalEvent((FlowArrivalEvent)event);
                Simulator.events.addEvent(event);  
                
                event = new DelayedFlowDepartureEvent(flow.getID());
                event.setTime(time + minimumHoldingTime);
                flow.setDepartureEvent((FlowDepartureEvent) event);
                Simulator.events.addEvent(event);    
                
            }else{               
                
                System.out.println("Deu ruim");
                delayedFlowDeparture(f);                
                
            }
            
        }*/
        
        //delayedFlows.clear();
        
    }
    
    public void dropFlow(Flow f){
        
        /*double droppedTraffic = 0;//st.getDroppedTraffic();
        int droppedFlows = 0;//st.getDroppedFlows();
        int restoredFlows =0; //st.getRestoredFlows();        
        //float dropRate = st.getDropRate();     
                       
        droppedTraffic=+f.getDroppedTraffic();
        droppedFlows+=1;
        restoredFlows-=1;
        //dropRate = (float)droppedFlows/(droppedFlows+restoredFlows);
        
        st.setDroppedTraffic(st.getDroppedTraffic() + droppedTraffic);
        st.setDroppedFlows(st.getDroppedFlows() + droppedFlows);
        //st.setDropRate(dropRate);
        st.setRestoredFlows(st.getRestoredFlows() + restoredFlows);
        
        if (f.getServiceInfo().getServiceInfo() == 0) {
            st.setFullServiceDroppedFlows(st.getFullServiceDroppedFlows() + 1);
        }
        if (f.getServiceInfo().getServiceInfo() == 1) {
            st.setDelayTolerantDroppedFlows(st.getDelayTolerantDroppedFlows() + 1);
        }
        if (f.getServiceInfo().getServiceInfo() == 2) {
            st.setDegradedTolerantDroppedFlows(st.getDegradedTolerantDroppedFlows() + 1);
        }
        if (f.getServiceInfo().getServiceInfo() == 3) {
            st.setDegradedServiceDroppedFlows(st.getDegradedServiceDroppedFlows() + 1);
        }*/   
        
        /*if(SimulationRunner.timer - TrafficGenerator.disasterArrival[TrafficGenerator.globalCount]>100)
            System.out.println("Deu ruim: " + TrafficGenerator.globalCount);*/
        
        if(f.isIsDelayed()&&SimulationRunner.timer - TrafficGenerator.disasterArrival[TrafficGenerator.globalCount]<100)
            st.setWaitMean(st.getWaitMean()+SimulationRunner.timer - TrafficGenerator.disasterArrival[TrafficGenerator.globalCount]);
            
        f.setRate(0);
        f.dropFlow();
        this.droppedFlows.add(f);
        this.removeActiveFlow(f.getID());
        
        
    }

    private void applyDisaster(DisasterArea area) {
        
        for(Integer link : area.getLinks()){
            
            pt.linkVector[link].setIsInterupted(true);
            
        }
        
        for (int i = 0; i < mappedFlows.size(); i++) {

            Path path = (Path) mappedFlows.values().toArray()[i];
            LightPath lightpath;
            lightpath = path.getLightpaths()[0];

            for (int link : lightpath.getLinks()) {

                if (area.getLinks().contains(link)) {

                    Flow flow = (Flow) mappedFlows.keySet().toArray()[i];                     
                    degradeFlow(flow,flow.getNumWave());
                    interuptedFlows.add(flow);
                    break;                    

                }

            }

        }
        
        
        for (Flow f : this.interuptedFlows) {
            LightPath[] lightpaths;
            if (activeFlows.containsKey(f.getID()) && mappedFlows.containsKey(f)) {
                lightpaths = mappedFlows.get(f).getLightpaths();
                f.setInterupted(true);
                removeFlowFromPT(f, lightpaths);
                mappedFlows.remove(f);
            }
        }

    }

    /**
     * Adds a given Flow object to the HashMap of active flows. The HashMap also
     * stores the object's unique identifier (ID).
     *
     * @param flow Flow object to be added
     */
    private void newFlow(Flow flow) {
        activeFlows.put(flow.getID(), flow);
    }

    /**
     * Removes a given Flow object from the list of active flows.
     *
     * @param id the unique identifier of the Flow to be removed
     */
    private void removeFlow(long id) {
        Flow flow;
        LightPath[] lightpaths;

        if (activeFlows.containsKey(id)) {
            flow = activeFlows.get(id);
            if (mappedFlows.containsKey(flow)) {                
                lightpaths = mappedFlows.get(flow).getLightpaths();
                removeFlowFromPT(flow, lightpaths);
                mappedFlows.remove(flow);
            }
            activeFlows.remove(id);
        }
    }
    
    @Override
    public void removeActiveFlow(long id){
        removeFlow(id);
    }

    /**
     * Removes a given Flow object from a Physical Topology.
     *
     * @param flow the Flow object that will be removed from the PT
     * @param lightpaths a list of LighPath objects
     */
    private void removeFlowFromPT(Flow flow, LightPath[] lightpaths) {        
        for (LightPath lightpath : lightpaths) {            
            pt.removeFlow(flow, lightpath);            
            // Can the lightpath be removed?
            EONLightPath elp = (EONLightPath) lightpath;
            /*if((flow.isInterupted()&&flow.calcDegradation()<1)||(flow.getServiceInfo().getDegradationTolerance()>0&&flow.calcDegradation()<1)){
                System.out.println("Degra: " + flow.calcDegradation() + " Tam: " + elp.getBw() + " Disponivel: " + elp.getBwAvailable());
            }*/
                
            if (vt.isLightpathIdle(lightpath.getID())||(flow.isInterupted()&&flow.calcDegradation()<1)||(flow.getServiceInfo().getDegradationTolerance()>0&&flow.calcDegradation()<1)) {                
                vt.removeLightPath(lightpath.getID());
            }
        }
    }

    /**
     * Says whether or not a given Flow object can be added to a determined
     * Physical Topology, based on the amount of bandwidth the flow requires
     * opposed to the available bandwidth.
     *
     * @param flow the Flow object to be added
     * @param lightpaths list of LightPath objects the flow uses
     * @return true if Flow object can be added to the PT, or false if it can't
     */
    private boolean canAddFlowToPT(Flow flow, LightPath[] lightpaths) {
        for (LightPath lightpath : lightpaths) {
            if (!pt.canAddFlow(flow, lightpath)) {
                return false;
            }
            for(int l:lightpath.getLinks()){
                Link link = pt.getLink(l);
                if(link.isIsInterupted()){
                    return false;
                }
            }
        }
        return true;
    }

    /**
     * Adds a Flow object to a Physical Topology. This means adding the flow to
     * the network's traffic, which simply decreases the available bandwidth.
     *
     * @param flow the Flow object to be added
     * @param lightpaths list of LightPath objects the flow uses
     */
    private void addFlowToPT(Flow flow, LightPath[] lightpaths) {
        for (LightPath lightpath : lightpaths) {
            pt.addFlow(flow, lightpath);
        }
    }

    @Override
    public boolean upgradeFlow(Flow f, LightPath[] lps) {
                      
        if (!mappedFlows.containsKey(f)&& lps!=null) {

            if (acceptFlow(f.getID(), lps)) {
                f.getUsedLps().add((EONLightPath) lps[0]);                
                f.upgradeFlow(1); 
                return true;

            } else {
                
                return false;

            }

        }

        if (this.getPT() instanceof EONPhysicalTopology) {
            
            Path path = mappedFlows.get(f);

            if (path == null) {
                return false;
            }
            if (f.getNumWave() < f.getNumWavesInUse()) {
                               
                return false;

            }
            
            
            if(lps==null){
                
                for(LightPath lp: path.getLightpaths()){
                    
                    EONLightPath elp = (EONLightPath)lp;
                    
                    /*double sizeRoute = 0;
                    for (int i = 0; i < elp.getLinks().length; i++) {               
                        sizeRoute += ((EONLink) getPT().getLink(elp.getLinks()[i])).getWeight();
                    }
                    int modulation = Modulation.getBestModulation(sizeRoute);                 
                    
                    // Calculates the required slots
                    int requiredSlots = Modulation.convertRateToSlot((int) (f.getBwReq()*f.calcDegradation()) + (int)f.getMaxRate(), EONPhysicalTopology.getSlotSize(), modulation);                    
                    
                    EONLightPath canlp = createCandidateEONLightPath(f.getSource(), f.getDestination(), elp.getLinks(), elp.getFirstSlot(), elp.getLastSlot() + 1, modulation);
                    this.vt.removeLightPath(elp.getID());
                    
                    if(canlp.canAddFlowOnLightPath(canlp.getBwAvailable())) {
                        canlp.addFlowOnLightPath(canlp.getBwAvailable());
                        f.upgradeFlow(1);                        
                        mappedFlows.replace(f, path);
                        f.getUsedLps().add(elp);                         
                        return true;
                    }*/
                    
                                      
                    
                    
                    //System.out.println("Precisa alocar  "+1000+"  e tem     "+elp.getBwAvailable());
                    if (elp.canAddFlowOnLightPath((int) f.getMaxRate())) {
                        elp.addFlowOnLightPath((int) f.getMaxRate());
                        f.upgradeFlow(1);                        
                        mappedFlows.replace(f, path);
                        f.getUsedLps().add(elp);                         
                        return true;
                    }
                    
                }
                //System.out.println("Upgrade falhou");
                /*EONLightPath elp = (EONLightPath) path.getLightpaths()[0];
                if (elp.canAddFlowOnLightPath((int) f.getMaxRate())) {
                    elp.addFlowOnLightPath((int) f.getMaxRate());
                    f.upgradeFlow(1);
                    mappedFlows.replace(f, path);
                    return true;
                } */
                
            }else{
                
                //Add lightpath (Multipath Upgrade)
                for (int i = 0; i < lps.length; i++) {                    
                    LightPath lp = lps[i];
                    if (getPT().canAddFlow(f, lp)) {
                        getPT().addFlow(f, lp);
                        f.upgradeFlow(1);
                    } else {                        
                        return false;

                    }

                }
                path.addLightpaths(lps);
                mappedFlows.replace(f, path);
                return true;
                
            }
                        
            return false;
            
        } else {

            Path path = mappedFlows.get(f);

            if (path == null) {
                return false;
            }

            List<LightPath> list = new ArrayList<LightPath>(Arrays.asList(path.getLightpaths()));

            if (f.getNumWave() < f.getNumWavesInUse()) {

                return false;

            }
            for (int i = 0; i < lps.length; i++) {

                LightPath lp = lps[i];                  
                if (getPT().canAddFlow(f, lp)) {
                    getPT().addFlow(f, lp);
                    f.upgradeFlow(1);
                } else {

                    return false;

                }

            }
            path.addLightpaths(lps);
            mappedFlows.replace(f, path);
            return true;

        }    

    }

    @Override
    public void degradeFlow(Flow f, int waveUnit) {
        
        if (this.getPT() instanceof EONPhysicalTopology) {
            if (mappedFlows.containsKey(f)){
                Flow flow = f;
                Path path = mappedFlows.get(flow);       
                
                EONLightPath testeLp = (EONLightPath) path.getLightpaths()[0];
                //System.out.println("Tem disponivel " + testeLp.getBwAvailable());
                
                if(waveUnit>=f.getNumWave()){  
                    
                    f.degradeFlow(waveUnit);
                    mappedFlows.replace(flow, path);
                    removeFlow(flow.getID());
                    newFlow(flow);
                    return;
                
                }
                                                
                for (int i = 0; i < waveUnit; i++) {
                    for (LightPath lp : Arrays.asList(path.getLightpaths())) {
                        EONLightPath elp = (EONLightPath) lp;
                        //System.out.println("Removendo " + (int)f.getMaxRate());
                        elp.removeFlowOnLightPath((int) f.getMaxRate());

                    }
                    f.degradeFlow(1);
                }
                
                if (f.getNumWavesInUse() > 0) {
                    //rerouteFlow(flow.getID(),remaningLps);                    
                    
                    mappedFlows.replace(flow, path);
                    testeLp = (EONLightPath) mappedFlows.get(f).getLightpaths()[0];
                    //System.out.println(testeLp.getBwAvailable());
                }
                
            }           
            
            
        } else if (mappedFlows.containsKey(f)) {

            Flow flow = f;
            Path path = mappedFlows.get(flow);
            Stack<LightPath> list;
            list = new Stack<LightPath>();

            for (LightPath lp : Arrays.asList(path.getLightpaths())) {
                list.push(lp);
            }

            int auxWave = waveUnit;

            if (waveUnit > flow.getNumWave()) {

                auxWave = flow.getNumWave();

            }
            for (int i = 0; i < auxWave; i++) {

                LightPath lp = list.pop();
                getPT().removeFlow(flow, lp);
                flow.degradeFlow(1);

            }
            LightPath[] remaningLps = new LightPath[list.size()];
            remaningLps = list.toArray(remaningLps);
            Path pathNovo = new Path(remaningLps);

            if (remaningLps.length > 0) {
                //rerouteFlow(flow.getID(),remaningLps);            
                mappedFlows.replace(flow, pathNovo);

            } else {

                mappedFlows.replace(flow, pathNovo);
                removeFlow(flow.getID());
                newFlow(flow);

            }

        }

    }

    @Override
    public ArrayList<Flow> getInteruptedFlows() {

        return interuptedFlows;

    }

    @Override
    public ArrayList<Flow> getActiveFlows() {

        ArrayList<Flow> flows = new ArrayList<Flow>();

        for (int i = 0; i < activeFlows.size(); i++) {

            Flow flow = (Flow) activeFlows.keySet().toArray()[i];
            if(flow.calcDegradation()>0)
                flows.add(flow);

        }

        return flows;

    }
    
    public ArrayList<Flow> getMappedFlowsAsList(){
        
        ArrayList<Flow> flows = new ArrayList<Flow>();

        for (int i = 0; i < mappedFlows.size(); i++) {

            Flow flow = (Flow) mappedFlows.keySet().toArray()[i];
            if(flow.calcDegradation()>0)
                flows.add(flow);

        }

        return flows;
        
    }

    /**
     * Checks the lightpaths continuity in multihop and if flow src and dst is
     * equal in lightpaths
     *
     * @param flow the flow requisition
     * @param lightpaths the set of lightpaths
     * @return true if evething is ok, false otherwise
     */
    private boolean checkLightpathContinuity(Flow flow, LightPath[] lightpaths) {
        if (flow.getSource() == lightpaths[0].getSource() && flow.getDestination() == lightpaths[lightpaths.length - 1].getDestination()) {
            for (int i = 0; i < lightpaths.length - 1; i++) {
                if (!(lightpaths[i].getDestination() == lightpaths[i + 1].getSource())) {
                    return false;
                }
            }
            return true;
        } else {            
            return false;
        }
    }

    /**
     * Retrieves a Path object, based on a given Flow object. That's possible
     * thanks to the HashMap mappedFlows, which maps a Flow to a Path.
     *
     * @param flow Flow object that will be used to find the Path object
     * @return Path object mapped to the given flow
     */
    @Override
    public Path getPath(Flow flow) {
        return mappedFlows.get(flow);
    }

    /**
     * Retrieves the complete set of Flow/Path pairs listed on the mappedFlows
     * HashMap.
     *
     * @return the mappedFlows HashMap
     */
    @Override
    public Map<Flow, Path> getMappedFlows() {
        return mappedFlows;
    }

    /**
     * Retrieves a Flow object from the list of active flows.
     *
     * @param id the unique identifier of the Flow object
     * @return the required Flow object
     */
    @Override
    public Flow getFlow(long id) {
        return activeFlows.get(id);
    }

    /**
     * Counts number of times a given LightPath object is used within the Flow
     * objects of the network.
     *
     * @param id unique identifier of the LightPath object
     * @return integer with the number of times the given LightPath object is
     * used
     */
    @Override
    public int getLightpathFlowCount(long id) {
        int num = 0;
        Path p;
        LightPath[] lps;
        ArrayList<Path> ps = new ArrayList<>(mappedFlows.values());
        for (Path p1 : ps) {
            p = p1;
            lps = p.getLightpaths();
            for (LightPath lp : lps) {
                if (lp.getID() == id) {
                    num++;
                    break;
                }
            }
        }
        return num;
    }

    /**
     * Retrieves the PhysicalTopology object
     *
     * @return PhysicalTopology object
     */
    @Override
    public PhysicalTopology getPT() {
        return pt;
    }

    /**
     * Retrieves the VirtualTopology object
     *
     * @return VirtualTopology object
     */
    @Override
    public VirtualTopology getVT() {
        return vt;
    }

    /**
     * Creates a WDM LightPath candidate to put in the Virtual Topology (this
     * method should be used by RA classes)
     *
     * @param src the source node of the lightpath
     * @param dst the destination node of the lightpath
     * @param links the id links used by lightpath
     * @param wavelengths the wavelengths used by lightpath
     * @return the WDMLightPath object
     */
    @Override
    public WDMLightPath createCandidateWDMLightPath(int src, int dst, int[] links, int[] wavelengths) {
        return new WDMLightPath(1, src, dst, links, wavelengths);
    }

    /**
     * Creates a EON LightPath candidate to put in the Virtual Topology (this
     * method should be used by RA classes)
     *
     * @param src the source node of the lightpath
     * @param dst the destination node of the lightpath
     * @param links the id links used by lightpath
     * @param firstSlot the first slot used in this lightpath
     * @param lastSlot the last slot used in this lightpath
     * @param modulation the modulation id used in this lightpath
     * @return the EONLightPath object
     */
    @Override
    public EONLightPath createCandidateEONLightPath(int src, int dst, int[] links, int firstSlot, int lastSlot, int modulation) {
        return new EONLightPath(1, src, dst, links, firstSlot, lastSlot, modulation, EONPhysicalTopology.getSlotSize());
    }

    @Override
    public void disasterDeparture(DisasterArea area) {         
        
        float period = SimulationRunner.timer - TrafficGenerator.disasterArrival[TrafficGenerator.globalCount];
        //System.out.println("Disruption Period: " + period);
        st.setDisruptionPeriod(st.getDisruptionPeriod() + period);
        
        //System.out.println("Tamanho Or: " + interuptedFlows.size() + " Tam Drop: " + droppedFlows.size() + " Tam Rest: " + restoredFlows.size());
        for (Integer link : area.getLinks()) {
                           
            pt.linkVector[link].setIsInterupted(false);

        }
        interuptedFlows.clear(); 
        //System.out.println("Links reparados " + SimulationRunner.timer);
                

    }
       

    @Override
    public Path getFlowPath(Flow f) {
        return mappedFlows.get(f);
    }

    @Override
    public boolean canAddLightPath(Flow flow,LightPath lightpath) {
      
        if (!pt.canAddFlow(flow, lightpath)) {
            return false;
        }

        return true;
    }

    /**
     * @return the delayedFlows
     */
    @Override
    public ArrayList<Flow> getDelayedFlows() {
        return delayedFlows;
    }

    /**
     * @param delayedFlows the delayedFlows to set
     */
    @Override
    public void setDelayedFlows(ArrayList<Flow> delayedFlows) {
        this.delayedFlows = delayedFlows;
    }

    @Override
    public ArrayList<EONLightPath> getDegradationFriends() {
        return degradationFriends;
    }

    @Override
    public ArrayList<Flow> getDroppedFlows() {
        return this.droppedFlows;
    }

    @Override
    public ArrayList<Flow> getRestoredFlows() {
        return this.restoredFlows;
    }

    @Override
    public void updateData() {
        
        
        for(Flow f: this.restoredFlows){
                      
            st.setDisasterEventBWReq(st.getDisasterEventBWReq() + f.getBwReq());            
            if(f.getServiceInfo().getServiceInfo()==0)
                st.setFullServiceFlows(st.getFullServiceFlows()+1);
            if(f.getServiceInfo().getServiceInfo()==2)
                st.setDelayTolerantFlows(st.getDelayTolerantFlows()+1);
            if(f.getServiceInfo().getServiceInfo()==1)
                st.setDegradedTolerantFlows(st.getDegradedTolerantFlows()+1);
            if(f.getServiceInfo().getServiceInfo()==3)
                st.setDegradedServiceFlows(st.getDegradedServiceFlows()+1);
            
        }
        
        for(Flow f: this.getDroppedFlows()){          
                        
            if (f.getServiceInfo().getServiceInfo() == 0) {
                st.setFullServiceDroppedFlows(st.getFullServiceDroppedFlows() + 1);
            }
            if (f.getServiceInfo().getServiceInfo() == 2) {
                st.setDelayTolerantDroppedFlows(st.getDelayTolerantDroppedFlows() + 1);
            }
            if (f.getServiceInfo().getServiceInfo() == 1) {
                st.setDegradedTolerantDroppedFlows(st.getDegradedTolerantDroppedFlows() + 1);
            }
            if (f.getServiceInfo().getServiceInfo() == 3) {
                st.setDegradedServiceDroppedFlows(st.getDegradedServiceDroppedFlows() + 1);
            }
            
            DecimalFormat df = new DecimalFormat("0.00");
            double droppedTraffic = Double.valueOf(df.format(f.getDroppedTraffic()).toString().replace(",","."));

            st.setDroppedTraffic(st.getDroppedTraffic() + droppedTraffic);
            
            st.setDisasterEventBWReq(st.getDisasterEventBWReq() + f.getBwReq());
            if(f.getServiceInfo().getServiceInfo()==0)
                st.setFullServiceFlows(st.getFullServiceFlows()+1);
            if(f.getServiceInfo().getServiceInfo()==2)
                st.setDelayTolerantFlows(st.getDelayTolerantFlows()+1);
            if(f.getServiceInfo().getServiceInfo()==1)
                st.setDegradedTolerantFlows(st.getDegradedTolerantFlows()+1);
            if(f.getServiceInfo().getServiceInfo()==3)
                st.setDegradedServiceFlows(st.getDegradedServiceFlows()+1);
            
        }
        
        int droppedFlows = this.getDroppedFlows().size();
        int restoredFlows = this.getRestoredFlows().size();
        float droppedRate = (float)droppedFlows/(droppedFlows+restoredFlows);
        float restoreRate = (float)restoredFlows/(droppedFlows+restoredFlows);
        st.setFairness(1-st.getMinDegr());        
        st.setDropRate(droppedRate);
        st.setRestoreRate(restoreRate);
        st.setDroppedFlows(droppedFlows);
        st.setRestoredFlows(restoredFlows);
        
        
    }

    @Override
    public void delayedFlowsTreatment(Flow f) {
        
        /*float delayTolerance = f.getServiceInfo().getDelayTolerance();
        double remainingBw = f.getBwReq() - f.getTransmittedBw();
        double flowAge = SimulationRunner.timer - TrafficGenerator.disasterArrival[TrafficGenerator.globalCount];
        double minimumHoldingTime = flowAge * remainingBw / f.getTransmittedBw();
        double holdingTime = f.getDepartureEvent().getTime() - f.getArrivalEvent().getTime();*/
        
        float delayTolerance = f.getServiceInfo().getDelayTolerance();
        double flowAge = TrafficGenerator.disasterArrival[TrafficGenerator.globalCount] - f.getArrivalEvent().getTime();            
        double holdingTime = f.getDepartureEvent().getTime() - f.getArrivalEvent().getTime();
        double totalBWReq = holdingTime*f.getBwReq();
        double remainingBw = totalBWReq - f.getTransmittedBw();
        double minimumHoldingTime = (remainingBw)/(f.getBwReq()*f.calcDegradation());     
        double residual = holdingTime - flowAge;
        
       
        Flow flow = f;       
        //System.out.println("Flow Age: " + flowAge + " HoldingTime: " + holdingTime + " Residual: " + residual);
        flow.setBwReq((int) (f.getBwReq() - f.getTransmittedBw()));
        flow.setIsDelayed(true);
        float time = (float) (SimulationRunner.timer + delayTolerance * residual);
        Event event = new DelayedFlowsArrival(f);
        event.setTime(time);
        //flow.setArrivalEvent((FlowArrivalEvent) event);
        Simulator.events.addEvent(event);

        event = new FlowDepartureEvent(f.getID(),flow);
        event.setTime(time + Integer.MAX_VALUE/* minimumHoldingTime*/);
        SimulationRunner.events.replaceEvent(f.getDepartureEvent(), event);
        flow.setDepartureEvent((FlowDepartureEvent) event);

    }

    @Override
    public void delayFlow(Flow f) {
        
        st.setNumDelay(st.getNumDelay()+1);
        LightPath[] lightpaths;
        if (activeFlows.containsKey(f.getID())&& mappedFlows.containsKey(f)) {            
            lightpaths = mappedFlows.get(f).getLightpaths();
            removeFlowFromPT(f, lightpaths);
            mappedFlows.remove(f);            
        }
        getDelayedFlows().add(f);
    }
    
    
}
