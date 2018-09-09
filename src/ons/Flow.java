/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ons;

import java.text.DecimalFormat;
import java.util.ArrayList;

/**
 * The Flow class defines an object that can be thought of as a flow
 * of data, going from a source node to a destination node. 
 * 
 * @author onsteam
 */
public class Flow {

    private long id;
    private int src;
    private int dst;
    private int bw;
    private int bwReq;
    private int duration;
    private int cos;
    private float maxRate;
    private int numWaves;
    private int numWavesInUse;    
    private ArrayList<Integer>[] paths;
    private FlowArrivalEvent arrivalEvent;
    private FlowDepartureEvent departureEvent;
    private ServiceInfo serviceInfo;
    private double droppedTraffic;
    private double transmittedBw;
    private float holdTime;
    private boolean isDelayed;
    private int modulation;    
    private ArrayList<EONLightPath> usedLps;
    private boolean dropped;
    private boolean interupted;
    private boolean degraded;
    

    /**
     * Creates a new Flow object.
     * 
     * @param id            unique identifier
     * @param src           source node
     * @param dst           destination node
     * @param bw            bandwidth (Mbps)
     * @param bwReq         bandwidth required (Mbps)
     * @param duration      duration time (seconds)
     * @param cos           class of service
     */
    public Flow(long id, int src, int dst, int bw, int duration, int cos) {
        if (id < 0 || src < 0 || dst < 0 || bw < 1 || duration < 0 || cos < 0) {
            throw (new IllegalArgumentException());
        } else {
            this.id = id;
            this.src = src;
            this.dst = dst;
            this.bwReq = bw;
            this.bw = bw;
            this.duration = duration;            
            this.cos = cos;
            paths = null;
            numWaves = numWavesInUse = getNumWave();
            droppedTraffic=0;
            transmittedBw=0;
            holdTime=0;
            isDelayed = false;
            modulation=0;
            usedLps = new ArrayList<EONLightPath>();
            dropped=false;
            interupted=false;
            degraded=false;
        }
    }
    public Flow(long id, int src, int dst, int bw ,float maxRate, int cos, ServiceInfo sInf) {
        if (id < 0 || src < 0 || dst < 0 || bw < 1 || maxRate < 0 || cos < 0) {
            throw (new IllegalArgumentException());
        } else {
            this.id = id;
            this.src = src;
            this.dst = dst;
            this.bwReq = bw;
            this.bw = (int)maxRate;
            this.maxRate = maxRate;           
            this.cos = cos;
            paths = null;
            numWaves = numWavesInUse = getNumWave();
            this.serviceInfo = sInf;
            droppedTraffic=0;
            transmittedBw=0;
            holdTime=0;
            isDelayed = false;
            modulation=0;
            usedLps = new ArrayList<EONLightPath>();
            dropped=false;
            interupted=false;
            degraded=false;
        }
    }  
    
    public Flow(long id, int src, int dst, int bw,int duration ,float maxRate, int cos, ServiceInfo sInf) {
        if (id < 0 || src < 0 || dst < 0 || bw < 1 || maxRate < 0 || cos < 0) {
            throw (new IllegalArgumentException());
        } else {
            this.id = id;
            this.src = src;
            this.dst = dst;
            this.bwReq = bw;
            this.bw = (int)maxRate;
            this.maxRate = maxRate;           
            this.cos = cos;
            this.duration = duration;
            paths = null;
            numWaves = numWavesInUse = getNumWave();
            this.serviceInfo = sInf;
            droppedTraffic=0;
            transmittedBw=0;
            holdTime=0;
            isDelayed = false;
            modulation=0;
            usedLps = new ArrayList<EONLightPath>();
            dropped=false;
            interupted=false;
            degraded=false;
        }
    } 
    
    public double getDroppedTraffic(){
        return droppedTraffic;
    }
    
    
    
    public void dropFlow(){
        //System.out.println();   
        
        DecimalFormat df = new DecimalFormat("0.00000000");        
        double flowAge = SimulationRunner.timer - arrivalEvent.getTime();
        double flowTimer = (departureEvent.getTime() - arrivalEvent.getTime());      
        
        /*System.out.println("Flow age: " + df.format(flowAge));
        System.out.println("Flow inicio: " + df.format(arrivalEvent.getTime()));
        System.out.println("Flow fim: " + df.format(departureEvent.getTime()));
        System.out.println("Flow timer: " + flowTimer);*/
        holdTime = SimulationRunner.timer;
        droppedTraffic = this.getBwReq() -  flowAge*bwReq/flowTimer;          
        
    }
    
    public void updateTransmittedBw(){
        
        transmittedBw = 0;
        droppedTraffic = 0;
        DecimalFormat df = new DecimalFormat("0.00000000");        
        double flowAge = SimulationRunner.timer - arrivalEvent.getTime();
        double flowTimer = (departureEvent.getTime() - arrivalEvent.getTime());
        holdTime = SimulationRunner.timer;
        transmittedBw = flowAge*bwReq/flowTimer;
        droppedTraffic = this.getBwReq() -  flowAge*bwReq/flowTimer;  
        
    }
    
    /**
     * Retrieves the unique identifier for a given Flow.
     * 
     * @return the value of the Flow's id attribute
     */
    public long getID() {
        return id;
    }
    
    /**
     * Retrieves the source node for a given Flow.
     * 
     * @return the value of the Flow's src attribute
     */
    public int getSource() {
        return src;
    }
    
    
    /**
     * 
     * @param porc Degrade % 
     */
    public void upgradeFlow(int waveUnit){
        
        int wavesAfter = waveUnit + numWavesInUse;
        //System.out.println("Antes: " + numWavesInUse+ " Depois: " + wavesAfter);
        if(wavesAfter<=numWaves){
            
            numWavesInUse = wavesAfter;              
            
        }else{
            
            numWavesInUse = numWaves;  
            
        }                
        
    }
    
    public void upgradeRateInt(int bwUp){
        
        bw = bwUp;               
        
    }
    
    /**
     * 
     * @param porc Degrade % 
     */
    public void degradeFlow(int waveUnit){
        
        int wavesAfter = numWavesInUse - waveUnit;
        if(wavesAfter>=0){
            
            numWavesInUse = wavesAfter;              
            
        }else{
            
            numWavesInUse = 0;  
            
        } 
        
    }  
    
    public float calcDegradation(){
             
        float degr = (float)numWavesInUse/(float)numWaves;        
        return degr;
        
    }
    
    public int getBwPerWave(){
        int num = getNumWavesInUse();
        if(num>0)
            return (int)((int)bw/getNumWavesInUse());
        return 0;
        
    }
    
    public int getTotalBwReq(){
        
        return bwReq*duration;   
        
    }
    
    public int getNumWave(){
        
        return (int) (bwReq/getMaxRate());
        
    }
    
    /**
     * Retrieves the destination node for a given Flow.
     * 
     * @return the value of the Flow's dst attribute
     */
    public int getDestination() {
        return dst;
    }
    
    /**
     * Retrieves the required bandwidth for a given Flow.
     * 
     * @return the value of the Flow's bw attribute.
     */
    public int getRate() {
        return bw;
    }
    
    /**
     * Assigns a new value to the required bandwidth of a given Flow.
     * 
     * @param bw new required bandwidth 
     */
    public void setRate(int bw) {
        this.bw = bw;
    }
    
    /**
     * Retrieves the duration time, in seconds, of a given Flow.
     * 
     * @return the value of the Flow's duration attribute
     */
    public int getDuration() {
        return duration;
    }
    
    /**
     * Retrieves a given Flow's "class of service".
     * A "class of service" groups together similar types of traffic
     * (for example, email, streaming video, voice,...) and treats
     * each type with its own level of service priority.
     * 
     * @return the value of the Flow's cos attribute
     */
    public int getCOS() {
        return cos;
    }
    
    /**
     * Prints all information related to a given Flow.
     * 
     * @return string containing all the values of the flow's parameters
     */
    @Override
    public String toString() {
        String flow = Long.toString(id) + ": " + Integer.toString(src) + "->" + Integer.toString(dst) + " rate: " + Integer.toString(bw) + " duration: " + Integer.toString(duration) + " cos: " + Integer.toString(cos);
        return flow;
    }
    
    /**
     * Creates a string with relevant information about the flow, to be
     * printed on the Trace file.
     * 
     * @return string with values of the flow's parameters
     */
    public String toTrace()
    {
    	String trace = Long.toString(id) + " " + Integer.toString(src) + " " + Integer.toString(dst) + " " + Integer.toString(bw) + " " + Integer.toString(duration) + " " + Integer.toString(cos);
    	return trace;
    }

    /**
     * @return the bwReq
     */
    public int getBwReq() {
        return bwReq;
    }

    /**
     * @param bwReq the bwReq to set
     */
    public void setBwReq(int bwReq) {
        this.bwReq = bwReq;
    }

    /**
     * @return the paths
     */
    public ArrayList<Integer>[] getPaths() {
        return paths;
    }

    /**
     * @param paths the paths to set
     */
    public void setPaths(ArrayList<Integer>[] paths) {
        this.paths = paths;
    }

    /**
     * @return the numWavesInUse
     */
    public int getNumWavesInUse() {
        return numWavesInUse;
    }

    /**
     * @param numWavesInUse the numWavesInUse to set
     */
    public void setNumWavesInUse(int numWavesInUse) {
        this.numWavesInUse = numWavesInUse;
    }

    /**
     * @return the arrivalEvent
     */
    public FlowArrivalEvent getArrivalEvent() {
        return arrivalEvent;
    }

    /**
     * @param arrivalEvent the arrivalEvent to set
     */
    public void setArrivalEvent(FlowArrivalEvent arrivalEvent) {
        this.arrivalEvent = arrivalEvent;
    }

    /**
     * @return the departureEvent
     */
    public FlowDepartureEvent getDepartureEvent() {
        return departureEvent;
    }

    /**
     * @param departureEvent the departureEvent to set
     */
    public void setDepartureEvent(FlowDepartureEvent departureEvent) {
        this.departureEvent = departureEvent;
    }
    
    public void updateHoldingTime(){
        
        //double ht = departureEvent.getTime() - arrivalEvent.getTime();
        //System.out.println("Duracao: " + duration + " Ht: " + ht);
        
    }

    /**
     * @return the maxRate
     */
    public float getMaxRate() {
        return maxRate;
    }

    /**
     * @param maxRate the maxRate to set
     */
    public void setMaxRate(float maxRate) {
        this.maxRate = maxRate;
    }
    
    public boolean isDegradeTolerant(){
        return (this.getServiceInfo().getDegradationTolerance()>0);
    }
    
    public boolean isDelayTolerant(){
        return ((int)this.getServiceInfo().getDelayTolerance()*10>0);
    }

    /**
     * @return the serviceInfo
     */
    public ServiceInfo getServiceInfo() {
        return serviceInfo;
    }

    /**
     * @param serviceInfo the serviceInfo to set
     */
    public void setServiceInfo(ServiceInfo serviceInfo) {
        this.serviceInfo = serviceInfo;
    }
    
    public int getMaxDegradationNumber(){
        return (int) ((this.bwReq*this.serviceInfo.getDegradationTolerance()/10)/this.maxRate);
    }
    
    public int getMaxDegradationNumberEon(){
        
        int requiredSlots = Modulation.convertRateToSlot((int) (this.serviceInfo.getDegradationTolerance()), EONPhysicalTopology.getSlotSize(), this.modulation);   
        int reqSlotsOriginal = Modulation.convertRateToSlot(this.getBwReq() , EONPhysicalTopology.getSlotSize(), this.modulation);
        if(requiredSlots>=reqSlotsOriginal)
            requiredSlots = reqSlotsOriginal - 1;
        return requiredSlots;
    }
    
    public int getRequiredSlots(){
        
        int requiredSlots = Modulation.convertRateToSlot((int) (this.serviceInfo.getDegradationTolerance()), EONPhysicalTopology.getSlotSize(), this.modulation); 
        return requiredSlots;
    }

    /**
     * @return the transmittedBw
     */
    public double getTransmittedBw() {
        return transmittedBw;
    }

    /**
     * @param transmittedBw the transmittedBw to set
     */
    public void setTransmittedBw(double transmittedBw) {
        this.transmittedBw = transmittedBw;
    }

    /**
     * @return the holdTime
     */
    public float getHoldTime() {
        return holdTime;
    }

    /**
     * @param holdTime the holdTime to set
     */
    public void setHoldTime(float holdTime) {
        this.holdTime = holdTime;
    }

    /**
     * @return the isDelayed
     */
    public boolean isIsDelayed() {
        return isDelayed;
    }

    /**
     * @param isDelayed the isDelayed to set
     */
    public void setIsDelayed(boolean isDelayed) {
        this.isDelayed = isDelayed;
    }

    /**
     * @return the modulation
     */
    public int getModulation() {
        return modulation;
    }

    /**
     * @param modulation the modulation to set
     */
    public void setModulation(int modulation) {
        this.modulation = modulation;
    }

    /**
     * @return the usedLps
     */
    public ArrayList<EONLightPath> getUsedLps() {
        return usedLps;
    }

    /**
     * @param usedLps the usedLps to set
     */
    public void setUsedLps(ArrayList<EONLightPath> usedLps) {
        this.usedLps = usedLps;
    }

    /**
     * @return the dropped
     */
    public boolean isDropped() {
        return dropped;
    }

    /**
     * @param dropped the dropped to set
     */
    public void setDropped(boolean dropped) {
        this.dropped = dropped;
    }

    /**
     * @return the interupted
     */
    public boolean isInterupted() {
        return interupted;
    }

    /**
     * @param interupted the interupted to set
     */
    public void setInterupted(boolean interupted) {
        this.interupted = interupted;
    }
}
