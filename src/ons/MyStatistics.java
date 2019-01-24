package ons;

import java.text.DecimalFormat;

/**
 * This class calculates and print all simulator statistics.
 * @author onsteam
 */
public class MyStatistics {

    private static MyStatistics singletonObject;
    private int minNumberArrivals;
    private int numberArrivals;
    private int arrivals;
    private int departures;
    private int accepted;
    private int blocked;
    private long requiredBandwidth;
    private long blockedBandwidth;
    private int numNodes;
    private int[][] arrivalsPairs;
    private int[][] blockedPairs;
    private long[][] requiredBandwidthPairs;
    private long[][] blockedBandwidthPairs;
    private int numfails;
    private int flowfails;
    private int lpsfails;
    private float trafficfails;
    private long execTime;
    //for Number of transmiters
    private long numLightPaths;
    private long numTransponders = 0;
    private int MAX_NumTransponders;
    private long usedTransponders = 0;
    //for available slots
    private long times = 0;
    private long availableSlots;
    private boolean firstTime = false;
    private int MAX_AvailableSlots;
    //for virtual hops per request
    private long virtualHops = 0;
    //for physical hops per request
    private long physicalHops = 0;
    //for modulations requests
    private long[] modulations;
    //for verbose counter
    private int verboseCount = 1;
    //Diffs
    private int numClasses;
    private int[] arrivalsDiff;
    private int[] blockedDiff;
    private long[] requiredBandwidthDiff;
    private long[] blockedBandwidthDiff;
    private int[][][] arrivalsPairsDiff;
    private int[][][] blockedPairsDiff;
    private int[][][] requiredBandwidthPairsDiff;
    private int[][][] blockedBandwidthPairsDiff;
    private String bpToString;
    private int droppedFlows;
    private int restoredFlows;
    private float dropRate;
    private float restoreRate;
    private double droppedTraffic;
    private double disasterEventBWReq;
    private int fullServiceDroppedFlows;
    private int fullServiceFlows;
    private float fullServiceDropRate;
    
    private int delayTolerantDroppedFlows;
    private int delayTolerantFlows;    
    private float delayTolerantDropRate;
    
    private int degradedTolerantDroppedFlows;
    private int degradedTolerantFlows;
    private float degradedTolerantDropRate;
    
    private int degradedServiceDroppedFlows;
    private int degradedServiceFlows;
    private float degradedServiceDropRate;
    private int delayedFlows;
    private float waitMean;
    private float minDegr;
    private float maxDegr;
    private int numDegr;
    private int numDelay;   
    private float fairness;
    private float disruptionPeriod;
    private float disruptionPeriodRate;
    private int numFlowsDelayedRestored;
        

    
    
    /**
     * A private constructor that prevents any other class from instantiating.
     */
    private MyStatistics() {

        numberArrivals = 0;
        numFlowsDelayedRestored = 0;
        disruptionPeriodRate = 0;
        disruptionPeriod = 0;
        arrivals = 0;
        departures = 0;
        accepted = 0;
        blocked = 0;
        numDegr= 0 ;
        numDelay = 0;

        requiredBandwidth = 0;
        blockedBandwidth = 0;

        numfails = 0;
        flowfails = 0;
        lpsfails = 0;
        trafficfails = 0;

        execTime = 0;
        //add by lucasrc
        numLightPaths = 0;
        disasterEventBWReq = 0;
        fullServiceDroppedFlows = 0;
        fullServiceDropRate = 0;
        
        delayTolerantDroppedFlows = 0;
        delayTolerantFlows = 0;    
        delayTolerantDropRate = 0;
    
        degradedTolerantDroppedFlows = 0;
        degradedTolerantFlows = 0;
        degradedTolerantDropRate = 0;
    
        degradedServiceDroppedFlows = 0;
        degradedServiceFlows = 0;
        degradedServiceDropRate = 0;
        delayedFlows = 0;
        minDegr = 1;
    }

    /**
     * Creates a new MyStatistics object, in case it does'n exist yet.
     *
     * @return the MyStatistics singletonObject
     */
    public static synchronized MyStatistics getMyStatisticsObject() {
        if (singletonObject == null) {
            singletonObject = new MyStatistics();
        }
        return singletonObject;
    }

    /**
     * Throws an exception to stop a cloned MyStatistics object from being
     * created.
     */
    @Override
    public Object clone() throws CloneNotSupportedException {
        throw new CloneNotSupportedException();
    }
    
    
        /**
     * Attributes initializer.
     *
     * @param pt PhysicalTopology Object
     * @param numClasses number of classes of service
     * @param minNumberArrivals minimum number of arriving events
     */
    public void statisticsSetup(PhysicalTopology pt, int numClasses, int minNumberArrivals) {
        this.numNodes = pt.getNumNodes();
        this.arrivalsPairs = new int[numNodes][numNodes];
        this.blockedPairs = new int[numNodes][numNodes];
        this.requiredBandwidthPairs = new long[numNodes][numNodes];
        this.blockedBandwidthPairs = new long[numNodes][numNodes];

        this.minNumberArrivals = minNumberArrivals;

        //Diff
        this.numClasses = numClasses;
        this.arrivalsDiff = new int[numClasses];
        this.blockedDiff = new int[numClasses];
        this.requiredBandwidthDiff = new long[numClasses];
        this.blockedBandwidthDiff = new long[numClasses];
        for (int i = 0; i < numClasses; i++) {
            this.arrivalsDiff[i] = 0;
            this.blockedDiff[i] = 0;
            this.requiredBandwidthDiff[i] = 0;
            this.blockedBandwidthDiff[i] = 0;
        }
        this.arrivalsPairsDiff = new int[numNodes][numNodes][numClasses];
        this.blockedPairsDiff = new int[numNodes][numNodes][numClasses];
        this.requiredBandwidthPairsDiff = new int[numNodes][numNodes][numClasses];
        this.blockedBandwidthPairsDiff = new int[numNodes][numNodes][numClasses];
        //
        if(pt instanceof EONPhysicalTopology) {
            this.modulations = new long[EONPhysicalTopology.getMaxModulation() + 1];
        } 
    }

    /**
     * Adds an accepted flow to the statistics.
     *
     * @param flow the accepted Flow object
     * @param lightpaths list of lightpaths in the flow
     */
    public void acceptFlow(Flow flow, LightPath[] lightpaths) {
        if (this.numberArrivals > this.minNumberArrivals) {
            this.accepted++;
            this.virtualHops(lightpaths.length);
            int count = 0;
            for (LightPath lps : lightpaths) {
                count += lps.getHops();
            }
            this.physicalHops(count);
        }
    }
    
    /**
     * Adds a blocked flow to the statistics.
     *
     * @param flow the blocked Flow object
     */
    public void blockFlow(Flow flow) {
        if (this.numberArrivals > this.minNumberArrivals) {
            int cos = flow.getCOS();
            this.blocked++;
            this.blockedDiff[cos]++;
            this.blockedBandwidth += flow.getBwReq();
            this.blockedBandwidthDiff[cos] += flow.getBwReq();
            this.blockedPairs[flow.getSource()][flow.getDestination()]++;
            this.blockedPairsDiff[flow.getSource()][flow.getDestination()][cos]++;
            this.blockedBandwidthPairs[flow.getSource()][flow.getDestination()] += flow.getBwReq();
            this.blockedBandwidthPairsDiff[flow.getSource()][flow.getDestination()][cos] += flow.getBwReq();
        }
    }
    
    /**
     * Adds an event to the statistics.
     * @param event the Event object to be added
     * @param availableSlots the atual available slots in physical topology
     * @param availableTransponders the atual available transponders in physical topology
     */
    public void addEvent(Event event, int availableSlots, int availableTransponders) {
        if(!firstTime){
            MAX_AvailableSlots = availableSlots;
            MAX_NumTransponders = availableTransponders;
            firstTime = true;
        }
        addEvent(event, availableTransponders);
        if (this.numberArrivals > this.minNumberArrivals) {
            this.availableSlots += (long) availableSlots;
        }
    }

    /**
     * Adds an event to the statistics.
     *
     * @param event the Event object to be added
     * @param availableTransponders the atual available transponders in physical topology
     */
    public void addEvent(Event event, int availableTransponders) {
        try {
            if (!firstTime) {
                MAX_NumTransponders = availableTransponders;
                firstTime = true;
            }
            times++;
            if (event instanceof FlowArrivalEvent) {
                this.numberArrivals++;
                if (this.numberArrivals > this.minNumberArrivals) {
                    this.numTransponders += (long) availableTransponders;
                    int cos = ((FlowArrivalEvent) event).getFlow().getCOS();
                    this.arrivals++;
                    this.arrivalsDiff[cos]++;
                    this.requiredBandwidth += ((FlowArrivalEvent) event).getFlow().getBwReq();
                    this.requiredBandwidthDiff[cos] += ((FlowArrivalEvent) event).getFlow().getBwReq();
                    this.arrivalsPairs[((FlowArrivalEvent) event).getFlow().getSource()][((FlowArrivalEvent) event).getFlow().getDestination()]++;
                    this.arrivalsPairsDiff[((FlowArrivalEvent) event).getFlow().getSource()][((FlowArrivalEvent) event).getFlow().getDestination()][cos]++;
                    this.requiredBandwidthPairs[((FlowArrivalEvent) event).getFlow().getSource()][((FlowArrivalEvent) event).getFlow().getDestination()] += ((FlowArrivalEvent) event).getFlow().getBwReq();
                    this.requiredBandwidthPairsDiff[((FlowArrivalEvent) event).getFlow().getSource()][((FlowArrivalEvent) event).getFlow().getDestination()][cos] += ((FlowArrivalEvent) event).getFlow().getBwReq();
                }
                //to print the current progress calls
                if (Simulator.verbose && (numberArrivals ==  10000*verboseCount)) {
                    System.out.println(Integer.toString(numberArrivals));
                    verboseCount++;
                }
            } else if (event instanceof FlowDepartureEvent) {
                if (this.numberArrivals > this.minNumberArrivals) {
                    this.departures++;
                }
            }
        } catch (Exception e) {
            System.out.println(e.getMessage());
            e.printStackTrace();
        }
    }

    /**
     * This function is called during the simulation execution, but only if
     * verbose was activated.
     *
     * @param simType 0 if the physicalTopology is WDM; 1 if physicalTopology is EON
     * @return string with the obtained statistics
     */
    public String fancyStatistics(int simType) {
        float acceptProb, blockProb, bbr, meanK;
        float bpDiff[], bbrDiff[];
        if (accepted == 0) {
            acceptProb = 0;
            meanK = 0;
        } else {
            acceptProb = ((float) accepted) / ((float) arrivals) * 100;
        }
        if (blocked == 0) {
            blockProb = 0;
            bbr = 0;
        } else {
            blockProb = ((float) blocked) / ((float) arrivals) * 100;
            bbr = ((float) blockedBandwidth) / ((float) requiredBandwidth) * 100;
        }
        bpDiff = new float[numClasses];
        bbrDiff = new float[numClasses];
        for (int i = 0; i < numClasses; i++) {
            if (blockedDiff[i] == 0) {
                bpDiff[i] = 0;
                bbrDiff[i] = 0;
            } else {
                bpDiff[i] = ((float) blockedDiff[i]) / ((float) arrivalsDiff[i]) * 100;
                bbrDiff[i] = ((float) blockedBandwidthDiff[i]) / ((float) requiredBandwidthDiff[i]) * 100;
            }
        }

        String stats = "Arrivals \t: " + Integer.toString(arrivals) + "\n";
        stats += "Departures \t: " + Integer.toString(departures) + "\n";
        stats += "Accepted \t: " + Integer.toString(accepted) + "\t(" + Float.toString(acceptProb) + "%)\n";
        stats += "Blocked \t: " + Integer.toString(blocked) + "\t(" + Float.toString(blockProb) + "%)\n";
        stats += "Required BW \t: " + Long.toString(requiredBandwidth) + "\n";
        stats += "Blocked BW \t: " + Long.toString(blockedBandwidth) + "\n";
        stats += "BBR      \t: " + Float.toString(bbr) + "%\n";
        stats += "Called Blocked by COS (%)" + "\n";
        for (int i = 0; i < numClasses; i++) {
            stats += "BP-" + Integer.toString(i) + " " + Float.toString(bpDiff[i]) + "%\n";
        }
        stats += "\nNumber of LPs: " + numLightPaths + "\n";
        double freeTransponders = (float) numTransponders/times; //free transponders/times-requests
        double freeTranspondersRatio = (float) ((freeTransponders*100.0)/MAX_NumTransponders);
        int nodes = blockedPairs[0].length;
        stats += "Average of free Transponders by node: " +(double) freeTransponders/nodes + " ("+MAX_NumTransponders/nodes+")\n";
        stats += "Available Transponders ratio: " + freeTranspondersRatio + "%\n";
        double used = (double) this.usedTransponders / (double) accepted;
        stats += "Average of Transponders per request: "+used+"\n";
        used = (double) this.virtualHops / (double) accepted;
        stats += "Average of Virtual Hops per request: "+used+"\n";
        used = (double) this.physicalHops / (double) accepted;
        stats += "Average of Physical Hops per request: "+used+"\n";


        
        if(simType == 1){
            double averageSpectrumAvailable = availableSlots/times;
            double spectrumAvailableRatio = (averageSpectrumAvailable*100.0)/MAX_AvailableSlots;
            stats += "Spectrum Available ratio: " + spectrumAvailableRatio + "%\n";
            for(int i = 0; i < modulations.length; i++){
                stats += Modulation.getModulationName(i) +" Modulation used: " + Float.toString((float) modulations[i]/(float) numLightPaths*100) + "%\n";
            }
        }
        stats += "\n";
        stats += "Blocking probability per s-d node-pair:\n";
        for (int i = 0; i < numNodes; i++) {
            for (int j = 0; j < numNodes; j++) {
                if (i != j) {
                    stats += "Pair (" + Integer.toString(i) + "->" + Integer.toString(j) + ") ";
                    stats += "Calls (" + Integer.toString(arrivalsPairs[i][j]) + ")";
                    if (blockedPairs[i][j] == 0) {
                        blockProb = 0;
                        bbr = 0;
                    } else {
                        blockProb = ((float) blockedPairs[i][j]) / ((float) arrivalsPairs[i][j]) * 100;
                        bbr = ((float) blockedBandwidthPairs[i][j]) / ((float) requiredBandwidthPairs[i][j]) * 100;
                    }
                    stats += "\tBP (" + Float.toString(blockProb) + "%)";
                    stats += "\tBBR (" + Float.toString(bbr) + "%)\n";
                }
            }
        }
        return stats;
    }
    
    /**
     * Prints all the obtained statistics, but only if verbose was not
     * activated.
     * @param simType 0 if the physicalTopology is WDM; 1 if physicalTopology is EON
     */
    public void printStatistics(int simType) {
        float acceptProb, blockProb, bbr, meanK;
        float bpDiff[], bbrDiff[];
        if (accepted == 0) {
            acceptProb = 0;
            meanK = 0;
        } else {
            acceptProb = ((float) accepted) / ((float) arrivals) * 100;
        }
        if (blocked == 0) {
            blockProb = 0;
            bbr = 0;
        } else {
            blockProb = ((float) blocked) / ((float) arrivals) * 100;
            bbr = ((float) blockedBandwidth) / ((float) requiredBandwidth) * 100;
        }
        bpDiff = new float[numClasses];
        bbrDiff = new float[numClasses];
        for (int i = 0; i < numClasses; i++) {
            if (blockedDiff[i] == 0) {
                bpDiff[i] = 0;
                bbrDiff[i] = 0;
            } else {
                bpDiff[i] = ((float) blockedDiff[i]) / ((float) arrivalsDiff[i]) * 100;
                bbrDiff[i] = ((float) blockedBandwidthDiff[i]) / ((float) requiredBandwidthDiff[i]) * 100;
            }
        }
        
        System.out.println("Flows Dropadas: " + droppedFlows + " Restauradas: " + restoredFlows);
               
        String stats = "";
        stats += "BR \t: " + Float.toString(blockProb) + "%\n";
        DecimalFormat df = new DecimalFormat("0.00000000");        
        stats += "BBR \t: " + /*Double.toString(bbr)*/df.format(bbr) + "%\n";
        stats += "Called Blocked by COS (%)" + "\n";
        for (int i = 0; i < numClasses; i++) {
            stats += "BP-" + Integer.toString(i) + " " + Float.toString(bpDiff[i]) + "%\n";
            setBpToString(Float.toString(bpDiff[i]));
        }        
        
        stats += "Drop rate: "+this.dropRate+"\n";
        stats += "Dropped Flows: "+this.droppedFlows+"\n";
        if(fullServiceFlows>0){
            
            fullServiceDropRate = (float)this.fullServiceDroppedFlows/this.fullServiceFlows;
            
        }else{
            
            fullServiceDropRate = -1;
            
        }
        
        stats += "Full Service Drop Rate: " + this.fullServiceDropRate + "\n";
        
        if(getDelayTolerantFlows()>0){
            
            setDelayTolerantDropRate((float) this.getDelayTolerantDroppedFlows() / this.getDelayTolerantFlows());
            
        }else{
            
            setDelayTolerantDropRate(-1);
            
        }
        
        stats += "Delay Tolerant Service Drop Rate: " + this.getDelayTolerantDropRate() + "\n";
        
        if(getDegradedTolerantFlows()>0){
            
            setDegradedTolerantDropRate((float) this.getDegradedTolerantDroppedFlows() / this.getDegradedTolerantFlows());
            
        }else{
            
            setDegradedTolerantDropRate(-1);
            
        }
        
        stats += "Degradation Tolerant Service Drop Rate: " + this.getDegradedTolerantDropRate() + "\n";
        
        if(getDegradedServiceFlows()>0){
            
            setDegradedServiceDropRate((float) this.getDegradedServiceDroppedFlows() / this.getDegradedServiceFlows());
            
        }else{
            
            setDegradedServiceDropRate(-1);
            
        }
        
        stats += "Extreme Degradation Tolerant Service Drop Rate: " + this.getDegradedServiceDropRate() + "\n";
        
        stats += "\nLPs: " + numLightPaths + "\n";
        double freeTransponders = (float) numTransponders/times; //free transponders/times-requests
        double freeTranspondersRatio = (float) ((freeTransponders*100.0)/MAX_NumTransponders);     
        stats += "Available Transponders: " + freeTranspondersRatio + "%\n";
        double used = (double) this.usedTransponders / (double) accepted;
        stats += "Transponders per request: "+used+"\n";
        used = (double) this.virtualHops / (double) accepted;
        stats += "Virtual Hops per request: "+used+"\n";
        used = (double) this.physicalHops / (double) accepted;
        stats += "Physical Hops per request: "+used+"\n";
        
        if(simType == 1){
            double averageSpectrumAvailable = availableSlots/times;
            double spectrumAvailableRatio = (averageSpectrumAvailable*100.0)/MAX_AvailableSlots;


            stats += "Spectrum Available: " + spectrumAvailableRatio + "%\n";
            double frag_rede = (double) MAX_AvailableSlots / (double )availableSlots;
            System.out.println(MAX_AvailableSlots);
            System.out.println(averageSpectrumAvailable);
            stats += "Fragmentação da rede: " + minDegr + "%\n";
            for(int i = 0; i < modulations.length; i++){
                stats += Modulation.getModulationName(i) +" Modulation used: " + Float.toString((float) modulations[i]/(float) numLightPaths*100) + "%\n";
            }
        }
        System.out.println(stats);
    }

    /**
     * When a lightpath is allocated
     * @param lp the lightpath
     */
    public void createLightpath(LightPath lp) {
        this.numLightPaths++;
        if (lp instanceof EONLightPath) {
            this.modulations[((EONLightPath) lp).getModulation()]++;
        }
    }
    
    /**
     * When a lightpath is deallocated
     * @param lp the lightpath
     */
    public void deallocatedLightpath(LightPath lp) {
        this.numLightPaths--;
        if (lp instanceof EONLightPath) {
            this.modulations[((EONLightPath) lp).getModulation()]--;
        }
    }
    
    /**
     * When a transponder is allocated
     * @param usedTransponders 
     */
    public void userTransponder(int usedTransponders) {
        this.usedTransponders += (long) usedTransponders;
    }
    
    /**
     * When a new virtual hop occurs
     * @param virtualHops 
     */
    public void virtualHops(int virtualHops) {
        this.virtualHops += (long) virtualHops;
    }
    
    /**
     * When a new physical hop occurs
     * @param physicalHops 
     */
    public void physicalHops(int physicalHops) {
        this.physicalHops += (long) physicalHops;
    }
    
    /**
     * Terminates the singleton object.
     */
    public void finish() {
        singletonObject = null;
    }

    /**
     * @return the bpToString
     */
    public String getBpToString() {
        return bpToString;
    }
    
    /**
     * @return the bpToString
     */
    public String getBBRToString() {        
        float bbr = 0;
        
        if (blocked == 0) {            
            bbr = 0;
        } else {            
            bbr = ((float) blockedBandwidth) / ((float) requiredBandwidth) * 100;
        }
        DecimalFormat df = new DecimalFormat("0.00000000");  
        return String.valueOf(df.format(bbr));
    }

    /**
     * @param bpToString the bpToString to set
     */
    public void setBpToString(String bpToString) {
        this.bpToString = bpToString;
    }

    /**
     * @return the droppedFlows
     */
    public int getDroppedFlows() {
        return droppedFlows;
    }

    /**
     * @param droppedFlows the droppedFlows to set
     */
    public void setDroppedFlows(int droppedFlows) {
        this.droppedFlows = droppedFlows;
    }

    /**
     * @return the restoredFlows
     */
    public int getRestoredFlows() {
        return restoredFlows;
    }

    /**
     * @param restoredFlows the restoredFlows to set
     */
    public void setRestoredFlows(int restoredFlows) {
        this.restoredFlows = restoredFlows;
    }

    /**
     * @return the dropRate
     */
    public float getDropRate() {
        return dropRate;
    }

    /**
     * @param dropRate the dropRate to set
     */
    public void setDropRate(float dropRate) {
        this.dropRate = dropRate;
    }

    /**
     * @return the restoreRate
     */
    public float getRestoreRate() {
        return restoreRate;
    }

    /**
     * @param restoreRate the restoreRate to set
     */
    public void setRestoreRate(float restoreRate) {
        this.restoreRate = restoreRate;
    }

    /**
     * @return the droppedTraffic
     */
    public double getDroppedTraffic() {
        return droppedTraffic;
    }

    /**
     * @param droppedTraffic the droppedTraffic to set
     */
    public void setDroppedTraffic(double droppedTraffic) {
        this.droppedTraffic = droppedTraffic;
    }
    
    public double getDroppedTrafficRate(){
                
        return this.droppedTraffic/this.disasterEventBWReq;
        
    }

    /**
     * @return the disasterEventBWReq
     */
    public double getDisasterEventBWReq() {
        return disasterEventBWReq;
    }

    /**
     * @param disasterEventBWReq the disasterEventBWReq to set
     */
    public void setDisasterEventBWReq(double disasterEventBWReq) {
        this.disasterEventBWReq = disasterEventBWReq;
    }

    /**
     * @return the fullServiceDropRate
     */
    public float getFullServiceDropRate() {
        return fullServiceDropRate;
    }

    /**
     * @param fullServiceDropRate the fullServiceDropRate to set
     */
    public void setFullServiceDropRate(float fullServiceDropRate) {
        this.fullServiceDropRate = fullServiceDropRate;
    }

    /**
     * @return the fullServiceDroppedFlows
     */
    public int getFullServiceDroppedFlows() {
        return fullServiceDroppedFlows;
    }

    /**
     * @param fullServiceDroppedFlows the fullServiceDroppedFlows to set
     */
    public void setFullServiceDroppedFlows(int fullServiceDroppedFlows) {
        this.fullServiceDroppedFlows = fullServiceDroppedFlows;
    }

    /**
     * @return the fullServiceFlows
     */
    public int getFullServiceFlows() {
        return fullServiceFlows;
    }

    /**
     * @param fullServiceFlows the fullServiceFlows to set
     */
    public void setFullServiceFlows(int fullServiceFlows) {
        this.fullServiceFlows = fullServiceFlows;
    }

    /**
     * @return the delayTolerantDroppedFlows
     */
    public int getDelayTolerantDroppedFlows() {
        return delayTolerantDroppedFlows;
    }

    /**
     * @param delayTolerantDroppedFlows the delayTolerantDroppedFlows to set
     */
    public void setDelayTolerantDroppedFlows(int delayTolerantDroppedFlows) {
        this.delayTolerantDroppedFlows = delayTolerantDroppedFlows;
    }

    /**
     * @return the delayTolerantFlows
     */
    public int getDelayTolerantFlows() {
        return delayTolerantFlows;
    }

    /**
     * @param delayTolerantFlows the delayTolerantFlows to set
     */
    public void setDelayTolerantFlows(int delayTolerantFlows) {
        this.delayTolerantFlows = delayTolerantFlows;
    }

    /**
     * @return the delayTolerantDropRate
     */
    public float getDelayTolerantDropRate() {
        return delayTolerantDropRate;
    }

    /**
     * @param delayTolerantDropRate the delayTolerantDropRate to set
     */
    public void setDelayTolerantDropRate(float delayTolerantDropRate) {
        this.delayTolerantDropRate = delayTolerantDropRate;
    }

    /**
     * @return the degradedTolerantDroppedFlows
     */
    public int getDegradedTolerantDroppedFlows() {
        return degradedTolerantDroppedFlows;
    }

    /**
     * @param degradedTolerantDroppedFlows the degradedTolerantDroppedFlows to set
     */
    public void setDegradedTolerantDroppedFlows(int degradedTolerantDroppedFlows) {
        this.degradedTolerantDroppedFlows = degradedTolerantDroppedFlows;
    }

    /**
     * @return the degradedTolerantFlows
     */
    public int getDegradedTolerantFlows() {
        return degradedTolerantFlows;
    }

    /**
     * @param degradedTolerantFlows the degradedTolerantFlows to set
     */
    public void setDegradedTolerantFlows(int degradedTolerantFlows) {
        this.degradedTolerantFlows = degradedTolerantFlows;
    }

    /**
     * @return the degradedTolerantDropRate
     */
    public float getDegradedTolerantDropRate() {
        return degradedTolerantDropRate;
    }

    /**
     * @param degradedTolerantDropRate the degradedTolerantDropRate to set
     */
    public void setDegradedTolerantDropRate(float degradedTolerantDropRate) {
        this.degradedTolerantDropRate = degradedTolerantDropRate;
    }

    /**
     * @return the degradedServiceDroppedFlows
     */
    public int getDegradedServiceDroppedFlows() {
        return degradedServiceDroppedFlows;
    }

    /**
     * @param degradedServiceDroppedFlows the degradedServiceDroppedFlows to set
     */
    public void setDegradedServiceDroppedFlows(int degradedServiceDroppedFlows) {
        this.degradedServiceDroppedFlows = degradedServiceDroppedFlows;
    }

    /**
     * @return the degradedServiceFlows
     */
    public int getDegradedServiceFlows() {
        return degradedServiceFlows;
    }

    /**
     * @param degradedServiceFlows the degradedServiceFlows to set
     */
    public void setDegradedServiceFlows(int degradedServiceFlows) {
        this.degradedServiceFlows = degradedServiceFlows;
    }

    /**
     * @return the degradedServiceDropRate
     */
    public float getDegradedServiceDropRate() {
        return degradedServiceDropRate;
    }

    /**
     * @param degradedServiceDropRate the degradedServiceDropRate to set
     */
    public void setDegradedServiceDropRate(float degradedServiceDropRate) {
        this.degradedServiceDropRate = degradedServiceDropRate;
    }

    /**
     * @return the delayedFlows
     */
    public int getDelayedFlows() {
        return delayedFlows;
    }

    /**
     * @param delayedFlows the delayedFlows to set
     */
    public void setDelayedFlows(int delayedFlows) {
        this.delayedFlows = delayedFlows;
    }

    /**
     * @return the waitMean
     */
    public float getWaitMean() {
        return waitMean;
    }

    /**
     * @param waitMean the waitMean to set
     */
    public void setWaitMean(float waitMean) {
        this.waitMean = waitMean;
    }

    /**
     * @return the numDegr
     */
    public int getNumDegr() {
        return numDegr;
    }

    /**
     * @param numDegr the numDegr to set
     */
    public void setNumDegr(int numDegr) {
        this.numDegr = numDegr;
    }

    /**
     * @return the numDelay
     */
    public int getNumDelay() {
        return numDelay;
    }

    /**
     * @param numDelay the numDelay to set
     */
    public void setNumDelay(int numDelay) {
        this.numDelay = numDelay;
    }

    /**
     * @return the minDegr
     */
    public float getMinDegr() {
        return minDegr;
    }

    /**
     * @param minDegr the minDegr to set
     */
    public void setMinDegr(float minDegr) {
        this.minDegr = minDegr;
    }

    /**
     * @return the fairness
     */
    public float getFairness() {
        return fairness;
    }

    /**
     * @param fairness the fairness to set
     */
    public void setFairness(float fairness) {
        this.fairness = fairness;
    }

    /**
     * @return the disruptionPeriod
     */
    public float getDisruptionPeriod() {
        return disruptionPeriod;
    }

    /**
     * @param disruptionPeriod the disruptionPeriod to set
     */
    public void setDisruptionPeriod(float disruptionPeriod) {
        this.disruptionPeriod = disruptionPeriod;
    }

    /**
     * @return the disruptionPeriodRate
     */
    public float getDisruptionPeriodRate() {
        return disruptionPeriodRate;
    }

    /**
     * @param disruptionPeriodRate the disruptionPeriodRate to set
     */
    public void setDisruptionPeriodRate(float disruptionPeriodRate) {
        this.disruptionPeriodRate = disruptionPeriodRate;
    }

    /**
     * @return the numFlowsDelayedRestored
     */
    public int getNumFlowsDelayedRestored() {
        return numFlowsDelayedRestored;
    }

    /**
     * @param numFlowsDelayedRestored the numFlowsDelayedRestored to set
     */
    public void setNumFlowsDelayedRestored(int numFlowsDelayedRestored) {
        this.numFlowsDelayedRestored = numFlowsDelayedRestored;
    }
    
}
