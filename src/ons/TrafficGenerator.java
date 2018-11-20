/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ons;

import static java.lang.Integer.max;
import static java.lang.Integer.min;
import java.util.ArrayList;
import java.util.concurrent.ThreadLocalRandom;
import ons.util.DisasterGenerator;
import ons.util.Distribution;
import org.w3c.dom.*;

/**
 * Generates the network's traffic based on the information passed through the
 * command line arguments and the XML simulation file.
 * 
 * @author onsteam
 */
public class TrafficGenerator {

    private int calls;
    private double load;
    private int maxRate;
    public static TrafficInfo[] callsTypesInfo;
    private double meanRate;
    private double meanHoldingTime;
    private int TotalWeight;
    private int numberCallsTypes;
    private int numberServicesTypes;
    private ServiceInfo servicesTypesInfo[];
    private int servicesTotalWeight;
    public static float[] disasterArrival;
    public static int globalCount;

    /**
     * Creates a new TrafficGenerator object.
     * Extracts the traffic information from the XML file and takes the chosen load and
     * seed from the command line arguments.
     * 
     * @param xml file that contains all information about the simulation
     * @param forcedLoad range of offered loads for several simulations
     */
    public TrafficGenerator(Element xml, Element xml2, double forcedLoad) {
        int rate, cos, weight;
        double holdingTime;
        Element xml_links = (Element)xml.getOwnerDocument().getElementsByTagName("links").item(0);
        globalCount = -1;
        calls = Integer.parseInt(xml.getAttribute("calls"));
        load = forcedLoad;
        if (load == 0) {
            load = Double.parseDouble(xml.getAttribute("load"));
        }
        if(xml.hasAttribute("max-rate")){
            maxRate = Integer.parseInt(xml.getAttribute("max-rate"));
        } else if(xml_links.hasAttribute("slot-size")){
            maxRate = Integer.parseInt(xml_links.getAttribute("slot-size"));
        }else{
            maxRate = 0;
        }

        if (Simulator.verbose) {
            System.out.println(xml.getAttribute("calls") + " calls, " + xml.getAttribute("load") + " erlangs.");
        }

        // Process calls
        NodeList callslist = xml.getElementsByTagName("calls");
        numberCallsTypes = callslist.getLength();
        if (Simulator.verbose) {
            System.out.println(Integer.toString(numberCallsTypes) + " type(s) of calls:");
        }

        callsTypesInfo = new TrafficInfo[numberCallsTypes];
         
        TotalWeight = 0;
        meanRate = 0;
        meanHoldingTime = 0;

        for (int i = 0; i < numberCallsTypes; i++) {
            TotalWeight += Integer.parseInt(((Element) callslist.item(i)).getAttribute("weight"));
        }

        for (int i = 0; i < numberCallsTypes; i++) {
            holdingTime = Double.parseDouble(((Element) callslist.item(i)).getAttribute("holding-time"));
            rate = Integer.parseInt(((Element) callslist.item(i)).getAttribute("rate"));
            cos = Integer.parseInt(((Element) callslist.item(i)).getAttribute("cos"));
            weight = Integer.parseInt(((Element) callslist.item(i)).getAttribute("weight"));
            meanRate += (double) rate * ((double) weight / (double) TotalWeight);
            meanHoldingTime += holdingTime * ((double) weight / (double) TotalWeight);
            callsTypesInfo[i] = new TrafficInfo(holdingTime, rate, cos, weight);
            if (Simulator.verbose) {
                System.out.println("#################################");
                System.out.println("Weight: " + Integer.toString(weight) + ".");
                System.out.println("COS: " + Integer.toString(cos) + ".");
                System.out.println("Rate: " + Integer.toString(rate) + "Mbps.");
                System.out.println("Mean holding time: " + Double.toString(holdingTime) + " seconds.");
            }            
        }        
       
        // Process Services
        NodeList servicesList = xml2.getElementsByTagName("service");
        numberServicesTypes = servicesList.getLength();
        if (Simulator.verbose) {
            System.out.println(Integer.toString(numberServicesTypes) + " type(s) of services:");
        }

        servicesTypesInfo = new ServiceInfo[numberServicesTypes];
         
        servicesTotalWeight = 0;     

        for (int i = 0; i < numberServicesTypes; i++) {
            servicesTotalWeight += Integer.parseInt(((Element) servicesList.item(i)).getAttribute("weight"));
        }      
        
        for (int i = 0; i < numberServicesTypes; i++) {
            int serviceClass = Integer.parseInt(((Element) servicesList.item(i)).getAttribute("class"));
            float degrTolMin = Float.parseFloat(((Element) servicesList.item(i)).getAttribute("degradation-tolerance-min"));
            float degrTolMax = Float.parseFloat(((Element) servicesList.item(i)).getAttribute("degradation-tolerance-max"));
            float delayTolMin = Float.parseFloat(((Element) servicesList.item(i)).getAttribute("delay-tolerance-min"));
            float delayTolMax = Float.parseFloat(((Element) servicesList.item(i)).getAttribute("delay-tolerance-max"));
            weight = Integer.parseInt(((Element) servicesList.item(i)).getAttribute("weight"));  
            float degrTol = 0;
            if(degrTolMax!=0)
                degrTol = (float) ThreadLocalRandom.current().nextDouble(degrTolMin, degrTolMax);
            float delayTol = 0;
            if(delayTolMax!=0)
                delayTol = (float) ThreadLocalRandom.current().nextDouble(delayTolMin, delayTolMax);
            servicesTypesInfo[i] = new ServiceInfo(serviceClass,degrTol,delayTol,weight);
            if (false) {
                System.out.println("#################################");
                System.out.println("Class: " + Integer.toString(serviceClass) + ".");
                System.out.println("Degradation: " + Float.toString(degrTol) + ".");
                System.out.println("Delay: " + Float.toString(delayTol));
                System.out.println("Weight" + Integer.toString(weight));
            }
        }
        
        
    }

    /**
     * Generates the network's traffic.
     *
     * @param events EventScheduler object that will contain the simulation events
     * @param pt the network's Physical Topology
     * @param seed a number in the interval [1,25] that defines up to 25 different random simulations
     */
    public void generateTraffic(Element xml,PhysicalTopology pt, EventScheduler events, int seed) {

        // Compute the weight vector
        int[] weightVector = new int[TotalWeight];
        int aux = 0;
        for (int i = 0; i < numberCallsTypes; i++) {
            for (int j = 0; j < callsTypesInfo[i].getWeight(); j++) {
                weightVector[aux] = i;
                aux++;
            }
        }
        
        int[] weightVectorService = new int[servicesTotalWeight];
        int aux2 = 0;
        for (int i = 0; i < numberServicesTypes; i++) {
            for (int j = 0; j < servicesTypesInfo[i].getWeight(); j++) {
                weightVectorService[aux2] = i;
                aux2++;
            }
        }
        
        /* Compute the arrival time
         *
         * load = meanArrivalRate x holdingTime x bw/maxRate
         * 1/meanArrivalRate = (holdingTime x bw/maxRate)/load
         * meanArrivalTime = (holdingTime x bw/maxRate)/load
         */
        double meanArrivalTime;
        if (pt instanceof EONPhysicalTopology){
            //Because the EON architecture is not possible to obtain a maxRate... So:
            meanArrivalTime = meanHoldingTime/load;            
        } else {
            meanArrivalTime = (meanHoldingTime * (meanRate / (double) maxRate)) / load;
        }

        // Generate events
        int type, src, dst, qOsType;
        double time = 0.0;
        long id = 1;
        int numNodes = pt.getNumNodes();
        Distribution dist1, dist2, dist3, dist4;
        Event event;
        
        
        dist1 = new Distribution(1, seed);
        dist2 = new Distribution(2, seed);
        dist3 = new Distribution(3, seed);
        dist4 = new Distribution(4, seed);
                
        NodeList disasterAreas =  xml.getElementsByTagName("disaster-area");
        //System.out.println(disasterAreas.getLength());
        
        
        
         
        
        //DisasterGenerator.generateDisaster(pt, dist1.nextInt(numNodes), 1);
        int auxTime = (int) (calls/(float)(disasterAreas.getLength()+2));
        int disasterInst = auxTime;
        int count = 0;
        disasterArrival = new float[disasterAreas.getLength()];
        
        for (int j = 0; j < calls; j++) {
            type = weightVector[dist1.nextInt(TotalWeight)];
            qOsType = weightVectorService[dist1.nextInt(servicesTotalWeight)];
            src = dst = dist2.nextInt(numNodes);
            while (src == dst) {
                dst = dist2.nextInt(numNodes);
            }
            if(j==disasterInst && count<disasterAreas.getLength()){
                
                DisasterArea area = DisasterGenerator.generateDisaster(pt, dist2.nextInt(numNodes), 2);
                
                Element disasterArea = (Element) disasterAreas.item(count);                
                NodeList listNodes = disasterArea.getElementsByTagName("node");
                NodeList listLinks = disasterArea.getElementsByTagName("link"); 
                
                if(listNodes.getLength()>0||listLinks.getLength()>0){
                    
                    int []nodes = new int[listNodes.getLength()];
                    int []links = new int[listLinks.getLength()];
                    
                    for(int i = 0; i<listNodes.getLength();i++){
                        
                        nodes[i] = Integer.parseInt(((Element)listNodes.item(i)).getAttribute("id"));
                        
                    }
                    for(int i = 0; i<listLinks.getLength();i++){
                        
                        links[i] = Integer.parseInt(((Element)listLinks.item(i)).getAttribute("id"));
                        
                    }                    
                    area = DisasterGenerator.generateDisasterArea(nodes, links);
                    
                }
                    
                event = new DisasterArrivalEvent(area);
                time += dist3.nextExponential(meanArrivalTime);
                event.setTime(time);
                disasterArrival[count] = (float) time;
                events.addEvent(event);
                event = new DisasterDepartureEvent(area);
                //time += dist4.nextExponential(callsTypesInfo[type].getHoldingTime())*listLinks.getLength();//+10;
                //System.out.println("Holding time desastre: " + (dist4.nextExponential(callsTypesInfo[type].getHoldingTime())*((float)listLinks.getLength()/5)));//*3 + 3));
                event.setTime(time + (dist4.nextExponential(callsTypesInfo[type].getHoldingTime())*((float)listLinks.getLength()/5)));//*3 + 3));//dist4.nextExponential(callsTypesInfo[type].getHoldingTime()));                    
                events.addEvent(event);                            
             
                //time += dist4.nextExponential(callsTypesInfo[type].getHoldingTime())/8;
                /*event = new DelayedFlowsArrival();
                event.setTime(time + dist4.nextExponential(callsTypesInfo[type].getHoldingTime())/4);
                events.addEvent(event);*/
                id++;    
                disasterInst += auxTime;
                count++;
                continue;                                
                
            }
            ServiceInfo serviceInfo = new ServiceInfo(servicesTypesInfo[qOsType].getServiceInfo(), servicesTypesInfo[qOsType].getDegradationTolerance(), servicesTypesInfo[qOsType].getDelayTolerance(), servicesTypesInfo[qOsType].getWeight());
            //System.out.println(serviceInfo.printInfo());
            Flow flow = new Flow(id, src, dst, callsTypesInfo[type].getRate(), (float)maxRate, callsTypesInfo[type].getCOS(), serviceInfo);
            FlowArrivalEvent ArrivalEvent = new FlowArrivalEvent(flow);
            time += dist3.nextExponential(meanArrivalTime);                        
            ArrivalEvent.setTime(time);
            event = new FlowDepartureEvent(id,flow);
            event.setTime(time + dist4.nextExponential(callsTypesInfo[type].getHoldingTime()));            
            events.addEvent(event);             
            flow.setDepartureEvent((FlowDepartureEvent) event);
            id++;
            flow.setArrivalEvent(ArrivalEvent);
            ArrivalEvent.setFlow(flow);
            events.addEvent(ArrivalEvent);  
            //System.out.println("Holding time: " + (flow.getDepartureEvent().getTime() - flow.getArrivalEvent().getTime()));
                                   
        }
    }
}
