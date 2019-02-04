/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ons;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
import java.util.ArrayList;
import java.util.Locale;
import org.w3c.dom.*;

import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.DocumentBuilder;
import static ons.TrafficGenerator.callsTypesInfo;
import ons.util.SatsReq;
import org.xml.sax.SAXException;
import org.xml.sax.SAXParseException;

/**
 * Centralizes the simulation execution. Defines what the command line
 * arguments do, and extracts the simulation information from the XML file.
 * 
 * @author onsteam
 */
public class Simulator {

    private static String simName;
    private static final Float simVersion = (float) 1.0;
    public static boolean verbose = false;
    public static boolean trace = false;
    public static EventScheduler events = new EventScheduler();
    
    /**
     * Executes simulation based on the given XML file and the used command line arguments.
     * 
     * @param simConfigFile name of the XML file that contains all information about the simulation
     * @param trace activates the Tracer class functionalities
     * @param verbose activates the printing of information about the simulation, on runtime, for debugging purposes
     * @param forcedLoad range of loads for which several simulations are automated; if not specified, load is taken from the XML file
     * @param seed a number in the interval [1,25] that defines up to 25 different random simulations
     */
    public void Execute(String simConfigFile, boolean trace, boolean verbose, double forcedLoad, int seed) {

        Simulator.verbose = verbose;
        Simulator.trace = trace;

        if (Simulator.verbose) {
            System.out.println("########################################################");
            System.out.println("# ONS - Optical Network Simulator - version " + simVersion.toString() + "  #");
            System.out.println("#######################################################\n");
        }

        try {

            long begin = System.currentTimeMillis();

            if (Simulator.verbose) {
                System.out.println("(0) Accessing simulation file " + simConfigFile + "...");
            }
            DocumentBuilderFactory docBuilderFactory = DocumentBuilderFactory.newInstance();
            DocumentBuilder docBuilder = docBuilderFactory.newDocumentBuilder();
            Document doc = docBuilder.parse(new File(simConfigFile));

            // normalize text representation
            doc.getDocumentElement().normalize();

            // check the root TAG name and version
            int simType = -1;
            simName = doc.getDocumentElement().getNodeName(); 
            
            NodeList nodeList = doc.getElementsByTagName("ra");
            Node node = nodeList.item(0);
            Element el = (Element)node;
            String module = el.getAttribute("module");   
            
            switch (simName) {
                case "wdmsim":
                    if(Simulator.verbose)
                        System.out.println("Simulation type: " + simName + " (WDM)");
                    simType = 0;
                    break;
                case "eonsim":
                    if(Simulator.verbose)
                        System.out.println("Simulation type: " + simName + " (EON)");
                    simType = 1;
                    break;
                default:
                    System.out.println("Root element of the simulation file is " + doc.getDocumentElement().getNodeName() + ", eonsim or wdmsim is expected!");
                    System.exit(0);
            }
            
            if (!doc.getDocumentElement().hasAttribute("version")) {
                System.out.println("Cannot find version attribute!");
                System.exit(0);
            }
            if (Float.compare(new Float(doc.getDocumentElement().getAttribute("version")), simVersion) > 0) {
                System.out.println("Simulation config file requires a newer version of the simulator!");
                System.exit(0);
            }
            if (Simulator.verbose) {
                System.out.println("(0) Done. (" + Float.toString((float) ((float) (System.currentTimeMillis() - begin) / (float) 1000)) + " sec)\n");
            }

            /*
             * Extract physical topology part
             */
            begin = System.currentTimeMillis();
            if (Simulator.verbose) {
                System.out.println("(1) Loading physical topology information...");
            }
            
            PhysicalTopology pt;
            if (simType == 0){
            	pt = new WDMPhysicalTopology((Element) doc.getElementsByTagName("physical-topology").item(0));
            } else{
            	pt = new EONPhysicalTopology((Element) doc.getElementsByTagName("physical-topology").item(0));
            }
            
            if (Simulator.verbose) {
                System.out.println(pt);
            }

            if (Simulator.verbose) {
                System.out.println("(1) Done. (" + Float.toString((float) ((float) (System.currentTimeMillis() - begin) / (float) 1000)) + " sec)\n");
            }

            /*
             * Extract virtual topology part
             */
            begin = System.currentTimeMillis();
            if (Simulator.verbose) {
                System.out.println("(2) Loading virtual topology information...");
            }

            VirtualTopology vt = new VirtualTopology((Element) doc.getElementsByTagName("virtual-topology").item(0), pt);
            		
            if (Simulator.verbose) {
                System.out.println(vt);
            }

            if (Simulator.verbose) {
                System.out.println("(2) Done. (" + Float.toString((float) ((float) (System.currentTimeMillis() - begin) / (float) 1000)) + " sec)\n");
            }

            /*
             * Extract simulation traffic part
             */
            begin = System.currentTimeMillis();
            if (Simulator.verbose) {
                System.out.println("(3) Loading traffic information...");
            }
            
            TrafficGenerator traffic = new TrafficGenerator((Element) doc.getElementsByTagName("traffic").item(0), (Element) doc.getElementsByTagName("QoS").item(0), forcedLoad);
            traffic.generateTraffic((Element) doc.getElementsByTagName("disaster-event").item(0),pt, events, seed);

            if (Simulator.verbose) {
                System.out.println("(3) Done. (" + Float.toString((float) ((float) (System.currentTimeMillis() - begin) / (float) 1000)) + " sec)\n");
            }

            /*
             * Extract simulation setup part
             */
            begin = System.currentTimeMillis();
            if (Simulator.verbose) {
                System.out.println("(4) Loading simulation setup information...");
            }

            MyStatistics st = MyStatistics.getMyStatisticsObject();
            int numberOfCOS = 0;
            if(((Element) doc.getElementsByTagName("traffic").item(0)).hasAttribute("cos")){
                numberOfCOS = Integer.parseInt(((Element) doc.getElementsByTagName("traffic").item(0)).getAttribute("cos"));
                if(numberOfCOS == 0){
                    throw (new IllegalArgumentException("\"cos\" in xml can not be \"0\""));
                }
            } else {
                throw (new IllegalArgumentException("\"cos\" in xml was not set"));
            }
            int statisticStart = 0;
            if(((Element) doc.getElementsByTagName("traffic").item(0)).hasAttribute("statisticStart")){
                statisticStart = Integer.parseInt(((Element) doc.getElementsByTagName("traffic").item(0)).getAttribute("statisticStart"));
            }
            st.statisticsSetup(pt, numberOfCOS, statisticStart);
            
            Tracer tr = Tracer.getTracerObject();
            if (Simulator.trace == true)
            {
            	if (forcedLoad == 0) {
                	tr.setTraceFile(simConfigFile.substring(0, simConfigFile.length() - 4) + ".trace");
            	} else {
                	tr.setTraceFile(simConfigFile.substring(0, simConfigFile.length() - 4) + "_Load_" + Double.toString(forcedLoad) + ".trace");
            	}
            }
            tr.toogleTraceWriting(Simulator.trace);
            
            String raModule = "ons.ra." + ((Element) doc.getElementsByTagName("ra").item(0)).getAttribute("module");
            if (Simulator.verbose) {
                System.out.println("RA module: " + raModule);
            }
            ControlPlane cp = new ControlPlane(raModule, pt, vt);

            if (Simulator.verbose) {
                System.out.println("(4) Done. (" + Float.toString((float) ((float) (System.currentTimeMillis() - begin) / (float) 1000)) + " sec)\n");
            }

            /*
             * Run the simulation
             */
            begin = System.currentTimeMillis();
            if (Simulator.verbose) {
                System.out.println("(5) Running the simulation...");
            }

            SimulationRunner sim = new SimulationRunner(cp, events);

            if (Simulator.verbose) {
                System.out.println("(5) Done. (" + Float.toString((float) ((float) (System.currentTimeMillis() - begin) / (float) 1000)) + " sec)\n");
            }
            
            cp.updateData();
            
            
            int [] numDropReq = new int[5];
            int [] numRestReq = new int[5];
            
            for(Flow f:ControlPlane.restoredFlows){
                
                for (int i = 0; i < 5; i++) {

                    if (f.getBwReq() == callsTypesInfo[i].getRate()) {

                        numRestReq[i]+=1;

                    }

                }
                
            }

            for (Flow f : ControlPlane.droppedFlows) {

                for (int i = 0; i < 5; i++) {

                    if (f.getBwReq() == callsTypesInfo[i].getRate()) {

                        numDropReq[i] += 1;

                    }

                }

            }              
            
            
            
            
            SatsReq [] sts = new SatsReq [4];
            for(int i=0; i<4;i++){
                
                float [] v = new float[5];
                
                for(int j=0; j<5;j++){
                    
                    v[j] = 0;
                    
                }
                
                ArrayList<Flow> classDrop = new ArrayList<Flow>();
                for(Flow f: ControlPlane.droppedFlows){
                    
                    if(f.getServiceInfo().getServiceInfo() == i)
                        classDrop.add(f);
                    
                }
                
                /*for(TrafficInfo tf: callsTypesInfo)
                    System.out.println(tf.getRate());*/
                
                for(Flow f: classDrop){
                    
                    if(f.getBwReq() == callsTypesInfo[0].getRate()/* 100000*/){
                        
                        v[0] += 1;
                        
                    }
                    if(f.getBwReq() == callsTypesInfo[1].getRate() /*150000*/){
                        
                        v[1] += 1;
                        
                    }
                    if(f.getBwReq() == callsTypesInfo[2].getRate() /*200000*/){
                        
                        v[2] += 1;
                        
                    }
                    if(f.getBwReq() == callsTypesInfo[3].getRate() /*250000*/){
                        
                        v[3] += 1;
                        
                    }
                    if(f.getBwReq() == callsTypesInfo[4].getRate() /*300000*/){
                        
                        v[4] += 1;
                        
                    }
                    
                }
                
                for(int j=0; j<5;j++){
                    
                    int num = numRestReq[j]+numDropReq[j];                                   
                        
                    if(num>0)
                        v[j] = (float)v[j]/num;
                    
                }
                sts[i] = new SatsReq(v);
                
            }
            
            
            if (Simulator.verbose) {
                if (forcedLoad == 0) {
                    System.out.println("Statistics (" + simConfigFile + "):\n");
                } else {
                    System.out.println("Statistics for " + Double.toString(forcedLoad) + " erlangs (" + simConfigFile + "):\n");
                }
                System.out.println(st.fancyStatistics(simType));
            } else {
                System.out.println("*****");
                if (forcedLoad != 0) {
                    System.out.println("Load:" + Double.toString(forcedLoad));
                }
                st.printStatistics(simType); 
                try {
                    
                    File dirDP = new File("Chart/testes/"+module+"/dp");
                    if(!dirDP.exists()){
                        
                        try {
                            dirDP.mkdirs();                            
                        } catch (SecurityException se) {
                            //handle it
                        }
                        
                    }
                    File dirBP = new File("Chart/testes/"+module+"/bp");
                    if(!dirBP.exists()){
                        
                        try {
                            dirBP.mkdirs();
                        } catch (SecurityException se) {
                            //handle it
                        }
                        
                    }
                    File dirDTR = new File("Chart/testes/"+module+"/dtr");
                    if(!dirDTR.exists()){
                        
                        try {
                            dirDTR.mkdirs();
                        } catch (SecurityException se) {
                            //handle it
                        }
                        
                    }
                    File dirBBR = new File("Chart/testes/"+module+"/bbr");
                    if(!dirBBR.exists()){
                        
                        try {
                            dirBBR.mkdirs();
                        } catch (SecurityException se) {
                            //handle it
                        }
                        
                    }
                    
                    File dirDPnum = new File("Chart/testes/"+module+"/dpNum");
                    if(!dirDPnum.exists()){
                        
                        try {
                            dirDPnum.mkdirs();
                        } catch (SecurityException se) {
                            //handle it
                        }
                        
                    }
                    
                    File wait = new File("Chart/testes/"+module+"/wait");
                    if(!wait.exists()){
                        
                        try {
                            wait.mkdirs();
                        } catch (SecurityException se) {
                            //handle it
                        }
                        
                    }
                    
                    File intNum = new File("Chart/testes/"+module+"/intNum");
                    if(!intNum.exists()){
                        
                        try {
                            intNum.mkdirs();
                        } catch (SecurityException se) {
                            //handle it
                        }
                        
                    }
                    
                    File ff = new File("Chart/testes/"+module+"/ff");
                    if(!ff.exists()){
                        
                        try {
                            ff.mkdirs();
                        } catch (SecurityException se) {
                            //handle it
                        }
                        
                    }
                    
                    File dirClasses = new File("Chart/testes/"+module+"/classes");
                    if(!dirClasses.exists()){
                        
                        try {
                            dirClasses.mkdirs();
                        } catch (SecurityException se) {
                            //handle it
                        }
                        
                    }
                    
                    File dirRestored = new File("Chart/testes/"+module+"/restNum");
                    if(!dirRestored.exists()){
                        
                        try {
                            dirRestored.mkdirs();
                        } catch (SecurityException se) {
                            //handle it
                        }
                        
                    }
                    
                    File dirRestoredRate = new File("Chart/testes/"+module+"/rest");
                    if(!dirRestoredRate.exists()){
                        
                        try {
                            dirRestoredRate.mkdirs();
                        } catch (SecurityException se) {
                            //handle it
                        }
                        
                    }
                    
                   File dirnumRestDelay = new File("Chart/testes/"+module+"/numRestDelay");
                        if(!dirnumRestDelay.exists()){

                            try {
                                dirnumRestDelay.mkdirs();
                            } catch (SecurityException se) {
                                //handle it
                            }

                        }

                    File networkFrag = new File("Chart/testes/"+module+"/networkFrag");
                    if(!networkFrag.exists()){

                        try {
                            networkFrag.mkdirs();
                        } catch (SecurityException se) {
                            //handle it
                        }

                    }
                    
                    File dirdisPeriod = new File("Chart/testes/"+module+"/disPeriod");
                    if(!dirdisPeriod.exists()){
                        
                        try {
                            dirdisPeriod.mkdirs();
                        } catch (SecurityException se) {
                            //handle it
                        }
                        
                    }
                    
                    for(int i =0; i<4;i++){
                        
                        File service = new File("Chart/testes/" + module + "/classes" + "/"+i);
                        if (!service.exists()) {

                            try {
                                service.mkdirs();
                            } catch (SecurityException se) {
                                //handle it
                            }

                        }
                        
                    } 
                    
                    for(int i =0; i<4;i++){
                        
                        File service = new File("Chart/testes/" + module + "/classes" + "/"+i+"/dropsPorTeste");
                        if (!service.exists()) {

                            try {
                                service.mkdirs();
                            } catch (SecurityException se) {
                                //handle it
                            }

                        }
                        
                    }
                    
                    
                    PrintWriter writer = new PrintWriter(new FileWriter("Chart/testes/"+module+"/dp/testesDP"+Double.toString(forcedLoad)+".txt", true));
                    writer.println(st.getDropRate());                    
                    writer.close();
                    
                    PrintWriter writer2 = new PrintWriter(new FileWriter("Chart/testes/"+module+"/bp/testesBP"+Double.toString(forcedLoad)+".txt", true));
                    writer2.println(st.getBpToString());                    
                    writer2.close();
                    
                    PrintWriter writer3 = new PrintWriter(new FileWriter("Chart/testes/"+module+"/bbr/testesBBR"+Double.toString(forcedLoad)+".txt", true));
                    DecimalFormat df = new DecimalFormat("#.#######", new DecimalFormatSymbols(Locale.US));
                    writer3.println(String.valueOf(st.getBBRToString()).replace(',', '.')); 
                    writer3.close();
                    
                    PrintWriter writer4 = new PrintWriter(new FileWriter("Chart/testes/"+module+"/dtr/testesDTR"+Double.toString(forcedLoad)+".txt", true));
                  
                    writer4.println(df.format(st.getDroppedTrafficRate()));                    
                    writer4.close();    
                    
                    PrintWriter writer5 = new PrintWriter(new FileWriter("Chart/testes/"+module+"/classes/0/fullService_"+Double.toString(forcedLoad)+".txt", true));
                    writer5.println(st.getFullServiceDropRate());                    
                    writer5.close();                      
                    
                    PrintWriter writer6 = new PrintWriter(new FileWriter("Chart/testes/"+module+"/classes/1/delayTolerant_"+Double.toString(forcedLoad)+".txt", true));
                    writer6.println(st.getDelayTolerantDropRate());                    
                    writer6.close();
                    
                    PrintWriter writer7 = new PrintWriter(new FileWriter("Chart/testes/"+module+"/classes/2/degradationTolerant_"+Double.toString(forcedLoad)+".txt", true));
                    writer7.println(st.getDegradedTolerantDropRate());                    
                    writer7.close();
                    
                    PrintWriter writer8 = new PrintWriter(new FileWriter("Chart/testes/"+module+"/classes/3/extremeDegradation_"+Double.toString(forcedLoad)+".txt", true));
                    writer8.println(st.getDegradedServiceDropRate());                    
                    writer8.close();
                    
                    PrintWriter writer9 = new PrintWriter(new FileWriter("Chart/testes/"+module+"/dpNum/dpNum_"+Double.toString(forcedLoad)+".txt", true));
                    writer9.println(st.getDroppedFlows());                    
                    writer9.close(); 
                    
                    PrintWriter writer10 = new PrintWriter(new FileWriter("Chart/testes/"+module+"/intNum/intNum_"+Double.toString(forcedLoad)+".txt", true));
                    writer10.println(st.getRestoredFlows()+st.getDroppedFlows());                    
                    writer10.close();
                    
                    PrintWriter writer13 = new PrintWriter(new FileWriter("Chart/testes/"+module+"/ff/ff_"+Double.toString(forcedLoad)+".txt", true));
                    writer13.println(st.getFairness());                    
                    writer13.close();
                    
                    PrintWriter writer14 = new PrintWriter(new FileWriter("Chart/testes/"+module+"/restNum/restNum_"+Double.toString(forcedLoad)+".txt", true));
                    writer14.println(st.getRestoredFlows());                    
                    writer14.close();
                    
                    PrintWriter writer15 = new PrintWriter(new FileWriter("Chart/testes/"+module+"/rest/rest_"+Double.toString(forcedLoad)+".txt", true));
                    writer15.println(st.getRestoreRate());                    
                    writer15.close();
                    
                    st.setDisruptionPeriodRate((float)st.getDisruptionPeriod()/TrafficGenerator.disasterArrival.length);
                    
                    PrintWriter writer16 = new PrintWriter(new FileWriter("Chart/testes/"+module+"/numRestDelay/numRestDelay_"+Double.toString(forcedLoad)+".txt", true));
                    writer16.println(st.getNumDelay());                    
                    writer16.close();
                    
                    PrintWriter writer17 = new PrintWriter(new FileWriter("Chart/testes/"+module+"/disPeriod/disPeriod_"+Double.toString(forcedLoad)+".txt", true));
                    writer17.println(st.getDisruptionPeriodRate());                    
                    writer17.close();
                    
                    PrintWriter writer12 = new PrintWriter(new FileWriter("Chart/testes/"+module+"/wait/wait_"+Double.toString(forcedLoad)+".txt", true));

                    DecimalFormat df3 = new DecimalFormat("#.########", new DecimalFormatSymbols(Locale.US));
                    PrintWriter writer18 = new PrintWriter(new FileWriter("Chart/testes/"+module+"/networkfrag/networkfrag_"+Double.toString(forcedLoad)+".txt", true));
                    writer18.println(df3.format(st.getNetworkFragmentation()));
                    writer18.close();

                    if(st.getNumDelay()>0){
                        DecimalFormat df2 = new DecimalFormat("#.###", new DecimalFormatSymbols(Locale.US));
                        String result = df2.format(st.getWaitMean()/st.getNumDelay());
                        
                        //System.out.println(result);
                        writer12.println(result);
                        
                    }else{                        
                        writer12.println(-1); 
                        
                    }                        
                    
                    writer12.close();
                    
                    for(int i=0; i<4;i++){
                           
                        PrintWriter writer11 = new PrintWriter(new FileWriter("Chart/testes/"+module+"/classes/"+i+"/dropsPorTeste/dpPorTeste_"+Double.toString(forcedLoad)+".txt", true));
                        writer11.println(sts[i].gerarString());                    
                        writer11.close();                         
                                               
                        
                    }
                                        
                } catch (IOException e) {
                    // do something
                }
            }
            
            // Terminate MyStatistics singleton
            st.finish();

            // Flush and close the trace file and terminate the singleton
            if (Simulator.trace == true)
            	tr.finish();
            
        } catch (SAXParseException err) {
            System.out.println("** Parsing error" + ", line " + err.getLineNumber() + ", uri " + err.getSystemId());
            System.out.println(" " + err.getMessage());

        } catch (SAXException e) {
            Exception x = e.getException();
            ((x == null) ? e : x).printStackTrace();

        } catch (Throwable t) {
            t.printStackTrace();
        }
    }
}
  
