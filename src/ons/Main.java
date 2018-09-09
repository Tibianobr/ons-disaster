/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ons;

import java.io.File;
import java.io.IOException;
import javax.swing.JFrame;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import org.apache.commons.io.FileUtils;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

/**
 * The Main class takes care of the execution of the simulator,
 * which includes dealing with the arguments called (or not) on
 * the command line.
 * 
 * @author onsteam
 */
public class Main {

    /**
     * Instantiates a Simulator object and takes the arguments from the command line.
     * Based on the number of arguments, can detect if there are too many or too few,
     * which prints a message teaching how to run WDMSim. If the number is correct,
     * detects which arguments were applied and makes sure they have the expected effect.
     * 
     * @param args the command line arguments
     */
    public static void main(String[] args) throws ParserConfigurationException, SAXException, IOException {
       
        Simulator wdm;
        String usage = "Usage: ONS simulation_file seed numSeed [-trace] [-verbose] [minload maxload step]";
        String simConfigFile;
        boolean verbose = false;
        boolean trace = false;
        int seed = 1;
        double minload = 0, maxload = 0, step = 1;
        int numSeed=1;

        if (args.length < 2 || args.length > 8) {
            System.out.println(usage);
            System.exit(0);
        } else {
            if (args.length == 4 || args.length == 5) {
                if (args[3].equals("-verbose")) {
                    verbose = true;
                } else {
                    if (args[3].equals("-trace")) {
                        trace = true;
                    } else {
                        System.out.println(usage);
                        System.exit(0);
                    }
                }
            }
            if (args.length == 4 || args.length == 8) {
                if ((args[3].equals("-trace") && args[4].equals("-verbose")) || (args[4].equals("-trace") && args[3].equals("-verbose"))) {
                    trace = true;
                    verbose = true;
                } else {
                    System.out.println(usage);
                    System.exit(0);
                }
            }
            if (args.length == 6 || args.length == 7 || args.length == 8) {
                minload = Double.parseDouble(args[args.length - 3]);
                maxload = Double.parseDouble(args[args.length - 2]);
                step = Double.parseDouble(args[args.length - 1]);
            }            
        }

        simConfigFile = args[0];
        seed = Integer.parseInt(args[1]);        
        numSeed = Integer.parseInt(args[2]); 
        
        DocumentBuilderFactory docBuilderFactory = DocumentBuilderFactory.newInstance();
        DocumentBuilder docBuilder = docBuilderFactory.newDocumentBuilder();
        Document doc = docBuilder.parse(new File(simConfigFile));
        NodeList nodeList = doc.getElementsByTagName("ra");
        Node node = nodeList.item(0);
        Element el = (Element) node;
        String module = el.getAttribute("module"); 
        
        File dirDP = new File("Chart/testes/" + module);
        
        if (dirDP.exists()) {
            
            //System.out.println("Existe");
            FileUtils.deleteDirectory(dirDP);            

        }  
        
        for(int s=seed; s<numSeed+1; s++){
            
            for (double load = minload; load <= maxload; load += step) {
                wdm = new Simulator();
                wdm.Execute(simConfigFile, trace, verbose, load, s);
            }
            
        } 
        
    }
}
