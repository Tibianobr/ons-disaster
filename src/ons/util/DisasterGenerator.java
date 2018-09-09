package ons.util;

import java.util.ArrayList;
import ons.DisasterArea;
import ons.Link;
import ons.PhysicalTopology;

public class DisasterGenerator {

    public DisasterGenerator() {
    }

    public static DisasterArea generateDisaster(PhysicalTopology pt, int node, int range) {
        ArrayList<Integer> nodes = new ArrayList();

        ArrayList<Integer> links = new ArrayList();

        Link[][] adjMat = pt.getAdjMatrix();

        nodes = cascadeDisaster(pt, node, range);
        //System.out.println("Num nodes: " + nodes.size());
        for (int i = 0; i < nodes.size(); i++) {
            for (int j = 0; j < pt.getNumNodes(); j++) {
                if (adjMat[(nodes.get(i))][j] != null) {
                    links.add(adjMat[(nodes.get(i))][j].getID());
                }
            }
        }

        DisasterArea area = new DisasterArea(nodes, links);

        return area;
    }

    public static ArrayList<Integer> cascadeDisaster(PhysicalTopology pt, int node, int range) {
        ArrayList<Integer> nodes = new ArrayList();

        Link[][] adjMat = pt.getAdjMatrix();
        for (int i = 0; (i < pt.getNumNodes()) && (range > 0); i++) {
            if (adjMat[node][i] != null) {
                nodes.addAll(cascadeDisaster(pt, i, range - 1));
            }
        }

        if (!nodes.contains(node)) {
            nodes.add(node);            
        }

        return nodes;
    }
    
    public static DisasterArea generateDisasterArea(int[] no, int[] lin) {
        ArrayList<Integer> nodes = new ArrayList();
        ArrayList<Integer> links = new ArrayList();  
        
        for(int node : no){
            
            nodes.add(node);            
            
        }
        for(int link : lin){
            
            links.add(link);            
            
        }

        DisasterArea area = new DisasterArea(nodes, links);

        return area;
    }
}
