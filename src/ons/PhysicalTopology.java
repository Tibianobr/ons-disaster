/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ons;

import ons.util.WeightedGraph;

import org.w3c.dom.*;

/**
 * The physical topology of a network refers to he physical layout of devices on
 * a network, or to the way that the devices on a network are arranged and how
 * they communicate with each other.
 *
 * @author onsteam
 */
public abstract class PhysicalTopology {

    protected int nodes;
    protected int links;
    protected OXC[] nodeVector;
    protected Link[] linkVector;
    protected Link[][] adjMatrix;

    /**
     * Creates a new PhysicalTopology object. Takes the XML file containing all
     * the information about the simulation environment and uses it to populate
     * the PhysicalTopology object. The physical topology is basically composed
     * of nodes connected by links, each supporting different wavelengths.
     *
     * @param xml file that contains the simulation environment information
     */
    public PhysicalTopology(Element xml) {
        try {
            if (Simulator.verbose) {
                System.out.println(xml.getAttribute("name"));
            }
        } catch (Throwable t) {
            t.printStackTrace();
        }
    }
        

    /**
     * Retrieves the number of nodes in a given PhysicalTopology.
     *
     * @return the value of the PhysicalTopology's nodes attribute
     */
    public int getNumNodes() {
        return nodes;
    }

    /**
     * Retrieves the number of links in a given PhysicalTopology.
     *
     * @return number of items in the PhysicalTopology's linkVector attribute
     */
    public int getNumLinks() {
        return linkVector.length;
    }

    /**
     * Retrieves a specific node in the PhysicalTopology object.
     *
     * @param id the node's unique identifier
     * @return specified node from the PhysicalTopology's nodeVector
     */
    public OXC getNode(int id) {
        return nodeVector[id];
    }
    
    /**
     * Retrives the all free grooming input ports from all nodes
     * @return the number of grooming input ports from all nodes
     */
    public int getAllFreeGroomingInputPorts(){
        int ports = 0;
        for(int i = 0; i < nodes; i++){
            ports = ports + this.getNode(i).freeGroomingInputPorts.size();
        }
        return ports;
    }

    /**
     * Retrieves a specific link in the PhysicalTopology object, based on its
     * unique identifier.
     *
     * @param linkid the link's unique identifier
     * @return specified link from the PhysicalTopology's linkVector
     */
    public Link getLink(int linkid) {
        return linkVector[linkid];
    }

    /**
     * Retrieves a specific link in the PhysicalTopology object, based on its
     * source and destination nodes.
     *
     * @param src the link's source node
     * @param dst the link's destination node
     * @return the specified link from the PhysicalTopology's adjMatrix
     */
    public Link getLink(int src, int dst) {
        return adjMatrix[src][dst];
    }

    /**
     * Retrives a given PhysicalTopology's adjancency matrix, which contains the
     * links between source and destination nodes.
     *
     * @return the PhysicalTopology's adjMatrix
     */
    public Link[][] getAdjMatrix() {
        return adjMatrix;
    }

    /**
     * Says whether exists or not a link between two given nodes.
     *
     * @param node1 possible link's source node
     * @param node2 possible link's destination node
     * @return true if the link exists in the PhysicalTopology's adjMatrix
     */
    public boolean hasLink(int node1, int node2) {
        return adjMatrix[node1][node2] != null;
    }

    /**
     * Checks if a path made of links makes sense by checking its continuity
     *
     * @param links to be checked
     * @return true if the link exists in the PhysicalTopology's adjMatrix
     */
    public boolean checkLinkPath(int links[]) {
        for (int i = 0; i < links.length - 1; i++) {
            if (!(getLink(links[i]).dst == getLink(links[i + 1]).src)) {
                return false;
            }
        }
        return true;
    }

    /**
     * Returns a weighted graph with vertices, edges and weights representing
     * the physical network nodes, links and weights implemented by this class
     * object.
     *
     * @return an WeightedGraph class object
     */
    public WeightedGraph getWeightedGraph() {
        WeightedGraph g = new WeightedGraph(nodes);
        for (int i = 0; i < nodes; i++) {
            for (int j = 0; j < nodes; j++) {
                if (hasLink(i, j)) {
                    g.addEdge(i, j, getLink(i, j).getWeight());
                }
            }
        }
        return g;
    }

    /**
     *
     *
     */
    public void printXpressInputFile() {

        // Edges
        System.out.println("EDGES: [");
        for (int i = 0; i < this.getNumNodes(); i++) {
            for (int j = 0; j < this.getNumNodes(); j++) {
                if (this.hasLink(i, j)) {
                    System.out.println("(" + Integer.toString(i + 1) + " " + Integer.toString(j + 1) + ") 1");
                } else {
                    System.out.println("(" + Integer.toString(i + 1) + " " + Integer.toString(j + 1) + ") 0");
                }
            }
        }
        System.out.println("]");
        System.out.println();

        // SD Pairs
        System.out.println("TRAFFIC: [");
        for (int i = 0; i < this.getNumNodes(); i++) {
            for (int j = 0; j < this.getNumNodes(); j++) {
                if (i != j) {
                    System.out.println("(" + Integer.toString(i + 1) + " " + Integer.toString(j + 1) + ") 1");
                } else {
                    System.out.println("(" + Integer.toString(i + 1) + " " + Integer.toString(j + 1) + ") 0");
                }
            }
        }
        System.out.println("]");
    }

    /**
     * Prints all nodes and links between them in the PhysicalTopology object.
     *
     * @return string containing the PhysicalTopology's adjMatrix values
     */
    @Override
    public String toString() {
        String topo = "";
        for (int i = 0; i < nodes; i++) {
            for (int j = 0; j < nodes; j++) {
                if (adjMatrix[i][j] != null) {
                    topo += adjMatrix[i][j].toString() + "\n\n";
                }
            }
        }
        return topo;
    }

    public abstract void createPhysicalLightpath(LightPath lp);

    public abstract void removePhysicalLightpath(LightPath lp);

    public abstract boolean canCreatePhysicalLightpath(LightPath lp);
    
    public abstract int getBW(LightPath lp);

    public abstract int getBWAvailable(LightPath lp);

    public abstract boolean canAddFlow(Flow flow, LightPath lightpath);

    public abstract void addFlow(Flow flow, LightPath lightpath);

    public abstract void removeFlow(Flow flow, LightPath lightpath);
    
    public abstract void degradeFlow();
}
