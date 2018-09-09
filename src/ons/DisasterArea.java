package ons;

import java.util.ArrayList;

public class DisasterArea {

    private ArrayList<Integer> nodes;
    private ArrayList<Integer> links;

    public DisasterArea(ArrayList<Integer> n, ArrayList<Integer> l) {
        nodes = n;
        links = l;
    }

    public ArrayList<Integer> getNodes() {
        return nodes;
    }

    public void setNodes(ArrayList<Integer> nodes) {
        this.nodes = nodes;
    }

    public ArrayList<Integer> getLinks() {
        return links;
    }

    public void setLinks(ArrayList<Integer> links) {
        this.links = links;
    }
}
