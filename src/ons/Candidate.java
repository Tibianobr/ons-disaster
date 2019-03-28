package ons;

import java.util.ArrayList;

public class Candidate {
    private ArrayList<Integer> path;
    private int[] nodes;
    private int[] links;
    private Double sizeRoute;
    private int modulation;
    private int requiredSlots;
    private int largerSlotBlock;
    private int freeSlots;
    private Double fragmentation;
    private int[] slots;
    private EONLightPath lp;
    private boolean valid;
    private Double networkFragmentation;
    private long id;


    public Candidate(ArrayList<Integer> path) {
        this.path = path;
        this.modulation = 0;
        this.requiredSlots = 0;
        this.largerSlotBlock = 0;
        this.freeSlots = 0;
        this.sizeRoute = 0.0;
        this.fragmentation = 0.0;
        this.valid = true;

    }

    public long getId() {
        return id;
    }

    public void setId(long id) {
        this.id = id;
    }

    public ArrayList<Integer> getPath() {
        return path;
    }

    public void setPath(ArrayList<Integer> path) {
        this.path = path;
    }

    public int[] getNodes() {
        return nodes;
    }

    public void setNodes(int[] nodes) {
        this.nodes = nodes;
    }

    public int[] getLinks() {
        return links;
    }

    public void setLinks(int[] links) {
        this.links = links;
    }

    public Double getSizeRoute() {
        return sizeRoute;
    }

    public void setSizeRoute(Double sizeRoute) {
        this.sizeRoute = sizeRoute;
    }

    public int getModulation() {
        return modulation;
    }

    public void setModulation(int modulation) {
        this.modulation = modulation;
    }

    public int getRequiredSlots() {
        return requiredSlots;
    }

    public void setRequiredSlots(int requiredSlots) {
        this.requiredSlots = requiredSlots;
    }

    public int getLargerSlotBlock() {
        return largerSlotBlock;
    }

    public void setLargerSlotBlock(int largerSlotBlock) {
        this.largerSlotBlock = largerSlotBlock;
    }

    public int getFreeSlots() {
        return freeSlots;
    }

    public void setFreeSlots(int freeSlots) {
        this.freeSlots = freeSlots;
    }

    public Double getFragmentation() {
        return fragmentation;
    }

    public void setFragmentation(Double fragmentation) {
        this.fragmentation = fragmentation;
    }

    public int[] getSlots() {
        return slots;
    }

    public void setSlots(int[] slots) {
        this.slots = slots;
    }

    public EONLightPath getLp() {
        return lp;
    }

    public void setLp(EONLightPath lp) {
        this.lp = lp;
    }

    public boolean isValid() {
        return valid;
    }

    public void setValid(boolean valid) {
        this.valid = valid;
    }

    public Double getNetworkFragmentation() {
        return networkFragmentation;
    }

    public void setNetworkFragmentation(Double networkFragmentation) {
        this.networkFragmentation = networkFragmentation;
    }
}
