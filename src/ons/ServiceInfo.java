/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ons;

/**
 *
 * @author Gab
 */
public class ServiceInfo {  
        
    private int serviceInfo;
    private float degradationTolerance;
    private float delayTolerance;
    private int weight;
    
    public ServiceInfo(int id, float degT, float delT, int w){
        this.serviceInfo = id;
        this.degradationTolerance = degT;
        this.delayTolerance = delT;
        this.weight = w;
    }

    /**
     * @return the serviceInfo
     */
    public int getServiceInfo() {
        return serviceInfo;
    }

    /**
     * @param serviceInfo the serviceInfo to set
     */
    public void setServiceInfo(int serviceInfo) {
        this.serviceInfo = serviceInfo;
    }

    /**
     * @return the degradationTolerance
     */
    public float getDegradationTolerance() {
        return degradationTolerance;
    }

    /**
     * @param degradationTolerance the degradationTolerance to set
     */
    public void setDegradationTolerance(float degradationTolerance) {
        this.degradationTolerance = degradationTolerance;
    }

    /**
     * @return the delayTolerance
     */
    public float getDelayTolerance() {
        return delayTolerance;
    }

    /**
     * @param delayTolerance the delayTolerance to set
     */
    public void setDelayTolerance(float delayTolerance) {
        this.delayTolerance = delayTolerance;
    }

    /**
     * @return the weight
     */
    public int getWeight() {
        return weight;
    }
    
    public String printInfo(){
        return "Class: " + this.serviceInfo + " Degradation Tolerance: " + this.degradationTolerance + " Delay Tolerance: " + this.delayTolerance;
    } 
        
}
