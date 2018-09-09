/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ons;

/**
 * Modulation container class. 
 * This container contain the main informations of the modulations used by simulator
 * @author onsteam
 */
public final class Modulation {
    
    /**
     * Represents the number of modulations used.
     */
    public static final int N_MOD = 8;
    
    
    public static final int _BPSK = 0;
    public static final int _QPSK = 1;
    public static final int _8QAM = 2;
    public static final int _16QAM = 3;
    public static final int _32QAM = 4;
    public static final int _64QAM = 5;
    public static final int _128QAM = 6;
    public static final int _256QAM = 7;
    
    /**
     * Represents the reach of the BPSK in kilometers.
     */
    public static final int _BPSKReach = 8000;
    /**
     * Represents the reach of the QPSK in kilometers.
     */
    public static final int _QPSKReach = 4000;
    /**
     * Represents the reach of the 8QAM in kilometers.
     */
    public static final int _8QAMReach = 2000;
    /**
     * Represents the reach of the 16QAM in kilometers.
     */
    public static final int _16QAMReach = 1000;
    /**
     * Represents the reach of the 32QAM in kilometers.
     */
    public static final int _32QAMReach = 500;
    /**
     * Represents the reach of the 64QAM in kilometers.
     */
    public static final int _64QAMReach = 250;
    /**
     * Represents the reach of the 128QAM in kilometers.
     */
    public static final int _128QAMReach = 125;
    /**
     * Represents the reach of the 256QAM in kilometers.
     */
    public static final int _256QAMReach = 62;
    
    /**
     * Retrieves the number of slots needed this rate in this conditions.
     * @param rate the rate flow in Mbps
     * @param slotSize the slotSize in MHz
     * @param modulation the modulation Type
     * @return the number of slots
     */
    public static int convertRateToSlot(int rate, int slotSize, int modulation) {
        int slots;
        double m;
        m = 1.0 + (double) modulation;
        slots = (int) Math.ceil(((double) rate)/(((double)slotSize) * m));
        return slots;
    }
    
    /**
     * Retrieves the rate in Mbps that the amount of slots supports under this modulation.
     * @param numberOfSlots the number of slots
     * @param slotSize the slotSize in MHz
     * @param modulation the modulation Type
     * @return the rate in Mbps
     */
    public static int convertSlotToRate(int numberOfSlots, int slotSize, int modulation) {
        int rate;
        double m;
        m = 1.0 + (double) modulation;
        rate = (int) ((double) numberOfSlots * (double) slotSize * m);
        return rate;
    }
    
    /**
     * Retrieves the modulation name from the id integer value.
     * @param id integer correponding to the modulation format
     * @return the modulation name
     */
    public static String getModulationName(int id){
        switch (id){
            case 0:
                return "BPSK";
            case 1:
                return "QPSK";
            case 2:
                return "8QAM";
            case 3:
                return "16QAM";
            case 4:
                return "32QAM";
            case 5:
                return "64QAM";
            case 6:
                return "128QAM";
            case 7:
                return "256QAM";
        }
        return "ERROR!";
    }
    
    /**
     * Convert the string value of modulation type to the equivalent id integer
     * value, definied on this class.
     *
     * @param name the name of modulation format
     * @return integer correponding to the modulation format
     */
    public static int convertModulationTypeToInteger(String name) {
        String toLowerCase = name.toLowerCase();
        switch (toLowerCase) {
            case "256qam":
                return _256QAM;
            case "128qam":
                return _128QAM;
            case "64qam":
                return _64QAM;
            case "32qam":
                return _32QAM;
            case "16qam":
                return _16QAM;
            case "8qam":
                return _8QAM;
            case "qpsk":
                return _QPSK;
            case "bpsk":
                return _BPSK;
        }
        return -1;
    }
    
    /**
     * Retrieves the reach of modulation.
     * @param idModulation the modulation id
     * @return the reach of modulation
     */
    public static int getModulationReach(int idModulation){
        switch (idModulation){
            case 0:
                return _BPSKReach;
            case 1:
                return _QPSKReach;
            case 2:
                return _8QAMReach;
            case 3:
                return _16QAMReach;
            case 4:
                return _32QAMReach;
            case 5:
                return _64QAMReach;
            case 6:
                return _128QAMReach;
            case 7:
                return _256QAMReach;
        }
        return -1;
    }
    
    /**
     * Retrieves the best modulation format given the distance
     * @param distance the distance in Km
     * @return the best modulation format for this distance
     */
    public static int getBestModulation(double distance){
        if (distance > _BPSKReach) {
            return -1;
        } else {
            if (distance <= _256QAMReach) {
                return _256QAM;
            } else {
                if (distance <= _128QAMReach) {
                    return _128QAM;
                } else {
                    if (distance <= _64QAMReach) {
                        return _64QAM;
                    } else {
                        if (distance <= _32QAMReach) {
                            return _32QAM;
                        } else {
                            if (distance <= _16QAMReach) {
                                return _16QAM;
                            } else {
                                if (distance <= _8QAMReach) {
                                    return _8QAM;
                                } else {
                                    if (distance <= _QPSKReach) {
                                        return _QPSK;
                                    } else {
                                        return _BPSK;
                                    }

                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
