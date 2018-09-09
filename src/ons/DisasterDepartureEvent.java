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
public class DisasterDepartureEvent extends Event{
   
    private DisasterArea area;
    
    public DisasterDepartureEvent(DisasterArea area){
        
        this.area = area;
        
    }

    /**
     * @return the area
     */
    public DisasterArea getArea() {
        return area;
    }

    /**
     * @param area the area to set
     */
    public void setArea(DisasterArea area) {
        this.area = area;
    }
}
