/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ons.util;

/**
 *
 * @author Gab
 */
public class SatsReq {
    
    private float [] valores;
    
    public SatsReq(float[] v){
        
        valores = v;
        
    }
    
    public String gerarString(){
        
        String s ="";
        s+=valores[0];
        for(int i=1; i<valores.length;i++){
            
            s+=" " + valores[i];
            
        }
        return s;
    }
    
}
