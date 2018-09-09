package ons;

import java.util.TreeSet;

/**
 * The Elastic Optical Network (EON) Link represents a Fiberlink in an optical
 * network divided by slots.
 *
 * @author onsteam
 */
public class EONLink extends Link {

    protected int numSlots;
    protected long slots[];
    protected static int guardband;

    public EONLink(int id, int src, int dst, double delay, double weight, int numSlots, int guardband) {
        super(id, src, dst, delay, weight);
        this.numSlots = numSlots;
        this.slots = new long[this.numSlots];
        for (int i = 0; i < this.numSlots; i++) {
            this.slots[i] = 0;
        }
        this.guardband = guardband;
    }

    /**
     * Retrieves the guardband size.
     *
     * @return the guardband size
     */
    public int getGuardband() {
        return this.guardband;
    }

    /**
     * Retrieves the number of slots in this link.
     *
     * @return the number of slots in this link
     */
    public int getNumSlots() {
        return numSlots;
    }

    /**
     * Retrieves the slots available in this link.
     *
     * @return the number slots available
     */
    public int getAvaiableSlots() {
        int cont = 0;
        for (int i = 0; i < this.slots.length; i++) {
            if (this.slots[i] == 0) {
                cont++;
            }
        }
        return cont;
    }
    
    /**
     * Retrieves the used slots in this link.
     *
     * @return the number used slots
     */
    public int getUsedSlots() {
        return numSlots - getAvaiableSlots();
    }

    /**
     * Check if there is available slots to accommodate the request.
     *
     * @param requiredSlots the request slots
     * @return true if can be accommodate, false otherwise
     */
    public boolean hasSlotsAvaiable(int requiredSlots) {
        if (requiredSlots > this.slots.length) {
            System.out.println("Required Slots: " + requiredSlots + " Avaible: " +  this.slots.length);
            throw (new IllegalArgumentException());
        }
        int i = 0, cont = 0;
        while (i < this.slots.length) {
            if (this.slots[i] == 0) {
                cont++;
                if (cont == requiredSlots) {
                    return true;
                }
                i++;
            } else {
                cont = 0;
                i++;
            }
        }
        return false;
    }

    /**
     * Retrieves the first slot available to accommodate this requisition.
     *
     * @param requiredSlots the required slots
     * @return the first slot available to accommodate this requisition
     */
    public int getFirstSlotAvailable(int requiredSlots) {
        if (requiredSlots > this.slots.length) {
            throw (new IllegalArgumentException());
        }
        int i = 0, cont = 0;
        while (i < this.slots.length) {
            if (this.slots[i] == 0) {
                cont++;
                if (cont == requiredSlots) {
                    return i - requiredSlots + 1;
                }
                i++;
            } else {
                cont = 0;
                i++;
            }
        }
        return -1;
    }

    /**
     * Retrieves the set of slots available given a minimum size.
     *
     * @param requiredSlots the required slots of set
     * @return the set with first slots available to 'requiredSlots'
     */
    public TreeSet<Integer> getSlotsAvailable(int requiredSlots) {//olha todos os espacos disponiveis levando em cosideracao a banda de guarda
        TreeSet<Integer> slotsAvailable = new TreeSet<>();
        for (int i = 0; i <= this.slots.length - requiredSlots; i++) {
            if (this.slots[i] == 0) {
                if (areSlotsAvaiable(i, i + requiredSlots - 1)) {
                    slotsAvailable.add(i);
                }
            }
        }
        return slotsAvailable;
    }
    
    /**
     * Retrieves the set of slots available given a minimum size [but in array
     * of 'int'].
     *
     * @param requiredSlots the required slots of set
     * @return the array with first slots available to 'requiredSlots'
     */
    public int[] getSlotsAvailableToArray(int requiredSlots) {
        TreeSet<Integer> slotsAvailable = getSlotsAvailable(requiredSlots);
        int[] out = new int[slotsAvailable.size()];
        for (int i = 0; i < out.length; i++) {
            out[i] = slotsAvailable.pollFirst();
        }
        return out;
    }

    /**
     * Checks for available slots considering the guard band.
     *
     * @param begin the begin slot
     * @param end the end slot
     * @return true if is avaiable slots, false otherwise
     */
    public boolean areSlotsAvaiable(int begin, int end) {
        if (begin < 0 || end >= this.slots.length || begin > end) {
            throw (new IllegalArgumentException());
        }
        if (begin < this.guardband) {
            while (begin <= end) {
                if (this.slots[begin] == 0) {
                    begin++;
                } else {
                    return false;
                }
            }
            for (int i = 0; i < this.guardband; i++) {
                if (this.slots[begin] == -1 || this.slots[begin] == 0) {
                    begin++;
                } else {
                    return false;
                }
            }
            return true;
        } else {
            if (end > (this.slots.length - 1) - this.guardband) {
                for (int i = (begin - this.guardband); i < begin; i++) {
                    if (!(this.slots[i] == -1 || this.slots[i] == 0)) {
                        return false;
                    }
                }
                while (begin <= end) {
                    if (this.slots[begin] == 0) {
                        begin++;
                    } else {
                        return false;
                    }
                }
                return true;
            } else {
                for (int i = (begin - this.guardband); i < begin; i++) {
                    if (!(this.slots[i] == -1 || this.slots[i] == 0)) {
                        return false;
                    }
                }
                while (begin <= end) {
                    if (this.slots[begin] == 0) {
                        begin++;
                    } else {
                        return false;
                    }
                }
                for (int i = 0; i < this.guardband; i++) {
                    if (this.slots[begin] == -1 || this.slots[begin] == 0) {
                        begin++;
                    } else {
                        return false;
                    }
                }
                return true;
            }
        }
    }

    /**
     * Reserve slots (with guard band) in this link, ie reserve lightpath.
     *
     * @param id the id of lightpath reserved
     * @param begin the begin of lightpath
     * @param end the end of lightpath
     */
    public void reserveSlots(long id, int begin, int end) {
        if (begin < 0 || end >= this.slots.length || begin > end) {
            throw (new IllegalArgumentException());
        }
        if (begin < this.guardband) {
            while (begin <= end) {
                this.slots[begin] = id;
                begin++;
            }
            for (int i = 0; i < this.guardband; i++) {
                this.slots[begin] = -1;
                begin++;
            }
        } else {
            if (end > (this.slots.length - 1) - this.guardband) {
                for (int i = (begin - this.guardband); i < begin; i++) {
                    this.slots[i] = -1;
                }
                while (begin <= end) {
                    this.slots[begin] = id;
                    begin++;
                }
            } else {
                for (int i = (begin - this.guardband); i < begin; i++) {
                    this.slots[i] = -1;
                }
                while (begin <= end) {
                    this.slots[begin] = id;
                    begin++;
                }
                for (int i = 0; i < this.guardband; i++) {
                    this.slots[begin] = -1;
                    begin++;
                }
            }
        }
    }

    /**
     * Release slots from this link. Examines the corresponding guard bands
     *
     * @param begin the begin
     * @param end the end
     */
    public void releaseSlots(int begin, int end) {
        if (begin < 0 || end >= this.slots.length || begin > end) {
            throw (new IllegalArgumentException());
        }
        if (begin < this.guardband) {
            while (begin <= end) {
                this.slots[begin] = 0;
                begin++;
            }
            if (this.slots[begin + this.guardband] == 0) {
                for (int i = 0; i < this.guardband; i++) {
                    this.slots[begin] = 0;
                    begin++;
                }
            } else {
                if (this.slots[begin + this.guardband] == -1) {
                    int k = begin + this.guardband;
                    while (this.slots[k] == -1) {
                        k++;
                    }
                    int tirar = k - (begin + this.guardband);
                    for (int i = 0; i < tirar; i++) {
                        this.slots[begin] = 0;
                        begin++;
                    }
                }
            }
        } else {
            if (end > (this.slots.length - 1) - this.guardband) {
                if (this.slots[begin - this.guardband - 1] == 0) {
                    for (int i = (begin - this.guardband); i < begin; i++) {
                        this.slots[i] = 0;
                    }
                }
                if (this.slots[begin - this.guardband - 1] == -1) {
                    int k = begin - this.guardband - 1;
                    while (this.slots[k] == -1) {
                        k--;
                    }
                    int tirar = (begin - this.guardband - 1) - k;
                    for (int i = (begin - tirar); i < begin; i++) {
                        this.slots[i] = 0;
                    }
                }
                while (begin <= end) {
                    this.slots[begin] = 0;
                    begin++;
                }
            } else {
                if (begin == this.guardband) {
                    for (int i = (begin - this.guardband); i < begin; i++) {
                        this.slots[i] = 0;
                    }
                } else {
                    if (this.slots[begin - this.guardband - 1] == 0) {
                        for (int i = (begin - this.guardband); i < begin; i++) {
                            this.slots[i] = 0;
                        }
                    }
                    if (this.slots[begin - this.guardband - 1] == -1) {
                        int k = begin - this.guardband - 1;
                        while (this.slots[k] == -1) {
                            k--;
                        }
                        int tirar = (begin - this.guardband - 1) - k;
                        for (int i = (begin - tirar); i < begin; i++) {
                            this.slots[i] = 0;
                        }
                    }
                }
                while (begin <= end) {
                    this.slots[begin] = 0;
                    begin++;
                }
                if (end == this.slots.length - 1 - this.guardband) {
                    for (int i = 0; i < this.guardband; i++) {
                        this.slots[begin] = 0;
                        begin++;
                    }
                } else {
                    if (this.slots[begin + this.guardband] == 0) {
                        for (int i = 0; i < this.guardband; i++) {
                            this.slots[begin] = 0;
                            begin++;
                        }
                    } else {
                        if (this.slots[begin + this.guardband] == -1) {
                            int k = begin + this.guardband;
                            while (this.slots[k] == -1) {
                                k++;
                            }
                            int tirar = k - (begin + this.guardband);
                            for (int i = 0; i < tirar; i++) {
                                this.slots[begin] = 0;
                                begin++;
                            }
                        }
                    }
                }
            }
        }
    }

    /**
     * Retrieves the max size of contiguous slots available.
     *
     * @return the max size of contiguous slots available
     */
    public int maxSizeAvaiable() {
        int cont = 0, max = 0;
        for (int i = 0; i < this.slots.length; i++) {
            if (this.slots[i] == 0) {
                cont++;
            } else {
                if (cont > max) {
                    max = cont;
                }
                cont = 0;
            }
        }
        if (cont > max) {
            return cont;
        }
        return max;
    }

    /**
     * Retrieves the minimun size of contiguous slots available.
     *
     * @return the minimun size of contiguous slots available
     */
    public int minSizeAvaiable() {
        int cont = 0, min = this.slots.length;
        boolean flag = true;
        for (int i = 0; i < this.slots.length; i++) {
            if (this.slots[i] == 0) {
                cont++;
                flag = true;
            } else {
                if (cont < min && flag) {
                    min = cont;
                    flag = false;
                }
                cont = 0;
            }
        }
        if (cont < min && flag) {
            return cont;
        }
        return min;
    }

    /**
     * Check if this lightpath is this link
     *
     * @param id the id of lightpath
     * @return if it is true, false otherwise
     */
    public boolean hasThisLightPath(long id) {
        if (id < 0) {
            throw (new IllegalArgumentException());
        }
        int i = 0;
        while (i < this.slots.length) {
            if (this.slots[i] == id) {
                return true;
            } else {
                i++;
            }
        }
        return false;
    }

    /**
     * Retrieves the information of this link
     *
     * @return the string information
     */
    @Override
    public String toString() {
        String link = Long.toString(id) + ": " + Integer.toString(src) + "->" + Integer.toString(dst) + " delay:" + Double.toString(delay) + "; slots:" + this.slots.length + "; weight:" + Double.toString(weight);
        return link;
    }

    /**
     * Print this link for debbug.
     */
    public void printLink() {
        System.out.print("\n ID:" + this.id + "_" + this.src + "->" + this.dst + " |");
        for (int i = 0; i < this.slots.length; i++) {
            System.out.print(this.slots[i] + "|");
        }
        System.out.println("");
    }

    /**
     * Retrieves the lightpath slot position.
     *
     * @param lp the id of lightpath
     * @return the array[2] with begin position and end position
     */
    public int[] findLightpath(long lp) {
        if (lp <= 0) {
            throw (new IllegalArgumentException());
        }
        int[] position = {-1, -1};
        int i;
        boolean b_flag = true, e_flag = false, flag = false;
        for (i = 0; i < numSlots; i++) {
            if (e_flag) {
                position[1] = i - 1;
                e_flag = false;
            }
            if (slots[i] == lp) {
                if (b_flag) {
                    position[0] = i;
                    b_flag = false;
                }
                e_flag = true;
                flag = true;
            }
            if(flag && !e_flag){
                break;
            }
        }
        if (e_flag) {
            position[1] = i - 1;
        }
        return position;
    }

    /**
     * Retrieves the number of free ranges to allocate these required slots
     * @param requiredSlots the number of required slots
     * @return the number of possibilities to allocate these required slots
     */
    public int rangeFree(int requiredSlots) {
        if (requiredSlots > this.slots.length) {
            throw (new IllegalArgumentException());
        }
        int i = 0, cont = 0, contPossibles = 0;
        while (i < this.slots.length) {
            if (this.slots[i] == 0) {
                cont++;
                if (cont == requiredSlots) {
                    contPossibles++;
                    cont = 0;
                }
                i++;
            } else {
                cont = 0;
                i++;
            }
        }
        return contPossibles;
    }
}
