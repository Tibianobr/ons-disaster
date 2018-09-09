package ons;

/**
 * The EONOXC refers to an OXC used on a elastic optical network which deals
 * with modulation.
 *
 * @author onsteam
 */
public class EONOXC extends OXC {

    private final int capacity;
    private final int modulations[];

    /**
     * Creates a new EONOXC object. All its attributes must be given given by
     * parameter.
     *
     * @param id the EONOXC's unique identifier
     * @param groomingInputPorts total number of grooming input ports
     * @param groomingOutputPorts total number of grooming output ports
     * @param capacity
     * @param modulations The modulations which the OXC can handle
     */
    public EONOXC(int id, int groomingInputPorts, int groomingOutputPorts, int capacity, int[] modulations) {
        super(id, groomingInputPorts, groomingOutputPorts);
        this.capacity = capacity;
        this.modulations = modulations;
    }

    /**
     * Retrieves the EONOXC's capacity
     *
     * @return the EONOXC's capacity
     */
    public int getCapacity() {
        return capacity;
    }

    /**
     * Check if the EONOXC has the specific modulations
     *
     * @param modulation The modulation ID
     * @return true if it's present and false if it's not
     */
    public boolean hasModulation(int modulation) {
        return this.modulations[modulation] == 1;
    }
}
