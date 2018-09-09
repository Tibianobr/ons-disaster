package ons;

public class DisasterArrivalEvent
        extends Event {

    private DisasterArea area;

    public DisasterArrivalEvent(DisasterArea a) {
        area = a;
    }

    public DisasterArea getArea() {
        return area;
    }

    public void setArea(DisasterArea area) {
        this.area = area;
    }
}
