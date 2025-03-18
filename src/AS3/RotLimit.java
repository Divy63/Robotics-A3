package AS3;

public record RotLimit(double minRad, double maxRad) {
    public static RotLimit of(double minRad, double maxRad) {
        return new RotLimit(minRad, maxRad);
    }
}
