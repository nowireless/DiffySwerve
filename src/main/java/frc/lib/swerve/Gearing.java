package frc.lib.swerve;

public class Gearing {
    private double gearing = 1.0;
    
    public double get() {
        return gearing;
    };

    public Gearing with(double  driven, double driving) {
        
        gearing *= driven/driving;

        return this;
    }
}
