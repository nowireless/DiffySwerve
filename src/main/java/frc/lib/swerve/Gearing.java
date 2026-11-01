package frc.lib.swerve;

public class Gearing {

    public static final Gearing kRevUltraPlanetary3to1 = new Gearing().with(84, 29);
    public static final Gearing kRevUltraPlanetary4to1 = new Gearing().with(76, 21);
    public static final Gearing kRevUltraPlanetary5to1 = new Gearing().with(68, 13);

    private final double gearing;

    public Gearing(double ratio) {
        gearing = ratio;
    }

    public Gearing() {
        this(1.0);
    }

    public Gearing(double driven, double driving) {
        this(driven / driving);
    }

    public double get() {
        return gearing;
    };

    public Gearing with(double  driven, double driving) { 
        return new Gearing(gearing *  driven/driving);
    }

    public Gearing with(Gearing stage) { 
        return new Gearing(gearing *  stage.get());
    }
}
