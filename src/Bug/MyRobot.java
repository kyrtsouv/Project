package Bug;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import simbad.sim.Agent;
import simbad.sim.LightSensor;
import simbad.sim.RangeSensorBelt;
import simbad.sim.RobotFactory;

public class MyRobot extends Agent {
    private enum State {
        FORWARD, ORIENTATE, FOLLOW, STOP
    }

    private double luxLimit = 0.07304601838134014;
    private double angleLimit = 0.00003;

    private LightSensor ll, lf, lr;
    private RangeSensorBelt sonars;
    private State state;

    // private double iL, iH;
    private double ik, ik_1, ik_2;

    public MyRobot(Vector3d position, String name) {
        super(position, name);
        lf = RobotFactory.addLightSensor(this);
        ll = RobotFactory.addLightSensor(this, new Vector3d(0.15, 0.25, 0.15), 0, "");
        lr = RobotFactory.addLightSensor(this, new Vector3d(0.15, 0.25, -0.15), 0, "");

        sonars = RobotFactory.addSonarBeltSensor(this, 8);
    }

    public void initBehavior() {
        // iL = lf.getLux();
        state = State.ORIENTATE;
    }

    public void performBehavior() {
        switch (state) {
            case FORWARD:
                goForward();
                break;
            case ORIENTATE:
                orientate();
                break;
            case FOLLOW:
                follow();
                break;
            case STOP:
                break;
        }
    }

    private void goForward() {
        System.out.println("Forward");
        ik_2 = ik_1;
        ik_1 = ik;
        ik = lf.getLux();

        if (lf.getLux() >= luxLimit) {
            this.setTranslationalVelocity(0);
            state = State.STOP;
            return;
        }
        if (sonars.getFrontQuadrantHits() > 0) {
            this.setTranslationalVelocity(0);
            // iH = lf.getLux();
            ik = 1;
            ik_1 = 1.1;
            ik_2 = 1.2;
            state = State.FOLLOW;
            return;
        }
        if (ik_2 < ik_1 && ik_1 > ik) {
            this.setTranslationalVelocity(0);
            state = State.ORIENTATE;
            return;
        }
        this.setTranslationalVelocity(1 - lf.getLux());
    }

    private void orientate() {
        System.out.println("Orientate");
        if (Math.abs(lr.getLux() - ll.getLux()) < angleLimit) {
            this.setRotationalVelocity(0);
            ik = 0;
            ik_1 = -0.1;
            ik_2 = -0.2;
            state = State.FORWARD;
            return;
        }
        this.setRotationalVelocity((lr.getLux() - ll.getLux()) * 1000);
    }

    private void follow() {
        System.out.println("Follow");
        circumNavigate();
        ik_2 = ik_1;
        ik_1 = ik;
        ik = lf.getLux();
        if (ik_2 < ik_1 && ik_1 > ik) {
            this.setTranslationalVelocity(0);
            this.setRotationalVelocity(0);
            state = State.ORIENTATE;
            return;
        }
    }

    double K1 = 5;
    double K2 = 0.8;
    double K3 = 1;
    double SAFETY = 0.8;

    private void circumNavigate() {
        int min = 0;
        for (int i = 1; i < sonars.getNumSensors(); i++)
            if (sonars.getMeasurement(i) < sonars.getMeasurement(min))
                min = i;
        Point3d closestPoint = getSensedPoint(min);
        double distance = closestPoint.distance(new Point3d(0, 0, 0));

        Vector3d navigationVector = new Vector3d(closestPoint.z, 0, -closestPoint.x);

        double linearAngle = Math.atan2(navigationVector.z, navigationVector.x);
        double rotationAngle = K3 * (distance - SAFETY);

        double desiredAngle = wrapToPi(linearAngle + rotationAngle);

        setRotationalVelocity(K1 * desiredAngle);
        setTranslationalVelocity(K2 * Math.cos(desiredAngle));
    }

    private Point3d getSensedPoint(int sonar) {
        double v;
        if (sonars.hasHit(sonar))
            v = getRadius() + sonars.getMeasurement(sonar);
        else
            v = getRadius() + sonars.getMaxRange();
        double x = v * Math.cos(sonars.getSensorAngle(sonar));
        double z = v * Math.sin(sonars.getSensorAngle(sonar));
        return new Point3d(x, 0, z);
    }

    private double wrapToPi(double a) {
        if (a > Math.PI)
            return a - Math.PI * 2;
        if (a <= -Math.PI)
            return a + Math.PI * 2;
        return a;
    }
}