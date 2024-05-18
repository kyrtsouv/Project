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
    private double angleLimit = 0.00001;

    private LightSensor ll, lf, lr;
    private RangeSensorBelt sonars;
    private State state;

    private double iL, iH;
    private double ik, ik_1, ik_2;

    public MyRobot(Vector3d position, String name) {
        super(position, name);
        lf = RobotFactory.addLightSensor(this);
        ll = RobotFactory.addLightSensor(this, new Vector3d(0.15, 0.25, 0.15), 0, "");
        lr = RobotFactory.addLightSensor(this, new Vector3d(0.15, 0.25, -0.15), 0, "");

        sonars = RobotFactory.addSonarBeltSensor(this, 8);
    }

    public void initBehavior() {
        iL = lf.getLux();
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
            iH = lf.getLux();
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
            state = State.FORWARD;
            ik = 0;
            ik_1 = -0.1;
            ik_2 = -0.2;
            this.setRotationalVelocity(0);
            return;
        }
        this.setRotationalVelocity((lr.getLux() - ll.getLux()) * 1000);
    }

    private void follow() {
        System.out.println("Follow");
        circumNavigate(this, sonars, true);
        // if (lf.getLux() < iH) {
        // this.setTranslationalVelocity(0);
        // this.setRotationalVelocity(0);
        // iL = lf.getLux();
        // state = State.ORIENTATE;
        // }
    }

    double K1 = 5;
    double K2 = 0.8;
    double K3 = 1;
    double SAFETY = 0.8;

    public void circumNavigate(Agent rob, RangeSensorBelt sonars, boolean CLOCKWISE) {
        int min = 0;
        for (int i = 1; i < sonars.getNumSensors(); i++)
            if (sonars.getMeasurement(i) < sonars.getMeasurement(min))
                min = i;
        Point3d p = this.getSensedPoint(rob, sonars, min);
        double d = p.distance(new Point3d(0, 0, 0));
        Vector3d v;
        v = CLOCKWISE ? new Vector3d(-p.z, 0, p.x) : new Vector3d(p.z, 0, -p.x);
        double phLin = Math.atan2(v.z, v.x);
        double phRot = Math.atan(K3 * (d - SAFETY));
        if (CLOCKWISE)
            phRot = -phRot;
        double phRef = wrapToPi(phLin + phRot);

        rob.setRotationalVelocity(K1 * phRef);
        rob.setTranslationalVelocity(K2 * Math.cos(phRef));
    }

    public Point3d getSensedPoint(Agent rob, RangeSensorBelt sonars, int sonar) {
        double v;
        if (sonars.hasHit(sonar))
            v = rob.getRadius() + sonars.getMeasurement(sonar);
        else
            v = rob.getRadius() + sonars.getMaxRange();
        double x = v * Math.cos(sonars.getSensorAngle(sonar));
        double z = v * Math.sin(sonars.getSensorAngle(sonar));
        return new Point3d(x, 0, z);
    }

    public double wrapToPi(double a) {
        if (a > Math.PI)
            return a - Math.PI * 2;
        if (a <= -Math.PI)
            return a + Math.PI * 2;
        return a;
    }
}