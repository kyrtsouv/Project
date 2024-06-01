package Bug;

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

    private double iH;
    private double[] luxValues;

    public MyRobot(Vector3d position, String name) {
        super(position, name);
        lf = RobotFactory.addLightSensor(this);
        ll = RobotFactory.addLightSensor(this, new Vector3d(0.15, 0.25, 0.15), 0, "");
        lr = RobotFactory.addLightSensor(this, new Vector3d(0.15, 0.25, -0.15), 0, "");

        sonars = RobotFactory.addSonarBeltSensor(this, 24);

        luxValues = new double[9];
    }

    public void initBehavior() {
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

    private void orientate() {
        double diff = lr.getLux() - ll.getLux();

        if (Math.abs(diff) < angleLimit && lf.getLux() < lr.getLux()) {
            this.setRotationalVelocity(0);
            resetLuxValues();
            state = State.FORWARD;
            return;
        }
        this.setRotationalVelocity(diff > 0 ? 0.5 : -0.5);
    }

    private void goForward() {
        addLuxValue();

        if (lf.getLux() >= luxLimit) {
            this.setTranslationalVelocity(0);
            state = State.STOP;
            return;
        }
        if (sonars.getFrontQuadrantHits() > 0) {
            this.setTranslationalVelocity(0);
            iH = lf.getLux();
            resetLuxValues();
            state = State.FOLLOW;
            return;
        }
        if (isLocalMaximum()) {
            this.setTranslationalVelocity(0);
            state = State.ORIENTATE;
            return;
        }
        this.setTranslationalVelocity(1 - lf.getLux());

    }

    private void follow() {
        circumNavigate();
        addLuxValue();
        if (isLocalMaximum() && lf.getLux() > iH) {
            this.setTranslationalVelocity(0);
            this.setRotationalVelocity(0);
            state = State.ORIENTATE;
            return;
        }
    }

    private void circumNavigate() {
        double K1 = 5;
        double K2 = 0.8;
        double K3 = 1;
        double SAFETY = 0.8;

        int min = 0;
        for (int i = 1; i < sonars.getNumSensors(); i++)
            if (sonars.getMeasurement(i) < sonars.getMeasurement(min))
                min = i;
        double distance = getRadius() + sonars.getMeasurement(min);
        double x = distance * Math.cos(sonars.getSensorAngle(min));
        double z = distance * Math.sin(sonars.getSensorAngle(min));

        Vector3d navigationVector = new Vector3d(z, 0, -x);

        double linearAngle = Math.atan2(navigationVector.z, navigationVector.x);
        double rotationAngle = K3 * (distance - SAFETY);

        double desiredAngle = linearAngle + rotationAngle;
        if (desiredAngle > Math.PI)
            desiredAngle -= Math.PI * 2;
        if (desiredAngle <= -Math.PI)
            desiredAngle += Math.PI * 2;

        setRotationalVelocity(K1 * desiredAngle);
        setTranslationalVelocity(K2 * Math.cos(desiredAngle));
    }

    private void resetLuxValues() {
        luxValues[0] = lf.getLux();
        for (int i = 1; i < 9; i++)
            luxValues[i] = luxValues[i - 1];
    }

    private void addLuxValue() {
        for (int i = 7; i >= 0; i--)
            luxValues[i + 1] = luxValues[i];
        luxValues[0] = lf.getLux();
    }

    private boolean isLocalMaximum() {
        double ik = (luxValues[0] + luxValues[1] + luxValues[2]) / 3;
        double ik_1 = (luxValues[3] + luxValues[4] + luxValues[5]) / 3;
        double ik_2 = (luxValues[6] + luxValues[7] + luxValues[8]) / 3;
        return ik_2 < ik_1 && ik_1 > ik;
    }
}