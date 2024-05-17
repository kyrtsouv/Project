package Bug;

import javax.vecmath.Vector3d;

import simbad.sim.CherryAgent;
import simbad.sim.EnvironmentDescription;

public class Env extends EnvironmentDescription {
    Env() {

        light1SetPosition(0, 2, 0);
        light1IsOn = true;
        // light2IsOn = false;

        add(new CherryAgent(new Vector3d(1, 0, 2), "cherry", 0.1f));
        add(new MyRobot(new Vector3d(0, 0, 0), "robot 1"));
    }
}