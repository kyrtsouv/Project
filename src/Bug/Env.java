package Bug;

import javax.vecmath.Vector3d;

import simbad.sim.EnvironmentDescription;
import simbad.sim.Wall;

public class Env extends EnvironmentDescription {
    Env() {

        light1SetPosition(5, 2, 0);
        light1IsOn = true;

        add(new MyRobot(new Vector3d(0, 0, 0), "robot 1"));
        Wall w1 = new Wall(new Vector3d(1, 0, 0), 4, 1, this);
        w1.rotate90(1);
        add(w1);
        Wall w2 = new Wall(new Vector3d(-1, 0, 1), 4, 1, this);
        add(w2);
        Wall w3 = new Wall(new Vector3d(-2, 0, 0), 4, 1, this);
        w3.rotate90(1);
        add(w3);
    }
}