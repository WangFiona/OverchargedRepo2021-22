package overcharged.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Overcharged Team #12599
 * Teleop for mecanum robot red alliance
 */
@Config
@TeleOp(name="red_teleop", group="Game")
public class redteleop extends teleop {
    @Override
    public void init() {
        super.init();
        direction = -1;
    }
}
