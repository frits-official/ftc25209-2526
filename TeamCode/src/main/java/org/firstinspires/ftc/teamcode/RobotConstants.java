package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.*;

@Configurable
public class RobotConstants {
    public static final class PINPOINT {
        public static final String hardwareName = "pinpoint";
        public static final double xOffset = 72;
        public static final double yOffset = 0;
        public static final EncoderDirection forwardDirection = EncoderDirection.REVERSED;
        public static final EncoderDirection strafeDirection = EncoderDirection.FORWARD;
    }
}
