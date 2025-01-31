package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public final class RobotSpecificConstants {
    public enum TypeOfRobot {
        CRESENDO_ROBOT,
        TESTING_ROBOT,
        CHARGED_UP_ROBOT
    }
    
    public static final TypeOfRobot THE_TYPE_OF_ROBOT = TypeOfRobot.CRESENDO_ROBOT; // Wonder if we can test robot itself to determine this
    public static final boolean IS_COMPETITION_ROBOT = true;

    public static double getFrontToBackAxleToAxleMeters() {
        return Units.inchesToMeters(ROBOT_SPECIFIC_CONSTANTS.getFrontToBackAxleToAxleInches());
    }

    public static double getSideToSideTreadCenterToTreadCenterMeters() {
        return Units.inchesToMeters(ROBOT_SPECIFIC_CONSTANTS.getSideToSideTreadCenterToTreadCenterInches());
    }

    public static double getFrontToBackAxleToAxleInches() {
        return ROBOT_SPECIFIC_CONSTANTS.getFrontToBackAxleToAxleInches();
    }

    public static double getSideToSideTreadCenterToTreadCenterInches() {
        return ROBOT_SPECIFIC_CONSTANTS.getSideToSideTreadCenterToTreadCenterInches();
    }

    public static Rotation2d getAngleOffsetModule0() {
        return Rotation2d.fromDegrees(ROBOT_SPECIFIC_CONSTANTS.getAngleOffsetDegreesMod0());
    }

    public static Rotation2d getAngleOffsetModule1() {
        return Rotation2d.fromDegrees(ROBOT_SPECIFIC_CONSTANTS.getAngleOffsetDegreesMod1());
    }

    public static Rotation2d getAngleOffsetModule2() {
        return Rotation2d.fromDegrees(ROBOT_SPECIFIC_CONSTANTS.getAngleOffsetDegreesMod2());
    }

    public static Rotation2d getAngleOffsetModule3() {
        return Rotation2d.fromDegrees(ROBOT_SPECIFIC_CONSTANTS.getAngleOffsetDegreesMod3());
    }

    private interface SpecificConstants {
        double getFrontToBackAxleToAxleInches();
        double getSideToSideTreadCenterToTreadCenterInches();
        double getAngleOffsetDegreesMod0();
        double getAngleOffsetDegreesMod1();
        double getAngleOffsetDegreesMod2();
        double getAngleOffsetDegreesMod3();
    }

    private static final class FRC2024Robot implements SpecificConstants {
        public double getFrontToBackAxleToAxleInches() { return 24.875; }
        public double getSideToSideTreadCenterToTreadCenterInches() { return 24.5; }
        public double getAngleOffsetDegreesMod0() { return 150.029296875+180.0; }
        public double getAngleOffsetDegreesMod1() { return 61.34765625/*241.34765625*/; }  // motor inversion 180
        public double getAngleOffsetDegreesMod2() { return 192.744140625; }
        public double getAngleOffsetDegreesMod3() { return 199.248046875/*19.248046875*/; }  // motor inversion 180
        //public double getAngleOffsetDegreesMod0() { return 70.400390625; }
        //public double getAngleOffsetDegreesMod1() { return 61.787109375; }
        //public double getAngleOffsetDegreesMod2() { return 193.53515625; }
        //public double getAngleOffsetDegreesMod3() { return 199.07225625; }
    }

    private static final class FRC2023Robot implements SpecificConstants {
        public double getFrontToBackAxleToAxleInches() { return 24.875; }
        public double getSideToSideTreadCenterToTreadCenterInches() { return 24.5; }
        public double getAngleOffsetDegreesMod0() { return 200.478515625; }
        public double getAngleOffsetDegreesMod1() { return 242.490234375; }
        public double getAngleOffsetDegreesMod2() { return 299.4433593749; }
        public double getAngleOffsetDegreesMod3() { return 269.0223093125; }
    }

    /**
    private static final class CompetitionRobot implements SpecificConstants {
        public double getFrontToBackAxleToAxleInches() { return 24.875; }
        public double getSideToSideTreadCenterToTreadCenterInches() { return 24.5; }
        public double getAngleOffsetDegreesMod0() { return 200.478515625; }
        public double getAngleOffsetDegreesMod1() { return 242.490234375; }
        public double getAngleOffsetDegreesMod2() { return 299.4433593749; }
        public double getAngleOffsetDegreesMod3() { return 269.0223093125; }
    }

    private static final class TestRobot implements SpecificConstants {
        public double getFrontToBackAxleToAxleInches() { return 27.25; }
        public double getSideToSideTreadCenterToTreadCenterInches() { return 27.25; }
        public double getAngleOffsetDegreesMod0() { return 62.75390625; }
        public double getAngleOffsetDegreesMod1() { return 173.49609375; }
        public double getAngleOffsetDegreesMod2() { return 21.533203125; }
        public double getAngleOffsetDegreesMod3() { return 290.302734375; }
    }
    **/

    private static final SpecificConstants ROBOT_SPECIFIC_CONSTANTS = IS_COMPETITION_ROBOT ? new FRC2024Robot() : new FRC2023Robot();
    // switch (THE_TYPE_OF_ROBOT) {
    //     case
    // }

}
