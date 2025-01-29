package artery.application.misbehavior.attack;

simple MisbehaviorCaService like artery.application.ItsG5Service
{
    parameters:
        // Specify static attack type according to attackTypes enum
        volatile int staticAttackType = default(0);
        string AttackType = default("");

        //visualize attack cams
        bool camLocationVisualizer = default(false);
        int camLocationVisualizerMaxLength = default(500);

        // evaluate DCC transmission interval restrictions
        bool withDccRestriction = default(true);

        // generation interval boundaries
        double minInterval @unit(s) = default(0.1s);
        double maxInterval @unit(s) = default(1.0s);

        // generate at fixed rate (using minInterval, optionally restricted by DCC)
        bool fixedRate = default(false);

        // change in orientation triggering CAM generation (in degree)
        double headingDelta = default(4.0);

        // change in position triggering CAM generation
        double positionDelta @unit(m) = default(4.0m);

        // change in speed triggering CAM generation (in meter/second)
        double speedDelta @unit(mps) = default(0.5mps);

}