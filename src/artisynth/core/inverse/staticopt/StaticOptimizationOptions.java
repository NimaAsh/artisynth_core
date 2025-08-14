package artisynth.core.inverse.staticopt;

import java.util.ArrayList;
import java.util.List;

import artisynth.core.mechmodels.MechModel;

/**
 * Options for single-posture static optimization against frame-spring wrenches.
 */
public class StaticOptimizationOptions {

    /**
     * Exponent p for the activation-based cost function (default 2).
     */
    public double activationCostExponent = 2.0;

    /**
     * Names of FrameSprings to target. If empty and {@link #autoSelectFrameSpringsByPattern} is
     * non-empty, springs will be auto-selected by name pattern.
     */
    public List<String> frameSpringNames = new ArrayList<>();

    /**
     * Optional name pattern (e.g., "IVDjnt") to auto-select FrameSprings from a {@link MechModel}.
     */
    public String autoSelectFrameSpringsByPattern = "";

    /**
     * Target wrench values for selected FrameSprings. If empty, defaults to zero wrench targets.
     * Size must be 6*N if provided (Fx,Fy,Fz,Mx,My,Mz per spring in the same order as frameSpringNames).
     */
    public double[] targetWrenches = new double[0];

    /**
     * Enable adding 6 reserve actuators (FX,FY,FZ,MX,MY,MZ) at a specified body frame.
     */
    public boolean enableReserves = false;

    /**
     * Body name on which to place 6 reserve {@code FrameExciter}s if reserves are enabled.
     * Typical choice: "pelvis" for OpenSim-like residuals.
     */
    public String reserveBodyName = "pelvis";

    /**
     * Maximum force/moment per reserve exciter (interpreted as max translational force for FX/FY/FZ
     * and max moment for MX/MY/MZ). These set a nominal capacity for mapping; bounds are still 0..1
     * on the excitation variable and a strong penalty discourages usage.
     */
    public double reserveMaxForce = 2000.0; // N
    public double reserveMaxMoment = 200.0; // N*m

    /**
     * Quadratic penalty weight for reserve excitations in the cost. Larger => less use of reserves.
     */
    public double reserveQuadraticWeight = 1e2;

    /**
     * Quadratic penalty weight for muscle excitations in the cost (L2 regularization strength).
     */
    public double activationQuadraticWeight = 1e-3;

    /**
     * Use soft equality (add ||Aeq x - beq||^2 with this weight) instead of hard equality constraints.
     * Strongly recommended to avoid singular/infeasible KKT systems.
     */
    public boolean useSoftEquality = true;
    public double equalityPenaltyWeight = 0.5; // default stronger equality fit for joint mode

    /**
     * Bounds on activations/reserves (if enabled): lower <= x <= upper.
     */
    public boolean enableBounds = true;
    public double lowerBound = 0.0;
    public double upperBound = 1.0;

    /**
     * Choose formulation: "frame" (net IVD wrenches) or "joint" (joint torque distribution like OpenSim).
     */
    public enum Mode { FRAME, JOINT }
    public Mode mode = Mode.FRAME;

    /**
     * For JOINT mode: list of joint coordinate handles by name or auto-select by pattern.
     */
    public java.util.List<String> jointCoordinateNames = new java.util.ArrayList<>();
    public String autoSelectJointCoordinatePattern = "";

    /**
     * Restrict joint-mode SO to only joints whose names contain this substring (e.g., "_IVDjnt").
     * If empty, all joints in jointset are considered.
     */
    public String jointNameFilterSubstring = "_IVDjnt";

    /**
     * Small excitation increment used to compute numerical columns for the mapping from excitations
     * to frame-spring wrenches.
     */
    public double excitationDelta = 0.01;

    /** Debug controls */
    public boolean debug = false;
    public double debugZeroTol = 1e-8;
    public int debugMaxList = 20;
}


