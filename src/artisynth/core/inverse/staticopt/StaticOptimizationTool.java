package artisynth.core.inverse.staticopt;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import artisynth.core.inverse.QPCostTerm;
import artisynth.core.inverse.QPConstraintTerm;
import artisynth.core.inverse.QPCostTermBase;
import artisynth.core.inverse.QPConstraintTermBase;
import artisynth.core.inverse.QPSolver;
import artisynth.core.mechmodels.ExcitationComponent;
import artisynth.core.inverse.FrameExciter;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.modelbase.ComponentList;
import artisynth.core.modelbase.ModelComponent;
import maspack.matrix.MatrixNd;
import maspack.matrix.VectorNd;

/**
 * Single-posture static optimization matching FrameSpring wrenches.
 *
 * Implementation note: v1 linearizes wrench response numerically around the current posture by
 * small perturbations of muscle (and optional reserve) excitations, then solves a convex QP with
 * L2 activation regularization and equality constraints to match target wrenches.
 */
public class StaticOptimizationTool {

    private final QPSolver qp = new QPSolver();

    public StaticOptimizationResult solveSinglePosture(
            MechModel mech,
            StaticOptimizationOptions opts) {

        boolean useJointMode = (opts.mode == StaticOptimizationOptions.Mode.JOINT);
        if (opts.debug) {
            System.out.println("[SO] mode=" + (useJointMode?"JOINT":"FRAME"));
        }
        // 1) Select targets
        List<String> springNames = useJointMode ? new ArrayList<>() : collectTargetSpringNames(mech, opts);
        if (opts.debug && !useJointMode) {
            System.out.println("[SO] springs ("+springNames.size()+"): " + String.join(", ", springNames.subList(0, Math.min(springNames.size(), opts.debugMaxList))));
        }

        // 2) Build decision variable list = [muscle exciters] + [optional reserves]
        List<ExcitationComponent> exciters = collectExciters(mech);
        List<FrameExciter> reserves = new ArrayList<>();
        if (opts.enableReserves) {
            reserves = createOrGetReserves(mech, opts.reserveBodyName, opts.reserveMaxForce, opts.reserveMaxMoment);
        }

        int numVars = exciters.size() + reserves.size();
        if (opts.debug) {
            System.out.println("[SO] exciters="+exciters.size()+", reserves="+reserves.size()+", numVars="+numVars);
            System.out.print("[SO] first exciters: ");
            for (int i=0; i<Math.min(exciters.size(), opts.debugMaxList); i++) {
                System.out.print(exciters.get(i).getName()+", ");
            }
            System.out.println();
        }

        // 3) Build mapping
        int rows;
        if (!useJointMode) {
            // Row blocks: 6 per spring (Fx,Fy,Fz,Mx,My,Mz)
            rows = 6 * springNames.size();
        } else {
            // One row per selected joint coordinate torque
            rows = collectJointCoordinates(mech, opts).size();
        }
        MatrixNd Aeq = new MatrixNd(rows, numVars);
        VectorNd beq = new VectorNd(rows);
        if (opts.debug) {
            System.out.println("[SO] rows (targets)="+rows);
            if (useJointMode) {
                List<artisynth.core.mechmodels.JointCoordinateHandle> cs = collectJointCoordinates(mech, opts);
                System.out.print("[SO] joint coords ("+cs.size()+"): ");
                for (int i=0; i<Math.min(cs.size(), opts.debugMaxList); i++) {
                    System.out.print(cs.get(i).getJoint().getName()+":"+cs.get(i).getCoordinateName()+", ");
                }
                System.out.println();
                // Joint inventory from jointset (preferred) or bodyConnectors fallback
                int jointCount = 0, ivdCount = 0;
                @SuppressWarnings("unchecked")
                ComponentList<ModelComponent> jointComponents = (ComponentList<ModelComponent>) mech.get("jointset");
                if (jointComponents != null) {
                    for (ModelComponent comp : jointComponents) {
                        if (comp instanceof artisynth.core.mechmodels.JointBase) {
                            artisynth.core.mechmodels.JointBase jb = (artisynth.core.mechmodels.JointBase) comp;
                            boolean match = (opts.jointNameFilterSubstring==null || opts.jointNameFilterSubstring.isEmpty() ||
                                             (jb.getName()!=null && jb.getName().contains(opts.jointNameFilterSubstring)));
                            jointCount++; if (match) ivdCount++;
                            System.out.println("[SO]   joint(jointset): name="+jb.getName()+" coords="+jb.numCoordinates()+
                                               " matchIVD="+match);
                        }
                    }
                    System.out.println("[SO] jointset joints="+jointCount+", joints(match filter)="+ivdCount);
                } else {
                    int bcCount = 0; jointCount = 0; ivdCount = 0;
                    for (artisynth.core.mechmodels.BodyConnector bc : mech.bodyConnectors()) {
                        bcCount++;
                        if (bc instanceof artisynth.core.mechmodels.JointBase) {
                            artisynth.core.mechmodels.JointBase jb = (artisynth.core.mechmodels.JointBase)bc;
                            jointCount++;
                            boolean match = (opts.jointNameFilterSubstring==null || opts.jointNameFilterSubstring.isEmpty() ||
                                             (jb.getName()!=null && jb.getName().contains(opts.jointNameFilterSubstring)));
                            if (match) ivdCount++;
                            System.out.println("[SO]   joint(bodyConnectors): name="+jb.getName()+" coords="+jb.numCoordinates()+
                                               " matchIVD="+match);
                        }
                    }
                    System.out.println("[SO] bodyConnectors="+bcCount+", joints="+jointCount+", joints(match filter)="+ivdCount);
                }
            }
        }

        // Target vector
        VectorNd tauBase = null;
        if (!useJointMode) {
            double[] target = (opts.targetWrenches != null && opts.targetWrenches.length == rows)
                    ? opts.targetWrenches
                    : new double[rows];
            for (int i = 0; i < rows; i++) beq.set(i, target[i]);
            if (opts.debug) {
                System.out.println("[SO] target wrench vec norm="+beq.norm());
            }
        } else {
            // joint torques target: inverse dynamics torques (gravity/passives)
            // Goal: A a + tau_base = 0 => A a = -tau_base
            tauBase = computeStaticJointTorques(mech, opts);
            beq.set(tauBase);
            beq.scale(-1); // beq = -tau_base
            if (opts.debug) {
                System.out.println("[SO] target joint tau size="+tauBase.size()+", base tau norm="+tauBase.norm()+", beq(norm)="+beq.norm());
            }
        }

        // Baseline: zero all excitations; store & restore originals at the end
        double[] savedExciters = setExcitations(exciters, 0.0);
        double[] savedReserves = setExcitations(reserves, 0.0);

        // Compute baseline net wrenches (force equilibrium at each IVD frame) and move to RHS
        VectorNd baseVec;
        if (!useJointMode) {
            Map<String, double[]> baseW = measureNetWrenchesAtSprings(mech, springNames);
            baseVec = stackWrenches(springNames, baseW);
            beq.sub(baseVec); // beq = target - base
            if (opts.debug) {
                System.out.println("[SO] baseline net wrench norm="+baseVec.norm()+", residual target norm="+beq.norm());
            }
        } else {
            // base is zero for mapping A a = tau (we already set beq = tau)
            baseVec = new VectorNd(rows);
            if (opts.debug) {
                System.out.println("[SO] baseline tau set to zero vec. beq norm="+beq.norm());
            }
        }

        // Build columns for each exciter by delta activation
        double da = Math.max(1e-4, opts.excitationDelta);
        int col = 0;
        for (ExcitationComponent ex : exciters) {
            setExcitation(ex, da);
            mech.updateForces(0.0);
            VectorNd colVec;
            if (!useJointMode) {
                Map<String, double[]> w = measureNetWrenchesAtSprings(mech, springNames);
                colVec = stackWrenches(springNames, w);
                colVec.sub(baseVec);
            } else {
                // tau under perturbed activation minus base tau
                VectorNd tauP = computeStaticJointTorques(mech, opts);
                colVec = new VectorNd(tauP);
                colVec.sub(tauBase);
            }
            for (int r = 0; r < rows; r++) {
                Aeq.set(r, col, colVec.get(r) / da);
            }
            setExcitation(ex, 0.0);
            if (opts.debug && col < 5) {
                VectorNd colN = new VectorNd(rows);
                Aeq.getColumn(col, colN);
                System.out.println("[SO] Aeq col " + col + " ("+ex.getName()+") norm="+colN.norm());
            }
            col++;
        }
        for (FrameExciter rex : reserves) {
            setExcitation(rex, da);
            mech.updateForces(0.0);
            VectorNd colVec;
            if (!useJointMode) {
                Map<String, double[]> w = measureNetWrenchesAtSprings(mech, springNames);
                colVec = stackWrenches(springNames, w);
                colVec.sub(baseVec);
            } else {
                VectorNd tauP = computeStaticJointTorques(mech, opts);
                colVec = new VectorNd(tauP);
                colVec.sub(tauBase);
            }
            for (int r = 0; r < rows; r++) {
                Aeq.set(r, col, colVec.get(r) / da);
            }
            setExcitation(rex, 0.0);
            if (opts.debug) {
                VectorNd colN = new VectorNd(rows);
                Aeq.getColumn(col, colN);
                System.out.println("[SO] Aeq reserve col " + col + " ("+rex.getName()+") norm="+colN.norm());
            }
            col++;
        }

        if (opts.debug) {
            // report row and column norms
            System.out.println("[SO] Aeq size="+Aeq.rowSize()+"x"+Aeq.colSize()+" rank-ish diagnostics");
            for (int r = 0; r < Math.min(Aeq.rowSize(), opts.debugMaxList); r++) {
                double nr = 0;
                for (int c = 0; c < Aeq.colSize(); c++) nr += Aeq.get(r,c)*Aeq.get(r,c);
                System.out.println("    row " + r + " norm="+Math.sqrt(nr));
            }
            for (int c = 0; c < Math.min(Aeq.colSize(), opts.debugMaxList); c++) {
                VectorNd colN = new VectorNd(Aeq.rowSize());
                Aeq.getColumn(c, colN);
                System.out.println("    col " + c + " norm="+colN.norm());
            }
        }

        // 4) Cost: 0.5 x^T Q x + p^T x; use diagonal quadratic weights for muscles and reserves
        MatrixNd Q = new MatrixNd(numVars, numVars);
        VectorNd p = new VectorNd(numVars);
        for (int i = 0; i < exciters.size(); i++) {
            Q.set(i, i, Math.max(0, opts.activationQuadraticWeight));
        }
        for (int i = exciters.size(); i < numVars; i++) {
            Q.set(i, i, Math.max(0, opts.reserveQuadraticWeight));
        }
        // Add simple box bounds if requested (inequalities)
        List<QPConstraintTerm> constraints = new ArrayList<>();
        if (opts.enableBounds) {
            // Lower bounds: x >= lb →  I x >=  lb
            // Upper bounds: x <= ub → -I x >= -ub
            MatrixNd Aineq = new MatrixNd(numVars * 2, numVars);
            VectorNd bineq = new VectorNd(numVars * 2);
            // Lower
            for (int i = 0; i < numVars; i++) {
                Aineq.set(i, i, 1.0);
                bineq.set(i, opts.lowerBound);
            }
            // Upper
            for (int i = 0; i < numVars; i++) {
                Aineq.set(numVars + i, i, -1.0);
                bineq.set(numVars + i, -opts.upperBound);
            }
            constraints.add(new SimpleInequalityConstraint(Aineq, bineq));
            if (opts.debug) {
                System.out.println("[SO] bounds enabled: lb="+opts.lowerBound+", ub="+opts.upperBound+" ("+(numVars*2)+" ineq rows)");
            }
        }

        // 5) Solve: either hard equality or soft equality via penalty
        List<QPCostTerm> costTerms = new ArrayList<>();
        if (opts.useSoftEquality) {
            // Add equality as a quadratic penalty: w * ||Aeq x - beq||^2
            MatrixNd Heq = new MatrixNd(Aeq); // H
            VectorNd veq = new VectorNd(beq); // b
            MatrixNd Qeq = new MatrixNd(numVars, numVars);
            VectorNd peq = new VectorNd(numVars);
            // Qeq += w * H^T H; peq += -w * H^T b
            MatrixNd tmp = new MatrixNd(numVars, numVars);
            tmp.mulTransposeLeft(Heq, Heq);
            // Penalize w||H x - b||^2 = w x^T H^T H x - 2 w b^T H x + const
            // Our QP is 1/2 x^T Q x + x^T p, so set Q += 2 w H^T H and p += -2 w H^T b
            Qeq.scaledAdd(2*opts.equalityPenaltyWeight, tmp);
            VectorNd Htb = new VectorNd(numVars);
            Htb.mulTranspose(Heq, veq);
            peq.scaledAdd(-2*opts.equalityPenaltyWeight, Htb);
            Q.add(Qeq); p.add(peq);
            costTerms.add(new SimpleQuadraticCost(Q, p));
            // no equality constraints added
            if (opts.debug) {
                System.out.println("[SO] using soft equality, w="+opts.equalityPenaltyWeight);
            }
        } else {
            costTerms.add(new SimpleQuadraticCost(Q, p));
            constraints.add(new SimpleEqualityConstraint(Aeq, beq));
            if (opts.debug) {
                System.out.println("[SO] using hard equality; constraints rows="+Aeq.rowSize());
            }
        }

        if (opts.debug) {
            System.out.println("[SO] Q diag (first 10):");
            for (int i=0; i<Math.min(10, Q.rowSize()); i++) System.out.print(Q.get(i,i)+" ");
            System.out.println();
        }

        VectorNd x = qp.solve(costTerms, constraints, numVars, 0, 0);
        if (opts.debug) {
            System.out.println("[SO] solver x norm="+x.norm());
            int nz=0; for (int i=0;i<x.size();i++) if (Math.abs(x.get(i))>opts.debugZeroTol) nz++;
            System.out.println("[SO] nonzero activations="+nz+"/"+x.size());
        }

        // 6) Apply solution and report
        applyExcitations(exciters, reserves, x);
        mech.updateForces(0.0);
        Map<String, double[]> achieved = useJointMode ? new LinkedHashMap<>() : measureNetWrenchesAtSprings(mech, springNames);

        // 6.1) Export Aeq, beq, and achieved tau/wrench vectors
        try {
            String outDir = (useJointMode ? "static_opt_results_joint" : "static_opt_results");
            java.io.File dir = new java.io.File(outDir);
            if (!dir.exists()) { dir.mkdirs(); }
            // Aeq
            try (java.io.FileWriter fw = new java.io.FileWriter(outDir+"/Aeq.csv")) {
                for (int r=0; r<Aeq.rowSize(); r++) {
                    for (int c=0; c<Aeq.colSize(); c++) {
                        fw.write(Double.toString(Aeq.get(r,c)));
                        if (c+1 < Aeq.colSize()) fw.write(",");
                    }
                    fw.write("\n");
                }
            }
            // beq
            try (java.io.FileWriter fw = new java.io.FileWriter(outDir+"/beq.csv")) {
                for (int r=0; r<beq.size(); r++) {
                    fw.write(Double.toString(beq.get(r)));
                    fw.write("\n");
                }
            }
            // achieved vector: frame mode -> stacked wrenches; joint mode -> Aeq x + tau_base
            try (java.io.FileWriter fw = new java.io.FileWriter(outDir+"/achieved_vector.csv")) {
                if (!useJointMode) {
                    Map<String,double[]> w = measureNetWrenchesAtSprings(mech, springNames);
                    VectorNd v = stackWrenches(springNames, w);
                    for (int i=0; i<v.size(); i++) { fw.write(Double.toString(v.get(i))); fw.write("\n"); }
                } else {
                    VectorNd Ax = new VectorNd(Aeq.rowSize());
                    Ax.mul(Aeq, x);
                    // achieved tau = A x + tau_base
                    VectorNd achievedTau = new VectorNd(Ax);
                    if (tauBase != null) { achievedTau.add(tauBase); }
                    for (int i=0; i<achievedTau.size(); i++) { fw.write(Double.toString(achievedTau.get(i))); fw.write("\n"); }
                }
            }
        } catch (IOException ioe) {
            System.err.println("[SO] CSV export error: "+ioe.getMessage());
        }

        // 7) Build result
        StaticOptimizationResult res = new StaticOptimizationResult();
        for (int i = 0; i < exciters.size(); i++) {
            res.activationsByName.put(exciters.get(i).getName(), x.get(i));
        }
        for (int i = 0; i < reserves.size(); i++) {
            res.activationsByName.put(reserves.get(i).getName(), x.get(exciters.size() + i));
        }
        // muscle forces (approx): re-read active forces from mech and sum by name
        // Here we store net forces per exciter by querying targets if possible; v1 keep zero
        for (String name : res.activationsByName.keySet()) {
            res.forcesByName.put(name, 0.0);
        }
        if (!useJointMode) {
            for (String sname : springNames) {
                res.achievedWrenches.put(sname, achieved.get(sname));
            }
            for (int iSpring = 0; iSpring < springNames.size(); iSpring++) {
                double[] tw = new double[6];
                for (int k = 0; k < 6; k++) tw[k] = (opts.targetWrenches != null && opts.targetWrenches.length == rows)
                        ? opts.targetWrenches[iSpring * 6 + k]
                        : 0.0;
                res.targetWrenches.put(springNames.get(iSpring), tw);
            }
            if (opts.debug) {
                VectorNd achievedVec = stackWrenches(springNames, achieved);
                VectorNd residual = new VectorNd(achievedVec);
                residual.sub(baseVec); residual.sub(beq); // residual of (A x - beq)
                System.out.println("[SO] achieved net wrench norm="+achievedVec.norm());
                System.out.println("[SO] residual norm (diag only approx)="+residual.norm());
            }
        }

        // 8) Restore original excitations (do not persist unless caller chooses to)
        restoreExcitations(exciters, savedExciters);
        restoreExcitations(reserves, savedReserves);
        mech.updateForces(0.0);

        return res;
    }

    private List<String> collectTargetSpringNames(MechModel mech, StaticOptimizationOptions opts) {
        List<String> names = new ArrayList<>();
        if (!opts.frameSpringNames.isEmpty()) {
            names.addAll(opts.frameSpringNames);
            return names;
        }
        if (opts.autoSelectFrameSpringsByPattern != null && !opts.autoSelectFrameSpringsByPattern.isEmpty()) {
            RenderableComponentList<?> springs = mech.frameSprings();
            String pat = opts.autoSelectFrameSpringsByPattern;
            for (int i = 0; i < springs.size(); i++) {
                ModelComponent mc = (ModelComponent) springs.get(i);
                if (mc.getName() != null && mc.getName().contains(pat)) {
                    names.add(mc.getName());
                }
            }
        }
        return names;
    }

    private List<ExcitationComponent> collectExciters(MechModel mech) {
        List<ExcitationComponent> out = new ArrayList<>();
        ComponentList<MuscleExciter> mex = mech.getMuscleExciters();
        for (int i = 0; i < mex.size(); i++) {
            out.add(mex.get(i));
        }
        return out;
    }

    private List<FrameExciter> createOrGetReserves(MechModel mech, String bodyName, double maxF, double maxM) {
        List<FrameExciter> out = new ArrayList<>();
        @SuppressWarnings("unchecked")
        ComponentList<artisynth.core.mechmodels.RigidBody> bodies = (ComponentList<artisynth.core.mechmodels.RigidBody>) mech.get("bodyset");
        artisynth.core.mechmodels.RigidBody body = (bodies != null ? bodies.get(bodyName) : null);
        if (body == null) {
            return out;
        }
        FrameExciter.WrenchDof[] dofs = FrameExciter.WrenchDof.values();
        for (FrameExciter.WrenchDof dof : dofs) {
            String name = bodyName + "_reserve_" + dof.toString().toLowerCase();
            FrameExciter ex = findFrameExciter(mech, name);
            if (ex == null) {
                double max = (dof.name().startsWith("M")) ? maxM : maxF;
                ex = new FrameExciter(name, body, dof, max);
                mech.addForceEffector(ex);
            }
            out.add(ex);
        }
        return out;
    }

    private FrameExciter findFrameExciter(MechModel mech, String name) {
        for (Object eff : mech.getForceEffectors()) {
            if (eff instanceof FrameExciter) {
                FrameExciter f = (FrameExciter) eff;
                if (name.equals(f.getName())) return f;
            }
        }
        return null;
    }

    @SuppressWarnings("unused")
    private Map<String, double[]> measureSpringWrenches(MechModel mech, List<String> springNames) {
        Map<String, double[]> map = new LinkedHashMap<>();
        RenderableComponentList<?> springs = mech.frameSprings();
        for (String sname : springNames) {
            ModelComponent mc = springs.get(sname);
            double[] w = new double[6];
            if (mc instanceof artisynth.core.mechmodels.FrameSpring) {
                artisynth.core.mechmodels.FrameSpring fs = (artisynth.core.mechmodels.FrameSpring) mc;
                maspack.spatialmotion.Wrench wr = fs.getSpringForce();
                w[0] = wr.f.x; w[1] = wr.f.y; w[2] = wr.f.z; w[3] = wr.m.x; w[4] = wr.m.y; w[5] = wr.m.z;
            }
            map.put(sname, w);
        }
        return map;
    }

    /**
     * Net wrench at each IVD frame (sum of applied forces/moments on the two bodies, expressed in the IVD frame).
     * This responds to gravity and muscle forces even at fixed posture.
     */
    private Map<String, double[]> measureNetWrenchesAtSprings(MechModel mech, List<String> springNames) {
        Map<String, double[]> map = new LinkedHashMap<>();
        RenderableComponentList<?> springs = mech.frameSprings();
        for (String sname : springNames) {
            ModelComponent mc = springs.get(sname);
            double[] w = new double[6];
            if (mc instanceof artisynth.core.mechmodels.FrameSpring) {
                artisynth.core.mechmodels.FrameSpring fs = (artisynth.core.mechmodels.FrameSpring) mc;
                // Net wrench about the IVD frame: sum of frame forces from both connected bodies
                maspack.spatialmotion.Wrench fA = new maspack.spatialmotion.Wrench(fs.getFrameA().getForce());
                maspack.spatialmotion.Wrench fB = new maspack.spatialmotion.Wrench(fs.getFrameB().getForce());
                maspack.spatialmotion.Wrench sum = new maspack.spatialmotion.Wrench();
                sum.add(fA); sum.add(fB);
                // Express in the spring's frame A coordinates at the frame's origin
                // Use full rigid transform to include moment shift due to the frame origin
                maspack.matrix.RigidTransform3d TAw = fs.getFrameA().getPose();
                maspack.spatialmotion.Wrench sumA = new maspack.spatialmotion.Wrench();
                sumA.inverseTransform(TAw, sum);
                w[0] = sumA.f.x; w[1] = sumA.f.y; w[2] = sumA.f.z; w[3] = sumA.m.x; w[4] = sumA.m.y; w[5] = sumA.m.z;
            }
            map.put(sname, w);
        }
        return map;
    }

    /**
     * Compute static joint torques for selected coordinates (gravity/passives/muscles) at current posture.
     * Uses coordinate wrench wr_i and joint cut wrench (force on frameA minus force on frameB) via
     * virtual work: tau_i = wr_i (world) dot (fA_world - fB_world).
     */
    private VectorNd computeStaticJointTorques(MechModel mech, StaticOptimizationOptions opts) {
        List<artisynth.core.mechmodels.JointCoordinateHandle> coords = collectJointCoordinates(mech, opts);
        VectorNd tau = new VectorNd(coords.size());
        int idx = 0;
        for (artisynth.core.mechmodels.JointCoordinateHandle ch : coords) {
            // coordinate wrench in world
            maspack.spatialmotion.Wrench wr = ch.getWrench();
            // Find the underlying joint and its frames A,B
            artisynth.core.mechmodels.JointBase jb = ch.getJoint();
            if (jb != null) {
                // Use frames of the two rigid bodies if available through the joint's attachments
                artisynth.core.mechmodels.Frame frameA = null;
                artisynth.core.mechmodels.Frame frameB = null;
                if (jb.getBodyA() instanceof artisynth.core.mechmodels.RigidBody) {
                    frameA = (artisynth.core.mechmodels.RigidBody)jb.getBodyA();
                }
                if (jb.getBodyB() instanceof artisynth.core.mechmodels.RigidBody) {
                    frameB = (artisynth.core.mechmodels.RigidBody)jb.getBodyB();
                }
                maspack.spatialmotion.Wrench fA = (frameA != null) ? new maspack.spatialmotion.Wrench(frameA.getForce()) : new maspack.spatialmotion.Wrench();
                maspack.spatialmotion.Wrench fB = (frameB != null) ? new maspack.spatialmotion.Wrench(frameB.getForce()) : new maspack.spatialmotion.Wrench();
                fA.sub(fB); // cut wrench across joint in world
                double t = wr.dot(fA);
                tau.set(idx++, t);
                if (opts.debug && idx <= opts.debugMaxList) {
                    System.out.println("[SO] tau("+jb.getName()+":"+ch.getCoordinateName()+") = " + t +
                        " |wr|="+Math.sqrt(wr.f.normSquared()+wr.m.normSquared())+
                        " |fcut|="+Math.sqrt(fA.f.normSquared()+fA.m.normSquared()));
                }
            } else {
                tau.set(idx++, 0);
            }
        }
        return tau;
    }

    private List<artisynth.core.mechmodels.JointCoordinateHandle> collectJointCoordinates(MechModel mech, StaticOptimizationOptions opts) {
        List<artisynth.core.mechmodels.JointCoordinateHandle> out = new ArrayList<>();

        // Prefer joints listed in the component tree under "jointset"
        @SuppressWarnings("unchecked")
        ComponentList<ModelComponent> jointComponents = (ComponentList<ModelComponent>) mech.get("jointset");

        if (jointComponents != null) {
            boolean byPattern = (opts.autoSelectJointCoordinatePattern != null && !opts.autoSelectJointCoordinatePattern.isEmpty());
            boolean byNames = (opts.jointCoordinateNames != null && !opts.jointCoordinateNames.isEmpty());

            for (ModelComponent comp : jointComponents) {
                if (!(comp instanceof artisynth.core.mechmodels.JointBase)) {
                    continue;
                }
                artisynth.core.mechmodels.JointBase jb = (artisynth.core.mechmodels.JointBase) comp;
                if (opts.jointNameFilterSubstring != null && !opts.jointNameFilterSubstring.isEmpty()) {
                    if (jb.getName() == null || !jb.getName().contains(opts.jointNameFilterSubstring)) {
                        continue;
                    }
                }
                for (int i = 0; i < jb.numCoordinates(); i++) {
                    String cname = jb.getCoordinateName(i);
                    if (byPattern) {
                        if (cname != null && cname.contains(opts.autoSelectJointCoordinatePattern)) {
                            out.add(new artisynth.core.mechmodels.JointCoordinateHandle(jb, i));
                        }
                    } else if (byNames) {
                        if (cname != null && opts.jointCoordinateNames.contains(cname)) {
                            out.add(new artisynth.core.mechmodels.JointCoordinateHandle(jb, i));
                        }
                    } else {
                        // No name/pattern constraint: include all coords for matching joints
                        out.add(new artisynth.core.mechmodels.JointCoordinateHandle(jb, i));
                    }
                }
            }

            // If byNames requested but nothing matched, include all coords from matching joints
            if (out.isEmpty() && byNames) {
                for (ModelComponent comp : jointComponents) {
                    if (comp instanceof artisynth.core.mechmodels.JointBase) {
                        artisynth.core.mechmodels.JointBase jb = (artisynth.core.mechmodels.JointBase) comp;
                        if (opts.jointNameFilterSubstring != null && !opts.jointNameFilterSubstring.isEmpty()) {
                            if (jb.getName() == null || !jb.getName().contains(opts.jointNameFilterSubstring)) {
                                continue;
                            }
                        }
                        for (int i = 0; i < jb.numCoordinates(); i++) {
                            out.add(new artisynth.core.mechmodels.JointCoordinateHandle(jb, i));
                        }
                    }
                }
            }
        }

        // Fallback: bodyConnectors list (may contain a subset only)
        if (out.isEmpty()) {
            for (artisynth.core.mechmodels.BodyConnector bc : mech.bodyConnectors()) {
                if (bc instanceof artisynth.core.mechmodels.JointBase) {
                    artisynth.core.mechmodels.JointBase jb = (artisynth.core.mechmodels.JointBase) bc;
                    if (opts.jointNameFilterSubstring != null && !opts.jointNameFilterSubstring.isEmpty()) {
                        if (jb.getName() == null || !jb.getName().contains(opts.jointNameFilterSubstring)) {
                            continue;
                        }
                    }
                    for (int i = 0; i < jb.numCoordinates(); i++) {
                        out.add(new artisynth.core.mechmodels.JointCoordinateHandle(jb, i));
                    }
                }
            }
        }

        return out;
    }

    private VectorNd stackWrenches(List<String> springNames, Map<String, double[]> wmap) {
        VectorNd v = new VectorNd(6 * springNames.size());
        int r = 0;
        for (String sname : springNames) {
            double[] w = wmap.get(sname);
            for (int i = 0; i < 6; i++) v.set(r++, w[i]);
        }
        return v;
    }

    @SuppressWarnings("unused")
    private double[] getExcitations(List<ExcitationComponent> xs) {
        double[] a = new double[xs.size()];
        for (int i = 0; i < xs.size(); i++) a[i] = xs.get(i).getExcitation();
        return a;
    }

    private double[] setExcitations(List<? extends ExcitationComponent> xs, double val) {
        double[] a = new double[xs.size()];
        for (int i = 0; i < xs.size(); i++) {
            a[i] = xs.get(i).getExcitation();
            xs.get(i).setExcitation(val);
        }
        return a;
    }

    private void restoreExcitations(List<? extends ExcitationComponent> xs, double[] vals) {
        for (int i = 0; i < xs.size(); i++) xs.get(i).setExcitation(vals[i]);
    }

    private void setExcitation(ExcitationComponent ex, double val) { ex.setExcitation(val); }

    private void applyExcitations(List<ExcitationComponent> exciters, List<FrameExciter> reserves, VectorNd x) {
        int idx = 0;
        for (ExcitationComponent ex : exciters) ex.setExcitation(x.get(idx++));
        for (FrameExciter ex : reserves) ex.setExcitation(x.get(idx++));
    }

    /**
     * Write results to CSV files in a simple layout.
     */
    public void writeResultCSVs(String dir, StaticOptimizationResult res) throws IOException {
        try (FileWriter wf = new FileWriter(dir + "/achieved_wrenches.csv")) {
            wf.write("spring,Fx,Fy,Fz,Mx,My,Mz\n");
            for (Map.Entry<String,double[]> e : res.achievedWrenches.entrySet()) {
                double[] w = e.getValue();
                wf.write(String.format("%s,%.8g,%.8g,%.8g,%.8g,%.8g,%.8g\n", e.getKey(), w[0],w[1],w[2],w[3],w[4],w[5]));
            }
        }
        try (FileWriter wf = new FileWriter(dir + "/target_wrenches.csv")) {
            wf.write("spring,Fx,Fy,Fz,Mx,My,Mz\n");
            for (Map.Entry<String,double[]> e : res.targetWrenches.entrySet()) {
                double[] w = e.getValue();
                wf.write(String.format("%s,%.8g,%.8g,%.8g,%.8g,%.8g,%.8g\n", e.getKey(), w[0],w[1],w[2],w[3],w[4],w[5]));
            }
        }
        try (FileWriter af = new FileWriter(dir + "/activations.csv")) {
            af.write("name,activation\n");
            for (Map.Entry<String,Double> e : res.activationsByName.entrySet()) {
                af.write(String.format("%s,%.8g\n", e.getKey(), e.getValue()));
            }
        }
        try (FileWriter ff = new FileWriter(dir + "/forces.csv")) {
            ff.write("name,force\n");
            for (Map.Entry<String,Double> e : res.forcesByName.entrySet()) {
                ff.write(String.format("%s,%.8g\n", e.getKey(), e.getValue()));
            }
        }
    }

    // Simple adapter cost/constraint terms for QPSolver
    private static class SimpleQuadraticCost extends QPCostTermBase {
        private final MatrixNd Q; private final VectorNd p;
        SimpleQuadraticCost(MatrixNd Q, VectorNd p) { this.Q = new MatrixNd(Q); this.p = new VectorNd(p); }
        @Override public void getQP(MatrixNd Qout, VectorNd pout, double t0, double t1) { Qout.add(Q); pout.add(p); }
    }

    private static class SimpleEqualityConstraint extends QPConstraintTermBase {
        private final MatrixNd A; private final VectorNd b;
        SimpleEqualityConstraint(MatrixNd A, VectorNd b) { this.A = new MatrixNd(A); this.b = new VectorNd(b); }
        @Override public int numConstraints(int qpsize) { return A.rowSize(); }
        @Override public int getTerm(MatrixNd Aeq, VectorNd beq, int rowoff, double t0, double t1) {
            Aeq.setSubMatrix(rowoff, 0, A); beq.setSubVector(rowoff, b); return A.rowSize();
        }
        @Override public Type getType() { return Type.EQUALITY; }
    }

    private static class SimpleInequalityConstraint extends QPConstraintTermBase {
        private final MatrixNd A; private final VectorNd b;
        SimpleInequalityConstraint(MatrixNd A, VectorNd b) { this.A = new MatrixNd(A); this.b = new VectorNd(b); }
        @Override public int numConstraints(int qpsize) { return A.rowSize(); }
        @Override public int getTerm(MatrixNd Aout, VectorNd bout, int rowoff, double t0, double t1) {
            Aout.setSubMatrix(rowoff, 0, A); bout.setSubVector(rowoff, b); return A.rowSize();
        }
        @Override public Type getType() { return Type.INEQUALITY; }
    }
}


