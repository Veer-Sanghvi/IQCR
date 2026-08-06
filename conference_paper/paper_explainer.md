# What this paper actually says (undergrad-level walkthrough)

## The one-sentence version
We propose putting a camera, a depth sensor, and a laser on one robot arm instead of using separate inspection stations, and we prove — with real MATLAB code, not just a claim — that the arm can actually reach the angles it would need to.

## The problem we're solving
Factories check parts two ways: cameras look for scratches/dents (vision), and probes/CMMs measure exact dimensions (contact). Today these are usually two different stations. Our pitch: put both sensor types on one robot arm, so a part gets fully checked without moving between stations.

## What's actually new here (be honest about this if asked)
Nothing about the *inspection* logic (the CNN, the sensor fusion, the CAD tolerance comparison) is verified with real numbers — that part is proposed methodology, described but not built or tested. **The genuinely new, computed-by-us contribution is narrower: we verified that a real six-axis arm (ABB IRB 120) can physically reach the poses this idea requires**, using MATLAB's Robotics System Toolbox with the manufacturer's own robot model, not numbers we typed in ourselves.

If someone asks "what did you actually contribute vs. what did you just propose?" — that's the honest answer. Say it that way, don't oversell it.

## What "forward kinematics" and "inverse kinematics" mean, concretely
- **Forward kinematics**: if you tell the robot "set your 6 joint angles to X," this tells you exactly where the hand (end effector) ends up in space. Easy, just matrix multiplication.
- **Inverse kinematics (IK)**: the harder, backwards question — "I want the camera at this exact spot and angle, what 6 joint angles get me there?" This is what a robot controller has to solve *every time* it moves to a new inspection viewpoint.

## What we did, step by step
1. Loaded ABB's real digital model of the IRB 120 arm into MATLAB (`loadrobot('abbIrb120')`) — this is the actual manufacturer data, not something we guessed.
2. Picked 8 random target poses (positions/angles) within the arm's real joint limits.
3. For each one, asked MATLAB's IK solver: "find me the joint angles that reach this pose."
4. Checked: did it find the right answer, and how hard did it have to work (how many iterations)?

## The results, and how to explain them if pressed
- **7 out of 8 worked essentially perfectly** — error around 0.00001 mm, solved in under 30 tries. That's far more precise than any inspection task would need.
- **1 out of 8 failed** — even after 1500 attempts, the solver landed 32.7 mm off target. This is not a bug in our code. General-purpose IK solvers use trial-and-error search (like rolling downhill to find the lowest point) and can get stuck if they start in a bad spot, similar to how you can get stuck in a valley that isn't the true lowest point on a hill.
- **Why we kept the failure in the paper instead of hiding it**: a paper claiming "8/8 perfect" would look suspicious to anyone who knows how these solvers behave — real solvers fail sometimes. Reporting the honest 7/8, and explaining *why* it failed, makes the paper more credible, not less.
- **What we recommend because of that failure**: use a proper "closed-form" solver instead — one that solves with algebra/geometry directly instead of trial-and-error, so it can't get stuck. We explain *why* that's possible for this arm (the last 3 joints all cross through one point, geometrically, which simplifies the math) but we didn't build that solver ourselves — we flagged it as the clear next step. If asked "why didn't you just do that," the honest answer is: it requires re-deriving the geometry to match the exact robot model, and doing that correctly under deadline pressure risked introducing an error, so we described it clearly as future work instead of rushing it.

## The "workspace envelope" figure — what it is and isn't
This is the region in space the robot's hand can actually reach, shown as a 2D slice (looking at it from the side). It's shaped like a **thick, curled ring** — not a full circle — because of the offset in the arm's forearm and wrist (it's geometrically off-center, so it can't fold back to reach points close to its own base).

**Be careful here**: this ring shape is not a discovery. Any robotics textbook would predict this shape for this type of arm. We present it as a *sanity check* — proof our digital model is behaving the way real physics says it should — not as a novel finding. If a reviewer asks "what's new about this figure," say exactly that: it confirms the model, it doesn't discover anything.

## Why we used the actual Robotics Toolbox model instead of typing in numbers ourselves
Early in this project we typed the arm's geometry (DH parameters) into MATLAB by hand from a published reference paper. That works, but it means trusting our own typing and a secondary source. Switching to `loadrobot('abbIrb120')` pulls the geometry directly from ABB's own official model, which is more credible and removes a source of possible error. If asked why the numbers changed partway through the project, that's why.

## Likely questions and how to answer them

**Q: Did you build a physical robot?**
No. Everything is simulation/computation. The paper says clearly that a physical benchtop or work-cell build, and validation against a coordinate measuring machine (CMM), is the next step, not something we've done.

**Q: What's a CMM and why does it matter here?**
A coordinate measuring machine is the current gold-standard way to precisely measure a part's dimensions with a physical probe. We'd compare our robot's measurements against CMM measurements on the same part to prove the robot is accurate enough to trust, not just self-consistent in simulation.

**Q: Why does one test case fail and should I be worried about that?**
No — explain it as I did above. It's expected behavior for general numerical solvers, we caught it and explained it rather than hiding it, and it directly motivates a specific, well-justified next step (closed-form IK).

**Q: Isn't the workspace ring shape just something you already knew?**
Yes, correct — and the paper says so. It's a validation step, not a discovery.

**Q: What would make this a stronger paper next time?**
Physical hardware test, the closed-form IK solver actually implemented and compared against the numerical one, and real inspection data (defect images, actual CAD tolerance comparisons) instead of only the kinematics being verified.
