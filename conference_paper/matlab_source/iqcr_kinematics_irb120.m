function iqcr_kinematics_irb120
% IQCR - IRB 120 forward/inverse kinematics verification, corrected DH table
%
% Companion to iqcr_kinematics.m, which is kept unchanged because it produced
% the hand-coded table in the published paper. That script pairs the standard
% DH matrix A_i = Rz(theta_i) * Tz(d_i) * Tx(a_i) * Rx(alpha_i) with the twist
% list [0 -90 0 -90 90 -90], which is shifted by one joint, and has no joint-2
% offset. Its table therefore describes a six-axis arm with the IRB 120's link
% lengths but not its joint layout (q = 0 puts the TCP about 340 mm off the
% base's symmetry plane, at shoulder height).
%
% This file uses the IRB 120 table that MATLAB's Robotics System Toolbox
% exports for loadrobot('abbIrb120') (see ikIRB120.m, dhParams and
% thetaOffsets): alpha = [-90 0 -90 90 -90 0] deg, a joint-2 offset of -90 deg,
% and the datasheet joint limits. With it q = 0 is the upright pose with the
% TCP at (374, 0, 630) mm. The Robotics System Toolbox results in the paper
% (rst_verification*.m) used ABB's own model and are unaffected.
%
% Solver, seeds, tolerances and figure recipes are otherwise identical, so the
% outputs are the like-for-like rerun on the corrected arm. Output files carry
% an _irb120 suffix so the archived paper outputs are never overwritten.
clear all; clc; rng(7);

a      = [0   270  70   0    0   0];          % mm
alpha  = deg2rad([-90  0  -90  90  -90  0]);  % rad
d      = [290  0    0   302   0  72];         % mm
offset = deg2rad([0  -90  0  0  0  0]);       % rad, added to theta before the DH matrix

n = 6;

% joint limits (deg), ABB IRB 120 datasheet, same values the toolbox model carries
qmin = deg2rad([-165 -110 -110 -160 -120 -400]);
qmax = deg2rad([ 165  110   70  160  120  400]);

N = 8;
results = zeros(N,4); % [pos_err_mm, orient_err_deg, iterations, converged]
errHistories = cell(N,1);

for k = 1:N
    q_true = qmin + rand(1,n).*(qmax-qmin);
    T_target = fkine(q_true, a, alpha, d, offset);

    % initial guess: perturb the true solution (numerical IK, damped Gauss-Newton)
    q = q_true + deg2rad(15)*(rand(1,n)-0.5);

    lambda = 1e-3;
    maxIter = 200;
    tol = 1e-9;
    errHist = zeros(maxIter,1);
    iter = 0;
    for it = 1:maxIter
        T_cur = fkine(q, a, alpha, d, offset);
        posErr = T_target(1:3,4) - T_cur(1:3,4);
        Rerr = T_target(1:3,1:3) * T_cur(1:3,1:3)';
        % axis-angle (log map) orientation error, small-angle safe
        cosang = (trace(Rerr)-1)/2;
        cosang = min(1,max(-1,cosang));
        ang = acos(cosang);
        if (ang < 1e-9)
            oriErr = [0;0;0];
        else
            rot_axis = (1/(2*sin(ang))) * [Rerr(3,2)-Rerr(2,3); Rerr(1,3)-Rerr(3,1); Rerr(2,1)-Rerr(1,2)];
            oriErr = ang*rot_axis;
        end
        e = [posErr; oriErr];
        errHist(it) = norm(posErr);
        iter = it;
        if (norm(e) < tol)
            break
        end
        % numerical Jacobian via central finite differences
        J = zeros(6,n);
        h = 1e-6;
        for j = 1:n
            qp = q; qp(j) = qp(j)+h;
            qm = q; qm(j) = qm(j)-h;
            Tp = fkine(qp, a, alpha, d, offset); Tm = fkine(qm, a, alpha, d, offset);
            dpos = (Tp(1:3,4)-Tm(1:3,4))/(2*h);
            Rp = Tp(1:3,1:3); Rm = Tm(1:3,1:3);
            dR = (Rp-Rm)/(2*h);
            % skew from dR*R' approx angular velocity column
            S = dR*Tm(1:3,1:3)';
            w = [S(3,2); S(1,3); S(2,1)];
            J(:,j) = [dpos; w];
        end
        dq = (J'*J + lambda*eye(n)) \ (J'*e);
        q = q + dq';
    end
    T_final = fkine(q, a, alpha, d, offset);
    posErrFinal = norm(T_target(1:3,4)-T_final(1:3,4));
    Rerr = T_target(1:3,1:3)*T_final(1:3,1:3)';
    cosang = min(1,max(-1,(trace(Rerr)-1)/2));
    oriErrFinal = rad2deg(acos(cosang));

    results(k,:) = [posErrFinal, oriErrFinal, iter, posErrFinal<1e-3];
    errHistories{k} = errHist(1:iter);
end

T = array2table(results, 'VariableNames', {'PosErr_mm','OriErr_deg','Iterations','Converged'});
T.Case = (1:N)';
T = T(:, {'Case','PosErr_mm','OriErr_deg','Iterations','Converged'});
disp(T)
writetable(T, 'ik_verification_results_irb120.csv');

% Figure: position error per test case (log scale)
f1 = figure('Visible','off','Position',[100 100 500 350]);
semilogy(1:N, max(results(:,1),1e-12), 'ko-', 'LineWidth', 1.3, 'MarkerFaceColor','k');
xlabel('Test pose'); ylabel('Final position error (mm)');
title('IK convergence error across randomized target poses');
grid on; xlim([0.5 N+0.5]);
saveas(f1, 'fig_ik_error_irb120.png');

% Figure: iterations to converge per test case
f2 = figure('Visible','off','Position',[100 100 500 350]);
bar(1:N, results(:,3), 'FaceColor',[0.3 0.3 0.3]);
xlabel('Test pose'); ylabel('Newton-Raphson iterations to converge');
title('Iterations required for numerical IK convergence');
grid on;
saveas(f2, 'fig_ik_iterations_irb120.png');

% Figure: reachable workspace envelope in X-Z plane (base joint = 0)
th2 = linspace(qmin(2), qmax(2), 60);
th3 = linspace(qmin(3), qmax(3), 60);
X = []; Z = [];
for i = 1:numel(th2)
    for j = 1:numel(th3)
        q = [0, th2(i), th3(j), 0, 0, 0];
        Tw = fkine(q, a, alpha, d, offset);
        X(end+1) = Tw(1,4); %#ok<AGROW>
        Z(end+1) = Tw(3,4); %#ok<AGROW>
    end
end
f3 = figure('Visible','off','Position',[100 100 500 420]);
plot(X, Z, '.', 'MarkerSize', 3, 'Color',[0.2 0.2 0.2]);
xlabel('X (mm)'); ylabel('Z (mm)');
title('Reachable workspace envelope, X-Z plane (base joint = 0)');
axis equal; grid on;
saveas(f3, 'fig_workspace_irb120.png');

fprintf('\nMean position error: %.3e mm\n', mean(results(:,1)));
fprintf('Max position error: %.3e mm\n', max(results(:,1)));
fprintf('Mean iterations: %.1f\n', mean(results(:,3)));
fprintf('All converged (<1e-3 mm): %d / %d\n', sum(results(:,4)), N);

end

function T = fkine(q, a, alpha, d, offset)
% forward kinematics: chain the DH transform of each joint together to
% get the end-effector pose T for joint angles q (ABB joint coordinates;
% the fixed offset moves joint 2 to the DH zero)
    T = eye(4);
    for i = 1:numel(q)
        T = T * dhtransform(q(i) + offset(i), d(i), a(i), alpha(i));
    end
end

function T = dhtransform(theta, d, a, alpha)
% single-link Denavit-Hartenberg transform, standard convention:
% A_i = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
    T = [cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta); ...
         sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta); ...
         0            sin(alpha)             cos(alpha)            d; ...
         0            0                      0                     1];
end
