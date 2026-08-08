function iqcr_kinematics
% IQCR - IRB 120 forward/inverse kinematics verification
% Standard DH convention: A_i = Rz(theta_i) * Tz(d_i) * Tx(a_i) * Rx(alpha_i)
% DH parameters taken from the project's Table 5 (ABB IRB 120 link dimensions, mm/deg)
clear all; clc; rng(7);

a     = [0   270  70   0    0   0];      % mm
alpha = deg2rad([0  -90   0  -90  90 -90]); % rad
d     = [290  0    0   302   0  72];     % mm

n = 6;

% joint limits (deg), conservative subset of IRB 120 published range
qmin = deg2rad([-150 -100  -90 -150 -100 -150]);
qmax = deg2rad([ 150  100   50  150  100  150]);

N = 8;
results = zeros(N,4); % [pos_err_mm, orient_err_deg, iterations, converged]
errHistories = cell(N,1);

for k = 1:N
    q_true = qmin + rand(1,n).*(qmax-qmin);
    T_target = fkine(q_true, a, alpha, d);

    % initial guess: perturb the true solution (numerical IK, damped Gauss-Newton)
    q = q_true + deg2rad(15)*(rand(1,n)-0.5);

    lambda = 1e-3;
    maxIter = 200;
    tol = 1e-9;
    errHist = zeros(maxIter,1);
    iter = 0;
    for it = 1:maxIter
        T_cur = fkine(q, a, alpha, d);
        posErr = T_target(1:3,4) - T_cur(1:3,4);
        Rerr = T_target(1:3,1:3) * T_cur(1:3,1:3)';
        % axis-angle (log map) orientation error, small-angle safe
        cosang = (trace(Rerr)-1)/2;
        cosang = min(1,max(-1,cosang));
        ang = acos(cosang);
        if (ang < 1e-9)
            oriErr = [0;0;0];
        else
            axis = (1/(2*sin(ang))) * [Rerr(3,2)-Rerr(2,3); Rerr(1,3)-Rerr(3,1); Rerr(2,1)-Rerr(1,2)];
            oriErr = ang*axis;
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
            Tp = fkine(qp, a, alpha, d); Tm = fkine(qm, a, alpha, d);
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
    T_final = fkine(q, a, alpha, d);
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
writetable(T, 'ik_verification_results.csv');

% Figure: position error per test case (log scale)
f1 = figure('Visible','off','Position',[100 100 500 350]);
semilogy(1:N, max(results(:,1),1e-12), 'ko-', 'LineWidth', 1.3, 'MarkerFaceColor','k');
xlabel('Test pose'); ylabel('Final position error (mm)');
title('IK convergence error across randomized target poses');
grid on; xlim([0.5 N+0.5]);
saveas(f1, 'fig_ik_error.png');

% Figure: iterations to converge per test case
f2 = figure('Visible','off','Position',[100 100 500 350]);
bar(1:N, results(:,3), 'FaceColor',[0.3 0.3 0.3]);
xlabel('Test pose'); ylabel('Newton-Raphson iterations to converge');
title('Iterations required for numerical IK convergence');
grid on;
saveas(f2, 'fig_ik_iterations.png');

% Figure: reachable workspace envelope in X-Z plane (base joint = 0)
th2 = linspace(qmin(2), qmax(2), 60);
th3 = linspace(qmin(3), qmax(3), 60);
X = []; Z = [];
for i = 1:numel(th2)
    for j = 1:numel(th3)
        q = [0, th2(i), th3(j), 0, 0, 0];
        Tw = fkine(q, a, alpha, d);
        X(end+1) = Tw(1,4); %#ok<AGROW>
        Z(end+1) = Tw(3,4); %#ok<AGROW>
    end
end
f3 = figure('Visible','off','Position',[100 100 500 420]);
plot(X, Z, '.', 'MarkerSize', 3, 'Color',[0.2 0.2 0.2]);
xlabel('X (mm)'); ylabel('Z (mm)');
title('Reachable workspace envelope, X-Z plane (base joint = 0)');
axis equal; grid on;
saveas(f3, 'fig_workspace.png');

fprintf('\nMean position error: %.3e mm\n', mean(results(:,1)));
fprintf('Max position error: %.3e mm\n', max(results(:,1)));
fprintf('Mean iterations: %.1f\n', mean(results(:,3)));
fprintf('All converged (<1e-3 mm): %d / %d\n', sum(results(:,4)), N);

end

function T = fkine(q, a, alpha, d)
% forward kinematics: chain the DH transform of each joint together to
% get the end-effector pose T for joint angles q
    T = eye(4);
    for i = 1:numel(q)
        T = T * dhtransform(q(i), d(i), a(i), alpha(i));
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
