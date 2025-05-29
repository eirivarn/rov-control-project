% test_hover_allDOF_verbose.m
clear; clc; close all

%% 1) Build linear hover model
[sys, A, B, C, ~] = rovHoverModel();
n = size(A,1);

%% 2) Stability & modal info
lambda = eig(A);
[wn,zeta,poles] = damp(sys);

fprintf('\n=== Stability & Modal Analysis ===\n');
fprintf('Total states: %d\n', n);
fprintf('Eigenvalues (mode#: Re, Im):\n');
for i = 1:length(lambda)
    fprintf('  %2d: %+8.4e %+8.4e j\n', i, real(lambda(i)), imag(lambda(i)));
end
maxRe = max(real(lambda));
fprintf('Max Re(eig) = %+8.4e → %s\n', maxRe, ternary(maxRe<0,'asymptotically stable','marginal/unstable'));

fprintf('\nNatural freqs (rad/s) and damping ratios:\n');
for i = 1:length(wn)
    fprintf('  %2d: wn=%6.3f   zeta=%5.3f\n', i, wn(i), zeta(i));
end

%% 3) Controllability & Observability
Wc = ctrb(A,B);
Wo = obsv(A,C);
rankWc = rank(Wc);
rankWo = rank(Wo);
svWc = svd(Wc);
svWo = svd(Wo);
fprintf('\n=== Gramian Checks ===\n');
fprintf('Controllability: rank = %d/%d,  min(sv) = %8.4e\n', rankWc, n, min(svWc));
fprintf('Observability:   rank = %d/%d,  min(sv) = %8.4e\n', rankWo, n, min(svWo));

%% 4) Simulate a 0.1‐unit step in all 6 DOF
Tend = 10; dt = 0.01; t = (0:dt:Tend)';
U    = 0.1 * ones(6, numel(t));
Ylin = lsim(sys, U', t);

% Nonlinear integration
Xnl = zeros(12, numel(t));
for k = 1:length(t)-1
    eta = Xnl(1:6,k);
    nu  = Xnl(7:12,k);
    tau = U(:,k);
    nud = computeDynamicsSimple(nu, tau, eta);
    Xdot = [nu; nud];
    Xnl(:,k+1) = Xnl(:,k) + dt * Xdot;
end
Ynl = Xnl';

%% 5) Per‐DOF error peaks & bound
L = 400;
E = abs(Ynl(:,1:6) - Ylin(:,1:6));    % per-DOF abs error
xn2 = vecnorm(Ynl(:,1:6),2,2).^2;
bound = 0.5 * L * cumtrapz(t, xn2);

fprintf('\n=== Error Analysis ===\n');
fprintf('%6s  %12s  %12s  %12s  %8s\n','DOF','MaxErr','Time [s]','Bound@t_max','Err/Bound');
dofNames = {'x','y','z','phi','theta','psi'};
for i = 1:6
    [maxErr, idx] = max(E(:,i));
    tb = bound(idx);
    ratio = maxErr/tb;
    fprintf(' %6s : %8.4e  %8.2f     %8.4e    %5.2f%%\n', ...
        dofNames{i}, maxErr, t(idx), tb, ratio*100);
end
% combined norm
Eall = vecnorm(Ynl - Ylin,2,2);
[maxEall, idxAll] = max(Eall);
tbAll = bound(idxAll);
fprintf('\n Combined max norm = %8.4e at t=%.2fs\n', maxEall, t(idxAll));
fprintf(' Bound@that time = %8.4e   Err/Bound = %5.2f%%\n', tbAll, 100*maxEall/tbAll);

%% helper
function out = ternary(c,a,b)
    if c, out = a; else out = b; end
end
