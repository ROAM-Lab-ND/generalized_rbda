function [fwd_dyn, inv_dyn] = derive_revolute_with_rotor(N)
    import casadi.*

    % Define symbolic variables
    q = SX.sym('q', N);
    dq = SX.sym('dq', N);
    ddq = SX.sym('ddq', N);
    tau = SX.sym('tau', N);

    % Define parameters
    I = zeros(N, 1);
    Irot = zeros(N, 1);
    m = zeros(N, 1);
    l = zeros(N, 1);
    c = zeros(N, 1);
    gr = zeros(N, 1);
    br = zeros(N, 1);
    g = -9.81;

    for n = 1:N
        I(n) = 1.;
        Irot(n) = 0.0001;
        m(n) = 1.;
        l(n) = 1.;
        c(n) = 0.5;
        gr(n) = 2;
        br(n) = 3;
    end

    % Helpful anonymous functions
    ddt = @(x) jacobian(x, q) * dq;
    F2Q = @(F, r) simplify(jacobian(r, q)' * (F));
    M2Q = @(M, w) simplify(jacobian(w, dq)' * (M));

    % Lagrangian
    ehat = cell(N, 1);
    net_angle = casadi.SX(0);

    for n = 1:N
        net_angle = net_angle + q(n);
        ehat{n} = [cos(net_angle) sin(net_angle) 0]';
    end

    ghat = [-1 0 0]';

    rc = cell(N, 1);
    r = cell(N, 1);
    vc = cell(N, 1);
    omega = casadi.SX(zeros(N, 1));
    omega_rotor = casadi.SX(zeros(N, 1));
    omega_rotor_approx = casadi.SX(zeros(N, 1));
    T_link = casadi.SX(0);
    T_rotor = casadi.SX(0);
    T_rotor_approx = casadi.SX(0);
    V = casadi.SX(0);
    Q = casadi.SX(0);

    for n = 1:N

        if (n == 1)
            rc{n} = c(n) * ehat{n};
            r{n} = l(n) * ehat{n};

            omega(n) = dq(n);
            omega_rotor(n) = gr(n) * br(n) * dq(n);

        else
            rc{n} = r{n - 1} + c(n) * ehat{n};
            r{n} = r{n - 1} + l(n) * ehat{n};

            omega(n) = omega(n - 1) + dq(n);
            omega_rotor(n) = omega(n - 1) + gr(n) * br(n) * dq(n);

        end

        vc{n} = ddt(rc{n});
        omega_rotor_approx(n) = gr(n) * br(n) * dq(n);

        T_link = T_link + m(n) * (vc{n}' * vc{n}) + I(n) * omega(n) ^ 2;
        T_rotor = T_rotor + Irot(n) * omega_rotor(n) ^ 2;
        T_rotor_approx = T_rotor_approx + Irot(n) * omega_rotor_approx(n) ^ 2;
        V = V + m(n) * g * dot(rc{n}, -ghat);

        Q = Q + M2Q(tau(n), dq(n));

    end

    T_link = 0.5 * T_link;
    T_rotor = 0.5 * T_rotor;
    T_rotor_approx = 0.5 * T_rotor_approx;

    T = T_link + T_rotor;
    T_approx = T_link + T_rotor_approx;

    L = simplify(T - V);
    L_approx = simplify(T_approx - V);

    % Symbolic forward dynamics
    fd_exact = computeSymbolicForwardDynamics(q, dq, L, Q);
    fd_rf = computeSymbolicForwardDynamics(q, dq, L_approx, Q);
    fd_rf_diag = computeSymbolicForwardDynamics(q, dq, L_approx, Q);

    % Casadi Functions
    args = {q, dq, tau};
    name = ['RevWithRotors', num2str(N), 'DofFwdDyn'];
    fwd_dyn.exact = Function(name, args, {fd_exact});
    fwd_dyn.rf = Function([name, 'ReflectedInertia'], args, {fd_rf});
    fwd_dyn.rf_diag = Function([name, 'ReflectedInertiaDiag'], args, {fd_rf_diag});

    % Symbolic inverse dynamics
    id_exact = computeSymbolicInverseDynamics(q, dq, ddq, L);
    id_rf = computeSymbolicInverseDynamics(q, dq, ddq, L_approx);
    id_rf_diag = computeSymbolicInverseDynamics(q, dq, ddq, L_approx);

    % Casadi Functions
    args = {q, dq, ddq};
    name = ['RevWithRotors', num2str(N), 'DofInvDyn'];
    inv_dyn.exact = Function(name, args, {id_exact});
    inv_dyn.rf = Function([name, 'ReflectedInertia'], args, {id_rf});
    inv_dyn.rf_diag = Function([name, 'ReflectedInertiaDiag'], args, {id_rf_diag});

end

%% Helper functions
function fwd_dyn = computeSymbolicForwardDynamics(q, dq, L, Q)
    dL_dq = jacobian(L, q)';
    dL_dqd = jacobian(L, dq)';

    H = hessian(L, dq);
    C = jacobian(dL_dqd, q) * dq - dL_dq - Q;

    fwd_dyn = H \ (-C);

end

function inv_dyn = computeSymbolicInverseDynamics(q, dq, ddq, L)
    dL_dq = jacobian(L, q)';
    dL_dqd = jacobian(L, dq)';

    H = hessian(L, dq);
    C = jacobian(dL_dqd, q) * dq - dL_dq;

    inv_dyn = H * ddq + C;

end
