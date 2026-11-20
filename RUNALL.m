%% ========================================================================
% 2103-322 Design of Mechanism - Symbolic Mathematics & Synthesis
% Mechanism: 6-Bar Parallel Gripper (Symmetric Upper & Lower Halves)
% File: RUN_ALL.m
% =========================================================================
clear; clc; close all;


if ~exist('results/main', 'dir'),  mkdir('results/main');  end
if ~exist('results/bonus', 'dir'), mkdir('results/bonus'); end

%% ========================================================================
% PART 1: SYMBOLIC KINEMATIC ANALYSIS (Loop-Closure, Velocity, Acceleration)
% =========================================================================
fprintf('>>> [1/3] Running Symbolic Kinematic Analysis...\n');

syms L3 L4a L4b Lp beta gamma xC yC xO6 yO6 yA real
syms t real
syms s(t) th3(t) th4(t)
syms s_sym th3_sym th4_sym vs w3 w4 as a3 a4 real

% 1.1 Position Loop-Closure Equations (Upper Half - Loop 1)
eq1_x = s(t) + L3*cos(th3(t)) - xC - L4a*cos(th4(t)) == 0;
eq1_y = yA   + L3*sin(th3(t)) - yC - L4a*sin(th4(t)) == 0;

syms dxP dyP real
Px(t) = xC + L4b*cos(th4(t) + beta) + dxP;
Py(t) = yC + L4b*sin(th4(t) + beta) + dyP;

% 1.2 Velocity Analysis 
old_v = [diff(s(t),t), diff(th3(t),t), diff(th4(t),t), s(t), th3(t), th4(t)];
new_v = [vs, w3, w4, s_sym, th3_sym, th4_sym];

veq1_x = subs(diff(eq1_x, t), old_v, new_v);
veq1_y = subs(diff(eq1_y, t), old_v, new_v);

sol_w  = solve([veq1_x, veq1_y], [w3, w4]);
w3_sym = simplify(sol_w.w3);
w4_sym = simplify(sol_w.w4);


Vy_sym = simplify(subs(subs(diff(Py(t), t), old_v, new_v), w4, w4_sym));

% 1.3 Acceleration Analysis 
old_a = [diff(s(t),t,2), diff(th3(t),t,2), diff(th4(t),t,2), old_v];
new_a = [as, a3, a4, new_v];

aeq1_x = subs(diff(eq1_x, t, 2), old_a, new_a);
aeq1_y = subs(diff(eq1_y, t, 2), old_a, new_a);

sol_a  = solve([aeq1_x, aeq1_y], [a3, a4]);
a4_sym = simplify(sol_a.a4);

Ay_sym = simplify(subs(diff(Py(t), t, 2), old_a, new_a));

fprintf('Symbolic omega_4 =\n'); pretty(w4_sym);
fprintf('Symbolic Tip Velocity Vy =\n'); pretty(Vy_sym);


sym_model.f_w3 = matlabFunction(w3_sym, 'Vars', [L3, L4a, th3_sym, th4_sym, vs]);
sym_model.f_w4 = matlabFunction(w4_sym, 'Vars', [L3, L4a, th3_sym, th4_sym, vs]);
sym_model.f_a4 = matlabFunction(a4_sym, 'Vars', [L3, L4a, th3_sym, th4_sym, w3, w4, as]);
sym_model.f_Vy = matlabFunction(Vy_sym, 'Vars', [L3, L4a, L4b, beta, th3_sym, th4_sym, vs]);
sym_model.f_Ay = matlabFunction(Ay_sym, 'Vars', [L4b, beta, th4_sym, w4, a4]);

%% ========================================================================
% PART 2: MAIN SIMULATION (Standard CAD Geometry)
% =========================================================================
fprintf('>>> [2/3] Running Main Simulation & Rendering Video...\n');


p_main.xC   = 0;      p_main.yC   = 35;
p_main.xO6  = -18;    p_main.yO6  = 15;
p_main.yA   = 18;
p_main.L3   = 25;
p_main.L4a  = 22;
p_main.L4b  = 55;
p_main.beta = deg2rad(230);   % มุมงอ Link 4 
p_main.dxP  = -45;            % ระยะปลายปากคีบ P จากจุด D (แกน X)
p_main.dyP  = -12;            % ระยะปลายปากคีบ P จากจุด D (แกน Y)
p_main.s1   = 15;             % ตำแหน่ง Slider เริ่มต้น (mm)
p_main.s2   = 33;             % ตำแหน่ง Slider สิ้นสุด (mm)
p_main.vs   = 10;             % ความเร็ว Slider คงที่ (mm/s)
p_main.as   = 0;              % ความเร่ง Slider (mm/s^2)
p_main.title = 'Main: 6-Bar Parallel Gripper';

run_gripper_sim(p_main, sym_model, ...
    'results/main/Render.jpg', 'results/main/Mechanism.avi');

%% ========================================================================
% PART 3: BONUS - ANALYTICAL TWO-POSITION SYNTHESIS
% =========================================================================
fprintf('>>> [3/3] Running Bonus Synthesis & Rendering Video...\n');

% โจทย์ Synthesis: ต้องการออกแบบระยะอ้าปากคีบ (Jaw Opening = 2*Py)
% จากเปิดสุด 105 mm ลดลงเหลือปิดสุด 45 mm ด้วยระยะชัก Slider (ds) = 22 mm
% และตรวจสอบเงื่อนไข Transmission Angle ให้อยู่ในช่วงปลอดภัย [40, 140] องศา
syms s1_syn L3_syn xB1 yB1 xB2 yB2 yA_s ds_s real
eq_s1 = (xB1 - s1_syn)^2 + (yB1 - yA_s)^2 == L3_syn^2;
eq_s2 = (xB2 - (s1_syn + ds_s))^2 + (yB2 - yA_s)^2 == L3_syn^2;
sol_syn = solve([eq_s1, eq_s2], [s1_syn, L3_syn]);

p_bonus = p_main;
p_bonus.ds = 22; % ระยะชักเป้าหมาย (mm)

% หามุม th4 เป้าหมายจากความสูงปลายปากคีบที่ต้องการ (Py = Opening / 2)
Py_open  = 105 / 2; % 52.5 mm
Py_close = 45  / 2; % 22.5 mm

% Py = yC + L4b*sin(th4 + beta) + dyP  =>  sin(th4 + beta) = (Py - yC - dyP)/L4b
ang_CD_open  = pi - asin((Py_open  - p_bonus.yC - p_bonus.dyP) / p_bonus.L4b);
ang_CD_close = pi - asin((Py_close - p_bonus.yC - p_bonus.dyP) / p_bonus.L4b);
th4_open  = ang_CD_open  - p_bonus.beta;
th4_close = ang_CD_close - p_bonus.beta;

% พิกัดจุดหมุน B ทั้ง 2 ตำแหน่ง
num_xB1 = p_bonus.xC + p_bonus.L4a*cos(th4_open);
num_yB1 = p_bonus.yC + p_bonus.L4a*sin(th4_open);
num_xB2 = p_bonus.xC + p_bonus.L4a*cos(th4_close);
num_yB2 = p_bonus.yC + p_bonus.L4a*sin(th4_close);

% แทนค่าในสมการ Symbolic Synthesis
vars_s = [xB1, yB1, xB2, yB2, yA_s, ds_s];
vals_s = [num_xB1, num_yB1, num_xB2, num_yB2, p_bonus.yA, p_bonus.ds];

p_bonus.s1 = double(subs(sol_syn.s1_syn(1), vars_s, vals_s));
p_bonus.s2 = p_bonus.s1 + p_bonus.ds;
p_bonus.L3 = abs(double(subs(sol_syn.L3_syn(2), vars_s, vals_s)));
p_bonus.title = sprintf('Synthesis: 105 -> 45 mm | L3 = %.2f mm', p_bonus.L3);

fprintf('--- Synthesis Results ---\n');
fprintf('  Target Jaw Opening : 105.0 mm -> 45.0 mm\n');
fprintf('  Synthesized s1     : %.3f mm (s2 = %.3f mm)\n', p_bonus.s1, p_bonus.s2);
fprintf('  Synthesized L3     : %.3f mm\n', p_bonus.L3);

run_gripper_sim(p_bonus, sym_model, ...
    'results/bonus/Synthesis_Render.jpg', 'results/bonus/Synthesis.avi');

fprintf('>>> ALL DONE! Check results/main/ and results/bonus/\n');

%% ========================================================================
% LOCAL FUNCTION: SOLVER, ANIMATION & VIDEO EXPORT
% =========================================================================
function run_gripper_sim(p, sym_model, img_path, vid_path)
    N = 60;
    s_arr = linspace(p.s1, p.s2, N);
    th3_arr = zeros(1,N); th4_arr = zeros(1,N);
    Py_arr  = zeros(1,N); Vy_arr  = zeros(1,N); Ay_arr = zeros(1,N);
    mu_arr  = zeros(1,N);

    guess = [deg2rad(-170), deg2rad(-110)];
    opts  = optimoptions('fsolve', 'Display', 'off');

    for i = 1:N
        sc = s_arr(i);
        fun = @(x) [sc + p.L3*cos(x(1)) - p.xC - p.L4a*cos(x(2));
                    p.yA + p.L3*sin(x(1)) - p.yC - p.L4a*sin(x(2))];
        sol = fsolve(fun, guess, opts);
        guess = sol;
        t3 = sol(1); t4 = sol(2);
        th3_arr(i) = t3; th4_arr(i) = t4;

        w3_c = sym_model.f_w3(p.L3, p.L4a, t3, t4, p.vs);
        w4_c = sym_model.f_w4(p.L3, p.L4a, t3, t4, p.vs);
        a4_c = sym_model.f_a4(p.L3, p.L4a, t3, t4, w3_c, w4_c, p.as);

        Py_arr(i) = 2 * (p.yC + p.L4b*sin(t4 + p.beta) + p.dyP); % ระยะอ้าปากคีบรวม (บน-ล่าง)
        Vy_arr(i) = 2 * sym_model.f_Vy(p.L3, p.L4a, p.L4b, p.beta, t3, t4, p.vs);
        Ay_arr(i) = 2 * sym_model.f_Ay(p.L4b, p.beta, t4, w4_c, a4_c);
        mu_arr(i) = abs(rad2deg(atan2(sin(t4 - t3), cos(t4 - t3))));
    end

    fprintf('  Transmission Angle Range: [%.2f deg, %.2f deg]\n', min(mu_arr), max(mu_arr));

    % ตั้งค่า Figure (ซ้าย: กลไก, ขวา: กราฟ 3 ชั้น s/v/a)
    fig = figure('Color', 'w', 'Position', [50, 50, 1100, 620]);
    theme(fig, 'light');
    v = VideoWriter(vid_path);
    v.FrameRate = 20;
    open(v);

    for i = 1:N
        clf(fig);
        sc = s_arr(i); t3 = th3_arr(i); t4 = th4_arr(i);

        % คำนวณพิกัดจุดหมุนครึ่งบน
        A  = [sc, p.yA];
        C  = [p.xC, p.yC];
        O6 = [p.xO6, p.yO6];
        B  = C  + [p.L4a*cos(t4), p.L4a*sin(t4)];
        D  = C  + [p.L4b*cos(t4 + p.beta), p.L4b*sin(t4 + p.beta)];
        E  = O6 + [p.L4b*cos(t4 + p.beta), p.L4b*sin(t4 + p.beta)];
        P  = D  + [p.dxP, p.dyP];

        % --- ฝั่งซ้าย: วาดกลไก (บน + ล่างสมมาตร) ---
        subplot(3, 2, [1 3 5]); hold on; grid on; axis equal;
        xlim([-115, 60]); ylim([-95, 95]);
        xlabel('x (mm)'); ylabel('y (mm)');
        title(sprintf('%s\ns = %.1f mm | Opening = %.1f mm', p.title, sc, Py_arr(i)));

        % แกนสกรูและ Slider
        plot([-20, 55], [0, 0], 'k--', 'LineWidth', 1.5);
        rectangle('Position', [sc-6, -p.yA-4, 12, 2*(p.yA+4)], ...
                  'FaceColor', [0.85 0.90 0.95], 'EdgeColor', [0.2 0.3 0.5], 'LineWidth', 1.5);

        % วาดทั้งครึ่งบน (sgn = +1) และครึ่งล่าง (sgn = -1)
        for sgn = [1, -1]
            % ปากคีบ (Jaw Polygon)
            jaw_x = [D(1), E(1), P(1), P(1)+12];
            jaw_y = sgn * [D(2), E(2), P(2), D(2)-6];
            fill(jaw_x, jaw_y, [0.75 0.87 0.95], 'EdgeColor', [0.1 0.25 0.45], 'LineWidth', 1.5);

            % ลิงก์ต่างๆ
            plot([A(1) B(1)], sgn*[A(2) B(2)], 'k-', 'LineWidth', 2.5);             % Link 3
            plot([B(1) C(1) D(1)], sgn*[B(2) C(2) D(2)], 'b-', 'LineWidth', 3);     % Link 4
            plot([O6(1) E(1)], sgn*[O6(2) E(2)], 'Color', [0.4 0.4 0.7], 'LineWidth', 2.5); % Link 5

            % จุดหมุน
            pts_x = [A(1), B(1), C(1), D(1), O6(1), E(1)];
            pts_y = sgn * [A(2), B(2), C(2), D(2), O6(2), E(2)];
            plot(pts_x, pts_y, 'ko', 'MarkerFaceColor', 'w', 'MarkerSize', 5);
            plot([C(1), O6(1)], sgn*[C(2), O6(2)], 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 6);
            plot(P(1), sgn*P(2), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 5);
        end

        % --- ฝั่งขวา: กราฟ 3 ชั้น (Displacement, Velocity, Acceleration) ---
        subplot(3, 2, 2);
        plot(s_arr, Py_arr, 'Color', [0.1 0.35 0.6], 'LineWidth', 1.8); hold on; grid on;
        plot(sc, Py_arr(i), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
        title('Gripper Jaw Opening (2P_y)'); xlabel('Slider s (mm)'); ylabel('Opening (mm)');

        subplot(3, 2, 4);
        plot(s_arr, Vy_arr, 'Color', [0.1 0.35 0.6], 'LineWidth', 1.8); hold on; grid on;
        plot(sc, Vy_arr(i), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
        title('Jaw Closing Velocity (2V_y)'); xlabel('Slider s (mm)'); ylabel('Velocity (mm/s)');

        subplot(3, 2, 6);
        plot(s_arr, Ay_arr, 'Color', [0.1 0.35 0.6], 'LineWidth', 1.8); hold on; grid on;
        plot(sc, Ay_arr(i), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
        title('Jaw Closing Acceleration (2A_y)'); xlabel('Slider s (mm)'); ylabel('Accel (mm/s^2)');

        drawnow;
        writeVideo(v, getframe(fig));

        % บันทึกภาพ Render.jpg ที่เฟรมกึ่งกลาง
        if i == round(N/2)
            exportgraphics(fig, img_path, 'Resolution', 200);
        end
    end
    close(v);
end