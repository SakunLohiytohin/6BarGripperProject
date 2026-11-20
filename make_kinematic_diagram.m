%% ========================================================================
% Generate Publication-Quality Kinematic Diagram (Upper Half - 6-Bar Gripper)
% Output: Kinematic_Diagram.png (300 DPI)
% =========================================================================
clear; clc; close all;

fig = figure('Color', 'w', 'Position', [100, 100, 820, 560]);
theme(fig, 'light');
hold on; axis equal; axis off;

% --- กำหนดพิกัดจุดหมุน (mm) เพื่อให้ภาพสมดุล อ่านง่าย ---
C  = [0,   32];
O6 = [-18, 14];
B  = [-4,  10];
A  = [22,  16];
D  = [-37, 69];
E  = [-55, 51];
P  = [-90, 48]; % ปลายปากคีบ

% --- 1. วาดเส้นแกนสกรูและรางเลื่อน (Ground Link 1) ---
plot([-28, 52], [0, 0], 'k-.', 'LineWidth', 1.0); % แนวแกนสกรู
plot([4, 42], [-7, -7], 'k-', 'LineWidth', 1.5);  % รางเลื่อนด้านล่าง
draw_hatch(4, 42, -7, -1, 14);                    % เส้นแรเงาฐาน (Ground)

% --- 2. วาดตัวเลื่อน Slider (Link 2) ---
rectangle('Position', [14, -7, 16, 26], 'FaceColor', [0.95 0.95 0.95], ...
          'EdgeColor', 'k', 'LineWidth', 1.8);
% ลูกศรแสดงระยะ Input s
quiver(33, 4, 14, 0, 0, 'Color', [0 0.2 0.7], 'LineWidth', 1.8, 'MaxHeadSize', 0.6);
text(40, 8, '$s$', 'Interpreter', 'latex', 'FontSize', 16, 'Color', [0 0.2 0.7]);

% --- 3. วาดสัญลักษณ์ฐานยึด (Fixed Pivots C และ O6) ---
draw_ground_pivot(C(1),  C(2),  'right');
draw_ground_pivot(O6(1), O6(2), 'down');

% --- 4. วาดข้อต่อต่างๆ (Links 3, 4, 5, 6) ---
% Link 6: ปากคีบ (Jaw) แรเงาสีอ่อน
jaw_x = [D(1), E(1), P(1), -74];
jaw_y = [D(2), E(2), P(2),  64];
fill(jaw_x, jaw_y, [0.92 0.95 0.98], 'EdgeColor', 'k', 'LineWidth', 1.8);

% Link 4: แขนรูปตัว L (Bell-crank B-C-D) แรเงาเทาอ่อนให้เห็นว่าเป็นชิ้นเดียวกัน
fill([B(1) C(1) D(1)], [B(2) C(2) D(2)], [0.88 0.88 0.88], ...
     'EdgeColor', 'none', 'FaceAlpha', 0.6);
plot([B(1) C(1) D(1)], [B(2) C(2) D(2)], 'k-', 'LineWidth', 2.4);

% Link 3 (A-B) และ Link 5 (O6-E)
plot([A(1) B(1)], [A(2) B(2)], 'k-', 'LineWidth', 2.2);
plot([O6(1) E(1)], [O6(2) E(2)], 'k-', 'LineWidth', 2.2);

% --- 5. วาดวงกลมจุดหมุน (Revolute Joints) ---
joints = [A; B; C; D; O6; E];
plot(joints(:,1), joints(:,2), 'ko', 'MarkerFaceColor', 'w', ...
     'MarkerSize', 8, 'LineWidth', 1.6);
plot(P(1), P(2), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 5); % จุดปลาย P

% --- 6. ใส่ตัวอักษรกำกับจุดต่อ (Joints) ---
text(22, -14, '$S$ (Prismatic)', 'Interpreter', 'latex', 'FontSize', 12, 'HorizontalAlignment', 'center');
text(A(1)+3,  A(2)+3,  '$A$',   'Interpreter', 'latex', 'FontSize', 14);
text(B(1)-6,  B(2)-4,  '$B$',   'Interpreter', 'latex', 'FontSize', 14);
text(C(1)+4,  C(2)+5,  '$C$',   'Interpreter', 'latex', 'FontSize', 14);
text(D(1)+2,  D(2)+5,  '$D$',   'Interpreter', 'latex', 'FontSize', 14);
text(O6(1)-10, O6(2)-2,'$O_6$', 'Interpreter', 'latex', 'FontSize', 14);
text(E(1)-7,  E(2)-4,  '$E$',   'Interpreter', 'latex', 'FontSize', 14);
text(P(1)-7,  P(2),    '$P$',   'Interpreter', 'latex', 'FontSize', 14);

% --- 7. ใส่ตัวเลขกำกับข้อต่อ (Links 1 - 6) ---
text(38, -12, '1', 'FontSize', 13, 'FontWeight', 'bold');
text(22,   5, '2', 'FontSize', 13, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
text(10,  16, '3', 'FontSize', 13, 'FontWeight', 'bold');
text(-14, 53, '4', 'FontSize', 13, 'FontWeight', 'bold');
text(-41, 30, '5', 'FontSize', 13, 'FontWeight', 'bold');
text(-66, 56, '6', 'FontSize', 13, 'FontWeight', 'bold');

% ใส่กรอบและชื่อรูปด้านบน
xlim([-105, 60]); ylim([-22, 82]);
title('Kinematic Diagram: 6-Bar Parallel Gripper (Upper Half, 1 DOF)', ...
      'FontSize', 13, 'FontWeight', 'bold');

% บันทึกไฟล์ภาพความละเอียดสูง
exportgraphics(fig, 'Kinematic_Diagram.png', 'Resolution', 300);
fprintf('Saved Kinematic_Diagram.png successfully!\n');

%% --- Helper Functions สำหรับวาดสัญลักษณ์ฐานยึดและเส้นแรเงา ---
function draw_ground_pivot(x, y, dir)
    s = 4.5;
    if strcmp(dir, 'down')
        plot([x, x-s, x+s, x], [y, y-s*1.6, y-s*1.6, y], 'k-', 'LineWidth', 1.3);
        plot([x-s*1.5, x+s*1.5], [y-s*1.6, y-s*1.6], 'k-', 'LineWidth', 1.4);
        draw_hatch(x-s*1.4, x+s*1.4, y-s*1.6, -1, 6);
    elseif strcmp(dir, 'right')
        plot([x, x+s*1.6, x+s*1.6, x], [y, y-s, y+s, y], 'k-', 'LineWidth', 1.3);
        plot([x+s*1.6, x+s*1.6], [y-s*1.5, y+s*1.5], 'k-', 'LineWidth', 1.4);
        for yy = linspace(y-s*1.3, y+s*1.3, 6)
            plot([x+s*1.6, x+s*1.6+3], [yy, yy-2.5], 'k-', 'LineWidth', 0.9);
        end
    end
end

function draw_hatch(x1, x2, y, side, n)
    xs = linspace(x1, x2, n);
    for k = 1:length(xs)
        plot([xs(k), xs(k)-2.5], [y, y + side*3.2], 'k-', 'LineWidth', 0.9);
    end
end