% MPHY0054 Tutorial 9 - Dynamics Solution
% This file contains the FULL SOLUTION for the student task.
clc; clear; close all;

%% 1. Define Robot Parameters
params.m1 = 1.0; params.l1 = 1.0; params.r1 = 0.5; params.I1 = 0.1;
params.m2 = 1.0; params.l2 = 1.0; params.r2 = 0.5; params.I2 = 0.1;
params.g  = 9.81;

%% 2. Define the State (Test Case)
% Let's test at a specific configuration
q = [pi/6; pi/3];      % Joint positions (rad)
dq = [0.5; -0.2];      % Joint velocities (rad/s)
ddq = [0.1; 0.1];      % Joint accelerations (rad/s^2)

%% 3. Student Implementation (Solved)
try
    tau_student = student_inverse_dynamics(q, dq, ddq, params);
    fprintf('Your Calculated Torque: [%f, %f] Nm\n', tau_student(1), tau_student(2));
catch ME
    fprintf('Error in your function: %s\n', ME.message);
    tau_student = [NaN; NaN];
end

%% 4. Verification (Using the Black-Box Checker)
% Ensure 'dynamics_verifier.p' is in your folder
try
    [is_correct, tau_ref] = dynamics_verifier(q, dq, ddq, params, tau_student);
    
    fprintf('\n--- Verification Result ---\n');
    fprintf('Reference Torque:     [%f, %f] Nm\n', tau_ref(1), tau_ref(2));
    if is_correct
        fprintf('SUCCESS: Your result matches the reference!\n');
    else
        fprintf('FAILURE: Your result deviates from the reference.\n');
        fprintf('Check your B, C, or g matrices again.\n');
    end
catch
    warning('Verification tool not found or failed. Please ensure dynamics_verifier.p is present.');
end

%% ============================================================
%% SOLVED FUNCTION BELOW
function tau = student_inverse_dynamics(q, dq, ddq, p)
    % Unpack parameters for easier reading
    m1 = p.m1; l1 = p.l1; r1 = p.r1; I1 = p.I1;
    m2 = p.m2; l2 = p.l2; r2 = p.r2; I2 = p.I2;
    g = p.g;
    
    q1 = q(1); q2 = q(2);
    dq1 = dq(1); dq2 = dq(2);
    
    % ---------------------------------------------------------
    % STEP 1: Calculate the Inertia Matrix B(q) [2x2]
    % Hint: Use the formula from Slide 106 / Standard 2-Link Dynamics
    
    % Helper term: interaction between link 1 and 2 depends on cos(q2)
    interaction_term = 2 * m2 * l1 * r2 * cos(q2);
    
    % B11: Inertia of Link 1 + Reflected Inertia of Link 2
    b11 = I1 + I2 + m1 * r1^2 + m2 * (l1^2 + r2^2) + interaction_term;
    
    % B12 & B21: Coupling Inertia
    b12 = I2 + m2 * (r2^2 + l1 * r2 * cos(q2));
    b21 = b12; % The matrix is symmetric
    
    % B22: Inertia of Link 2 only
    b22 = I2 + m2 * r2^2;
    
    B = [b11, b12; 
         b21, b22];
    
    % ---------------------------------------------------------
    % STEP 2: Calculate Coriolis/Centrifugal Matrix C(q, dq) [2x2] or Vector
    % Hint: Calculate Christoffel symbols or use the simplified form
    
    % 'h' is a common grouping term for the Coriolis effect magnitude
    % derived from the derivative of the Inertia matrix (specifically the sine component)
    h = -m2 * l1 * r2 * sin(q2);
    
    % Calculate the vector C(q,dq)*dq directly:
    % Term 1 includes Centrifugal force from joint 2 (dq2^2) and Coriolis (dq1*dq2)
    c1 = h * dq2^2 + 2 * h * dq1 * dq2;
    
    % Term 2 includes Centrifugal force from joint 1 (dq1^2)
    c2 = -h * dq1^2;
    
    C_term = [c1; c2]; 
    
    % ---------------------------------------------------------
    % STEP 3: Calculate Gravity Vector g(q) [2x1]
    % Hint: Partial derivative of Potential Energy
    
    % P_energy = m1*g*y1 + m2*g*y2
    % Taking partial derivatives dP/dq1 and dP/dq2:
    
    g1 = (m1 * r1 + m2 * l1) * g * cos(q1) + m2 * r2 * g * cos(q1 + q2);
    g2 = m2 * r2 * g * cos(q1 + q2);
    
    G = [g1; g2];
    
    % ---------------------------------------------------------
    % Final Equation: tau = B*ddq + C*dq + G
    tau = B*ddq + C_term + G; 
end