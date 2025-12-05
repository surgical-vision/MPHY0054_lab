% MPHY0054 Tutorial 9 - Dynamics Task
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

%% 3. Student Implementation
% TASK: Fill in the 'student_inverse_dynamics' function below
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
%% STUDENT TO EDIT THIS FUNCTION
function tau = student_inverse_dynamics(q, dq, ddq, p)
    % Unpack parameters for easier reading
    m1 = p.m1; l1 = p.l1; r1 = p.r1; I1 = p.I1;
    m2 = p.m2; l2 = p.l2; r2 = p.r2; I2 = p.I2;
    g = p.g;
    
    q1 = q(1); q2 = q(2);
    dq1 = dq(1); dq2 = dq(2);
    
    % ---------------------------------------------------------
    % STEP 1: Calculate the Inertia Matrix B(q) [2x2]
    % Hint: Use the formula from Slide 106
    % B = ...
    B = zeros(2,2); % REPLACE THIS WITH YOUR CODE
    
    % ---------------------------------------------------------
    % STEP 2: Calculate Coriolis/Centrifugal Matrix C(q, dq) [2x2] or Vector
    % Hint: Calculate Christoffel symbols or use the simplified form
    % C_matrix = ...
    C_term = zeros(2,1); % This represents C(q,dq)*dq. REPLACE THIS.
    
    % ---------------------------------------------------------
    % STEP 3: Calculate Gravity Vector g(q) [2x1]
    % Hint: Partial derivative of Potential Energy
    % G = ...
    G = zeros(2,1); % REPLACE THIS WITH YOUR CODE
    
    % ---------------------------------------------------------
    % Final Equation: tau = B*ddq + C*dq + G
    tau = B*ddq + C_term + G; 
end