function [suffix] = generateContactDynamics(q, v, T, V, B, phi, suffix)
% GENERATECONTACTDYNAMICS Generates .m files for contact dynamics
%   GENERATECONTACTDYNAMICS(T, V, B, phi, psi, q, v, suffix) takes the
%   following inputs:
%   q: Symbolic vector of configuration states.
%   v: Symboliv vector of configuration velocity states.
%   T: Total system kinetic energy as a symbolic expression of q and v.
%   V: Total system potential energy as a symbolic expression of q.
%   B: Matrix mapping system inputs into generalized forces as a symbolic
%   expression of q.
%   phi: Vector of contactdistance functions as a function of q.
%   suffix: suffix to be added to the output function names.
t_gen = int64(posixtime(datetime));
suffix = ['_',suffix, '_generated_',sprintf('%d', t_gen)];

x = [q; v];

% Compute manipulator equation matrices
jac = jacobian(T,v);

% Generate generalized mass-inertia matrix
H = jacobian(jac,v);

% generate generiled gravity force vector
G = jacobian(V,q).';

% generate coriolis matrix
% there is not necessarily a unique formula for C, but this form is valid;
% see appendix here:
% https://drive.google.com/file/d/0B-JofOC0V2viOFZ3S2hzdjJxWnc/view?usp=sharing
C = jacobian(jacobian(T,v).',q) - (1/2) * jacobian(H*v,q).';

% generate normal contact jacobian
J = jacobian(phi, q);

% generate inertial dynamics gradients
dCv_dq = jacobian(C*v,q);
dCv_dv = jacobian(C*v,v);
dCv  = [dCv_dq dCv_dv];

dG = jacobian(G,q);

% write .m files
writeMatrixFunction(H, q, 'q', ['H', suffix], 0);
writeMatrixFunction(G, q, 'q', ['G', suffix], 0);
writeMatrixFunction(C, x, 'x', ['C', suffix], 0);
writeMatrixFunction(B, [], '', ['B', suffix], 0);
writeMatrixFunction(J, q, 'q', ['J', suffix], 0);
writeMatrixFunction(phi, q, 'q' , ['phi', suffix], 0);

writeMatrixFunction(dG, q, 'q', ['dG', suffix], 0);
writeMatrixFunction(dCv, x, 'x', ['dCv', suffix], 0);


end