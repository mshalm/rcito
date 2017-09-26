function writeMatrixFunction(content, variables, varname, fcn_name, ...
    sparse_flg)
% WRITEMATRIXFUNCTION Converts a symbolic expression to a .m function file.
%   WRITEMATRIXFUNCTION(content, variables, varname, fcn_name) takes the
%   following inputs:
%   content: matrix (or vector/scalar) containing the symbolic expression.
%   variables: symbolic variables from which "content" was constructed.
%   varname: name of input of generated function.
%   fcn_name: name of generated function.
%   sparce_flag: if true, output of generated function will be of the
%   sparse matrix type.



[m, n] = size(content);

num_vars = numel(variables);

% Open m.file
fid = fopen(['./', fcn_name, '.m'], 'w+');

% Write header.
fprintf(fid, 'function [out]= %s(%s)\n', fcn_name, varname);

% Some header comments.
fprintf(fid, '%%%%');
fprintf(fid, '%%%%%%%% %s\n', ['  ', fcn_name, '.m']);
fprintf(fid, '%%%%%%%% %s \n', datestr(now));
fprintf(fid, '%s\n', '%%%%');
fprintf(fid, '%s\n', '%%%% Authors(s): Mathew Halm');
fprintf(fid, '%s\n', '%%%%');
fprintf(fid, '%s\n', '%%%%');
fprintf(fid, '%s\n', '%%%%');
fprintf(fid, '%s\n', '%%%%');


fprintf(fid, '%s\n', '% Inputs');


for i = 1:num_vars
    fprintf(fid, '%s = %s(%d);\n', variables(i), varname, i);
end


fprintf(fid, '\n');


fprintf(fid, '%s\n', ['out = zeros(', num2str(m), ',', num2str(n), ',' ...
    num2str(o), ');']);


fprintf(fid, '%% Populate output. \n');


for i=1:m
    for j=1:n
            fprintf(fid, '%s\n', ['out(', num2str(i), ',', num2str(j) ...
                ') = ', char(content(i,j)), ';']);
    end
end


if (sparse_flg)
    fprintf(fid, '%s\n', 'out = sparse(out)');
end


fprintf(fid, '%s\n', 'end');


% Close m.file
fclose(fid);


end