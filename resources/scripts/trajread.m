function [ S, C, cost ] = trajread( file_path, parameter )
%TRAJREAD parses the json results file of SBMPO into matlab matricies and
%   values.
%
%   [S C cost] = TRAJREAD('file') parses the trajectory from a file
%   and provides the results in matrix format, where:
%       S is a matrix with the trajectory node states as columns
%       C is a matrix with the trajectory node controls as columns
%       cost is a double representing the total trajectory cost
%
%   S = TRAJREAD('file', 'states') outputs just the states matrix
%   C = TRAJREAD('file', 'controls') outputs just the controls matrix
%   cost = TRAJREAD('file', 'cost') outputs the total trajectory cost
%
%   For example, if we have the following file trajectory.json:
%
%   {
%       "cost": 10.0,
%       "trajectory": [
%           {"control":[0.00],"state":[0.0000, 0.0000, 0.00]},
%           {"control":[0.75],"state":[0.0075, 0.0750, 0.75]},
%           {"control":[0.75],"state":[0.0225, 0.1500, 0.75]},
%           {"control":[0.75],"state":[0.0450, 0.2250, 0.75]},
%           {"control":[0.75],"state":[0.0750, 0.3000, 0.75]},
%           {"control":[0.75],"state":[0.1125, 0.375,  0.75]},
%           {"control":[0.75],"state":[0.1574, 0.4499, 0.75]}
%       ]
%   }
%
%   S = TRAJREAD('trajectory.json','states') will produce
%
%   S =
%
%       0    0.0075    0.0225    0.0450    0.0750    0.1125    0.1574
%       0    0.0750    0.1500    0.2250    0.3000    0.3750    0.4499
%       0    0.7500    0.7500    0.7500    0.7500    0.7500    0.7500
%
%   [S sz] = TRAJREAD('file', 'states') outputs the states matrix and the
%   size of the states matrix; sz equivalent to size(S)
%   [C sz] = TRAJREAD('file', 'controls') outputs the controls matrix and
%   the size of the states matrix; sz is equivalent to size(C)
%   results = TRAJREAD('file', 'struct') returns the trajectory struct which
%   follows structure of the json file.
%   raw = TRAJREAD('file', 'raw') outputs the raw character vector
%   with the contents of the file.
%
%   See also FILEREAD, JSONDECODE, SIZE

% Load json file to get sbmpo's trajectory
results=jsondecode(fileread(file_path));

if nargin == 2
    % If only one parameter was requested, provide just that parameter
    if strcmp(parameter,'states'), S = st(results); C = size(S); end
    if strcmp(parameter,'controls'), S = cn(results); C = size(S); end
    if strcmp(parameter,'cost'), S = results.cost; end
    if strcmp(parameter,'struct'), S = results; end
    if strcmp(parameter,'raw'), S = fileread(file_path); end
else
    % Extract relevant data from trajectory
    C = cn(results);
    S = st(results);
    cost = results.cost;
end

end


function C = cn( results )
% get controls from parsed results

C = arrayfun(@(node) node.control, results.trajectory, 'UniformOutput', false);
C = reshape( ...
    cell2mat(C), ...
    [size(results.trajectory(1).control,1) numel(results.trajectory)] ...
);

end


function S = st( results )
% get states from parsed results

S = arrayfun(@(node) node.state, results.trajectory, 'UniformOutput',false);
S = reshape( ...
    cell2mat(S), ...
    [size(results.trajectory(1).state,1) numel(results.trajectory)] ...
);

end
