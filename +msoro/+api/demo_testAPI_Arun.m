% 
% Script: demo_MSoRo_IO.m
%  
% Dependencies:
% Dependencies:
%   +demos/data/gait_library_2.mat
%
%   +msoro/+api.MSoRo_IO.m
%
% Description: 
%   Demonstrate how to create use experimental data for open-loop paths. 

% [0] == Script setup
clear; clc
% Add dependencies to classpath
addpath('../');

gait_sequences = {[3,7,2]; 
         [16,7,5,11,14];
         [8,9,16]; 
         [4,14];
         [9,16,1]};

n_gaits = length(gait_sequences);


% [1] == Extract and define parameters for GaitTest() objects.

% Define experimental parameters after investigating video. 
frame_start_list = [607, 103, 176, 226, 368];      % for orange robot

% Extract data.
for i = 1:n_gaits
    % Define robot.
    gait_exp_2(i).params.robot_name = 'orange';
    gait_exp_2(i).params.substrate = 'black mat';
    
    % Extract and store first frame information.
    gait_exp_2(i).params.frame_1 = frame_start_list(i);
    
    % Extract and store raw data from each trial.
    filename = ['data/visualtracking/Gait', ' ', num2str(char('A' + i -1)), '.mat'];
    gait_exp_2(i).raw_data = load(filename).all_pt;  
end


% [2] == Instantiate GaitTest() objects for each experimental trial.
% This analyzes the data from each trial to find motion primitive twist
% information.

% Instantiate objects for each gait tested. 
for i = 1:n_gaits
    all_gaits(i) = offlineanalysis.GaitTest(gait_exp_2(i).raw_data, ...
                                                 gait_sequences{i}(1,:), ...
                                                 gait_exp_2(i).params); 
end


% [3] == Instantiate gait objects for each gait tested to build a gait
% library. 
for i = 1:n_gaits
    gait_exp_2(i).params.gait_name = num2str(char('A' + i -1));
    gait_library_2(i) = gaitdef.Gait(all_gaits(i), gait_exp_2(i).params);

end


% [1] == Extract and define gait data.
n_gaits = length(gait_library_2);
% Extract the motion data and gait names from the gait library
gait_trans_seq = cell(1, n_gaits);
gait_names = cell(1, n_gaits);

for i = 1:n_gaits
    gait_trans_seq{i} = gait_library_2(i).robo_states;
    gait_names{i} = gait_library_2(i).gait_name;
end


% [2] == Define the open-loop canned sequence to be run. 
% canned_sequence = 'A7,B3,D2,C4,E5';
% 
% % Extract relevant values.
% canned_sequence = char(split(canned_sequence,','));
% gait_order = canned_sequence(:,1)';
% num_cycles = str2num(canned_sequence(:,2))';


%%
% Script using the MSoRo_IO class
port = 'COM4';
baud = 9600;
timeout = 30;

% Gait command
% gait_trans_seq = {[2, 3, 15, 9, 8, 7, 2],[2, 3, 5, 9, 8, 7]};
% gait_names = {'A','B'};
% num_gait_cycles = [3,5,4,5,6];


% [1] == Script setup
% Add dependencies to classpath
addpath('../');

% [2] == Reproduce gait execution sequence from Arun's API.m
msoro_robo = MSoRo_IO();
% Connect to Serial port
msoro_robo.connect(port, baud, timeout);
fprintf('[Unit Test] Serial port opened!\n\n');
pause(5);

% Defining gaits
msoro_robo.define_gait(gait_trans_seq, gait_names)

% Canned sequence
canned_sequence = 'B 10';
% Extract relevant values.
canned_sequence = char(split(canned_sequence,',')); 
for i = 1:size(canned_sequence,1)
canned_sequence_split = strsplit(canned_sequence(i,:),' ');
gait_perform_seq(i,1) = canned_sequence_split(1,1);
num_gait_cycles(i,1) = str2num(cell2mat(canned_sequence_split(1,2)));
end
gait_perform_seq = gait_perform_seq';
num_gait_cycles = num_gait_cycles';

for jj = 1:size(gait_perform_seq,2)
    msoro_robo.start_gait(gait_perform_seq{jj}, num_gait_cycles(jj));
    pause(1);
end
