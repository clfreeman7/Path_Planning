% 
% Script: utest_MSoRo_IO_03.m
% 
% Dependencies: 
%   +msoro.api.MSoRo_IO
%   +gait_def.Gait
%
% Description: 
%   Unit test of MSoRo Matlab-Arduino API: msoro.api.MSoRo_IO 
% 
%   Connect MSoRo robot to PC. Set serial port connection information
%   (Section [0]). Verify same sequence of gait commands Arun accomplished
%   with API.m (in https://github.com/arunniddish/API).
% 
% Prerequisites:
%   Connect MSoRo soft robot to PC via serial port.
% 
% 


% [0] == Script parameters
%  Serial port config.
port = '/dev/ttyUSB0';
baud = 9600;
timeout = 30;

% Gait selection (from library)
TRANS_GAIT_IND = 5;
ROT_GAIT_IND = 2;


% [1] == Script setup
% Add dependencies to classpath
addpath('../');

% Configure figure tex interpreters
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');

gait_library = load('data/gait_library_2.mat').gait_library_2;
gait_library(1).gait_name = 'A';    % temporarily overwritten (until Gait() demo script assigns names to gaits in library)
gait_library(2).gait_name = 'B';
gait_library(3).gait_name = 'C';
gait_library(4).gait_name = 'D';
gait_library(5).gait_name = 'E';


% [2] == Reproduce gait execution sequence from Arun's API.m
msoro_robo = msoro.api.MSoRo_IO();

% Open serial port
msoro_robo.connect(port, baud, timeout);
fprintf('[Unit Test] Serial port opened!\n\n');
pause(1);

% Send 'define' command for translation & rotation gaits
gait_trans_seq = {gait_library(TRANS_GAIT_IND).robo_states, gait_library(ROT_GAIT_IND).robo_states};
gait_names = {gait_library(TRANS_GAIT_IND).gait_name, gait_library(ROT_GAIT_IND).gait_name};
gait_dur = [ gait_library(TRANS_GAIT_IND).transition_time*(gait_library(TRANS_GAIT_IND).len_gait), ...
              gait_library(ROT_GAIT_IND).transition_time*(gait_library(ROT_GAIT_IND).len_gait) ];


msoro_robo.define_gait(gait_trans_seq, gait_names);
fprintf('[Unit Test] MSoRo DEFINE operation sent over serial.\n\n');
pause(1);

% 'start' gait B, 3 cycles
gait_ind = 2;
num_cycles = 3;
msoro_robo.start_gait(gait_names{gait_ind}, num_cycles);
fprintf('[Unit Test] Gait B MSoRo START command sent over serial.\n\n');
pause(gait_dur(gait_ind)*(num_cycles-1));

% 'start' gait E, 5 cycles
gait_ind = 1;
num_cycles = 5;
msoro_robo.start_gait(gait_names{gait_ind}, num_cycles);
fprintf('[Unit Test] Gait E MSoRo START command sent over serial.\n\n');
pause(gait_dur(gait_ind)*(num_cycles-1));

% 'start' gait B, 4 cycles
gait_ind = 2;
num_cycles = 4;
msoro_robo.start_gait(gait_names{gait_ind}, num_cycles);
fprintf('[Unit Test] Gait B MSoRo START command sent over serial.\n\n');
pause(gait_dur(gait_ind)*(num_cycles-1));

% 'start' gait E, 7 cycles
gait_ind = 1;
num_cycles = 7;
msoro_robo.start_gait(gait_names{gait_ind}, num_cycles);
fprintf('[Unit Test] Gait E MSoRo START command sent over serial.\n\n');
pause(gait_dur(gait_ind)*(num_cycles-1));

% Wait for and straggling serial message from MSoRo
pause(3);

% Close MSoRo connection
msoro_robo.disconnect();
fprintf('[Unit Test] Serial port disconnected.\n\n');
pause(1);

