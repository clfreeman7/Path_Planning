% 
% Script: utest_MSoRo_IO_01.m
% 
% Dependencies: 
%   +msoro.api.MSoRo_IO
%
% Description: 
%   Unit test of MSoRo Matlab-Arduino API:
%     msoro.api.MSoRo_IO 
% 
% Prerequisites:
%   Connect USB-to-serial adapter (dummy device)
% 
% 


% [0] == Script parameters
%  Serial port config.
port = '/dev/ttyUSB0';
baud = 9600;
timeout = 30;

% Gait command
gait_trans_seq = {[2, 3, 5, 9]};
gait_names = {'A'};
num_gait_cycles = 7;


% [1] == Script setup
% Add dependencies to classpath
addpath('../');

% Configure figure tex interpreters
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');


% [2] == Instantiate and exercise API methods
msoro_robo = msoro.api.MSoRo_IO();

% Open serial port
msoro_robo.connect(port, baud, timeout);
fprintf('[Unit Test] Serial port opened!\n');
pause(1);

% Exercise 'define' operation (send over dead-end serial port)
msoro_robo.define_gait(gait_trans_seq, gait_names);
fprintf('[Unit Test] MSoRo DEFINE operation sent over serial.\n');
pause(1);

% Exercise 'start' operation (send over dead-end serial port)
msoro_robo.start_gait(gait_names{1}, num_gait_cycles);
fprintf('[Unit Test] MSoRo START operation sent over serial.\n');
pause(1);


% [3] == Test disconnect->reconnect runs smoothly
msoro_robo.disconnect();
fprintf('[Unit Test] Serial port disconnected.\n');
pause(1);

msoro_robo.connect(port, baud, timeout);
fprintf('[Unit Test] Serial port re-connected.\n');
pause(1);

msoro_robo.disconnect();
fprintf('[Unit Test] Serial port disconnected.\n');
