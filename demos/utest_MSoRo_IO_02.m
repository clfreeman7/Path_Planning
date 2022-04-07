% 
% Script: utest_MSoRo_IO_02.m
% 
% Dependencies: 
%   +msoro.api.MSoRo_IO
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


% [2] == Reproduce gait execution sequence from Arun's API.m
msoro_robo = msoro.api.MSoRo_IO();

% Open serial port
msoro_robo.connect(port, baud, timeout);
fprintf('[Unit Test] Serial port opened!\n\n');
pause(1);

% Send 'define' command for gait A
msoro_robo.define_gait(gait_trans_seq, gait_names);
fprintf('[Unit Test] MSoRo DEFINE operation sent over serial.\n\n');
pause(1);

% Send 1st 'start' command: A7
msoro_robo.start_gait(gait_names{1}, num_gait_cycles);
fprintf('[Unit Test] 1st MSoRo START operation sent over serial.\n\n');
pause(1);

% Wait for MSoRo/Arduino completion message
rcv_timeout = 10;
start_time = datevec(now);
start_time_sec = start_time(4)*3600 + start_time(5)*60 + start_time(6);
fprintf('[Unit Test] Waiting for receipt of MSoRo message...\n');
while ( ~msoro_robo.serial_data_rcvd ) 
  pause(1);

  cur_time = datevec(now);
  cur_time_sec = cur_time(4)*3600 + cur_time(5)*60 + cur_time(6);

  if ( cur_time_sec - start_time_sec > rcv_timeout )
    fprintf('[Unit Test] No MSoRo message received within %d sec. Timing out.\n\n', rcv_timeout);
    break;
  end
end
msoro_robo.serial_data_rcvd = false;

% Send 2nd 'start' command: A7 
msoro_robo.start_gait(gait_names{1}, num_gait_cycles);
fprintf('[Unit Test] 2nd MSoRo START operation sent over serial.\n\n');
pause(1);

% Close MSoRo connection
msoro_robo.disconnect();
fprintf('[Unit Test] Serial port disconnected.\n\n');
pause(1);

