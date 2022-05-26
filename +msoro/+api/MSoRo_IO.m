%============================= MSoRo_IO =================
%
%  Serial interface to communicate and command MSoRo soft mobile robot.
%
%  MSoRo_IO()
%
%  INPUTS:
%    TBD
%
%
%  ============================= MSoRo_IO =================
classdef MSoRo_IO < handle
  properties  (Access = public)
    serial_props;

    serial_data_rcvd;     % boolean flag indicating receipt of data (set by default serial callback handler)
    serial_data;          % cell array of received serial data (set by default serial callback handler)
  end

  properties  (Access = private)
    ser_device;         % MSoRo serial comm. handle
    ser_device_open;    % boolean flag

  end

  methods
    % Constructor
    function this = MSoRo_IO( params )
      if ( nargin < 1 )
        params = [];
      end
      
      % == Set input parameters
      % this.set_property(params, 'subclass_specific_param', 0.25);

      % Initialization
      this.ser_device_open = false;

      this.serial_props.port = [];
      this.serial_props.baud = [];
      this.serial_props.timeout = [];

      this.serial_data_rcvd = false;
      this.serial_data = {};
    end

    % Open serial connection (to MSoRo)
    %
    % Input(s):
    %   a_port:         port ID (e.g. "COM6")
    %   a_baud:         baud rate (e.g. 9600)
    %   a_timeout:      [optional] timeout duration in sec. (e.g. 30)
    %   a_cb_hdl:       [optional] user-specified callback handler for serial receipt events
    % 
    function connect( this, a_port, a_baud, a_timeout, a_cb_hdl )
      assert( ~this.ser_device_open, ...
              '[MSoRo_IO::connect()] A port is already open. Disconnect before establishing a new connection.');       % check serial port opened

      if ( nargin < 5 )
        reg_custom_cb = false;
      else
        reg_custom_cb = true;        
      end
      
      if ( nargin < 4 )
        a_timeout = 30;   % sec.
      end

      this.serial_props.port = a_port;
      this.serial_props.baud = a_baud;
      this.serial_props.timeout = a_timeout;

      this.ser_device = serialport(a_port, a_baud, "Timeout", a_timeout);

      if ( ~isempty(this.ser_device) )
        flush(this.ser_device);
        this.ser_device_open = true;

        if ( reg_custom_cb )
          configureCallback(this.ser_device, "terminator", a_cb_hdl);                 % register user-specified serial comm. callback handler
        else
          configureCallback(this.ser_device, "terminator", @this.msoro_serial_cb);    % register serial comm. callback handler
        end
      else
        warning('[MSoRo_IO::connect()] Failed to open serial port.');
      end
    end

    % Close serial connection (to MSoRo)
    %
    % Input(s):
    %   None
    %
    function disconnect( this )
      assert( this.ser_device_open, ...
              '[MSoRo_IO::disconnect()] No serial port opened.');       % check serial port opened
      
      this.ser_device = [];

      this.ser_device_open = false;
    end

    % Issue MSoRo gait definition(s)
    %
    % Input(s):
    %   a_transition_seq:   (cell array) each cell contains vector of
    %                         MSoRo transition IDs
    %   a_gait_names:       (cell array) each cell contains a (string) gait 
    %                         name, for the corresponding transition 
    %                         sequence in a_transition_seq
    %
    function define_gait( this, a_transition_seq, a_gait_names )
      assert( iscell(a_transition_seq), ...
        '[MSoRo_IO::define_gait()] Input must be cell array of vectors.');  % check input data format
      assert( this.ser_device_open, ...
        '[MSoRo_IO::define_gait()] No serial port opened.');                % check serial port opened

      for ii = 1:length(a_transition_seq)
        %         gait_def = sprintf('%s%s%s',sprintf('define %s',a_gait_names{ii}),...
        %             sprintf('%3d',a_transition_seq{ii}),sprintf(' end '))
        gait_def = sprintf('%s',sprintf('define %s',a_gait_names{ii}));
        for i = 1:length(a_transition_seq{ii})
          gait_def = sprintf('%s%s',gait_def, sprintf(' %d',a_transition_seq{ii}(i)));
        end
        gait_def = sprintf('%s%s', gait_def, ' end ');
        write(this.ser_device,gait_def,"string");
        fprintf('[MSoRo_IO::define_gait()] %s\n', gait_def);
        pause(1);
      end
    end

    % Start MSoRo gait
    %
    % Input(s):
    %   a_gait_name:        (string) name of gait
    %   a_num_cycles:       number of gait cycles to run
    %
    function start_gait( this, a_gait_name, a_num_cycles )
      assert( this.ser_device_open, ...
              '[MSoRo_IO::start_gait()] No serial port opened.');    % check serial port opened
      
      if ( nargin < 3 )
        a_num_cycles = 1;     % if not specified, run for 1 gait cycle
      end

      gait_cmd = sprintf('start %s %d ', a_gait_name, a_num_cycles)

%       write(this.ser_device,'start ',"string");
%       writeline(this.ser_device, gait_cmd);
      write(this.ser_device, gait_cmd, "string");
      fprintf('[MSoRo_IO::start_gait()] %s\n', gait_cmd);
      fprintf('[MSoRo_IO::start_gait()] Gait sequence %s running for %d cycles.\n', a_gait_name, a_num_cycles);
%       check_complete = "#Completed";
%       ii = 1;
%       while(ii)
%         data_recieve = strtrim(readline(this.ser_device))
%         if(contains(data_recieve,check_complete) == 1)
%           ii = 0;
%         end
%       end
%       fprintf('Gait sequence %s completed\n',a_gait_name);
% while ~contains(check,'Completed')
%     check = readline(this.ser_device);
% end
    end

    % (Default) MSoRo serial callback handler (Arduino -> Matlab communication)
    %
    % Input(s):
    %   src:        serial device handle
    %   evnt:       (~, not used) triggering event data
    % 
    function msoro_serial_cb( this, src, ~ )
      data = readline(src);

      this.serial_data_rcvd = true;
      this.serial_data{end+1} = data;
      fprintf('[MSoRo_IO::msoro_serial_cb()] Data recieved: %s\n', this.serial_data{end}); % (VV) added the cell number {end}

      % TODO: how process readline data from Arduino?
    end

  end     % methods 
end     % class








