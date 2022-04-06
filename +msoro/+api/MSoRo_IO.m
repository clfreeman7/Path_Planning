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
%  OUTPUTS:
%
%
%  ============================= MSoRo_IO =================
classdef MSoRo_IO < handle
  properties  (Access = public)
    serial_props;

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
    end

    % Open serial connection (to MSoRo)
    %
    % Input(s):
    %   a_port:         port ID (e.g. "COM6")
    %   a_baud:         baud rate (e.g. 9600)
    %   a_timeout:      [optional] timeout duration in sec. (e.g. 30)
    % 
    function connect( this, a_port, a_baud, a_timeout )
      assert( this.ser_device_open, ...
              '[MSoRo_IO::connect()] A port is already open. Disconnect before establishing a new connection.');       % check serial port opened

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

        configureCallback(this.ser_device, "terminator", @this.msoro_serial_cb);    % register serial comm. callback handler
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
      assert( strcmp(a_transition_seq, 'cell'), ...
              '[MSoRo_IO::define_gait()] Input must be cell array of vectors.');  % check input data format
      assert( this.ser_device_open, ...
              '[MSoRo_IO::define_gait()] No serial port opened.');                % check serial port opened

      for ii = 1:length(a_transition_seq)
        gait = a_transition_seq{ii};
        seq_len_str = int2str( length(gait) );

        write(this.ser_device, 'define', "string");      % MSoRo operation
        write(this.ser_device, seq_len_str, "string");   % length of transition sequence
        write(this.ser_device, gait, "int8");            % transition sequence
      end

      % TODO: store gait internally?
      %   requires a little doing: map from assigned name to internal name,
      %   trans sequence, track last alphabetical name used
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

      % TODO: map internal gait name from a_gait_name?
      gait_cmd = sprintf('%s%d', a_gait_name, a_num_cycles);

      write(this.ser_device,'start',"string");
      writeline(this.ser_device, gait_cmd);
    end

    % MSoRo serial callback (Arduino -> Matlab communication)
    %
    % Input(s):
    %   src:        serial device handle
    %   evnt:       (~, not used) triggering event data
    % 
    function msoro_serial_cb( this, src, ~ )
%      data = readline(src);
      % TODO: how process readline data from Arduino?
    end

  end     % methods 
end     % class








