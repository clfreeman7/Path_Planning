%================================== SE2 ==================================
%
%  function g = SE2(d, rot)
%
%
%  Generates an instance of the class object SE2.  As a class, it acts as
%  though it were a Matlab variable.  Different operations and actions
%  can be applied to it.
%
% 
%  Inputs:
%    d		- the displacement/translation.
%    rot	- either the angle of rotation, or the rotation matrix.
%
%================================== SE2 ==================================
classdef SE2 <handle
    
properties (Constant)
  epsilon = 0.0001;
end

properties (Access = protected)
  M
end

%%
methods 

  function g = SE2(d, rot)

    if (nargin == 0)
      d = [0;0];
      rot = 0;
    elseif ( (size(d,1) == 1) && (size(d,2) == 2) )
      d = transpose(d);
    elseif ( (size(d,1) ~= 2) || (size(d,2) ~= 1) )
      error('The translation vector has incorrect dimensions');
    end

    if (isscalar(rot))
      rot = [cos(rot), -sin(rot); sin(rot), cos(rot)];
    elseif ( (size(rot,1) ~= 2) || (size(rot,2) ~= 2) )
      error('The rotation input has incorrect dimensions');
    end

    g.M = [rot, d; 0 0 1];

  end
  
  %============================== Adjoint ==============================
  %
  %  function g = adjoint(g1, g2)
  %
  %
  %  Computes and returns the adjoint of g.  When applied to a Lie group
  %  elements, the adjoint is defined to operate as:
  %
  %    Ad_g1 (g2) = g1 * g2 * inverse g1
  %
  %  When applied to a Lie algebra elements in homogeneous form, it
  %  operates as:
  %
  %    Ad_g1 xi_hat = g1 * xi_hat * inverse(g1)
  %
  %  and when applied to a Lie algebra element in vector form it is the
  %  linear matrix operation:
  %    
  %                 [ R  | JJ d ]    [ xi1 ]
  %    Ad_g1 xi =   [ --------- ] *  [ xi2 ]
  %                 [ 0  |  1   ]    [ xi3 ]
  %
  %
  %============================== Adjoint ==============================
  function z = adjoint(g, x)

    if (isa(x,'SE2'))
      z = g*x*inv(g);
    elseif ( (size(x,1) == 3) && (size(x,2) == 1) )
      JJ = [0 1;-1 0];
      z = [g.M(1:2,1:2) , JJ*g.M(1:2,3);0 0 1]*x;
    elseif ( (size(x,1) == 3) && (size(x,2) == 3) )
      z = g.M*x*inv(g.M);
    end

  end
  
  %================================ display ================================
%
%  function display(g)
%
%
%  This is the default display function for the SE2 class.  It simply
%  displays the position followed by the rotation.
%
%================================ display ================================
  function display(g)

    if isequal(get(0,'FormatSpacing'),'compact')
      disp([inputname(1) ' =']);
      disp(g.M);
    else
      disp(' ');
      disp([inputname(1) ' =']);
      disp(' ');
      disp(g.M);
    end
  end
  
  %================================ getAngle ===============================
  %
  %  Return the orientation of the Lie group object as an angle.
  %
  %
  function angle = getAngle(g)
  angle = atan2(g.M(2,1), g.M(1,1));
  end

  %============================= getTranslation ============================
  %
  %  Returns the translation or position associated to the group
  %  element.
  %
  function d = getTranslation(g)
  d = g.M(1:2,3);
  end
  
  %============================== getRotation ==============================
  %
  %  Return the orientation of the Lie group object as a rotation matrix.
  %
  function R = getRotation(g)
  R = g.M(1:2,1:2);
  end
  
  %================================ inv ================================
  %
  %  invg = inv(g)
  %
  %  
  %  Computes and returns the inverse to g.
  %
  %================================ inv ================================
  function invg = inv(g)
  tR = transpose(g.M(1:2,1:2));
  invgM = [tR, -tR*g.M(1:2,3); 0, 0, 1];
  invg = SE2(invgM(1:2,end), invgM(1:2,1:2));
  end
  
  %=============================== leftact ==============================
  %
  %  p2 = leftact(g, p)
  %		with p a 2x1 specifying point coordinates.
  %
  %  p2 = leftact(g, v)
  %		with v a 3x1 specifying a velocity.
  %		This applies to pure translational velocities in homogeneous
  %		form, or to SE2 velocities in vector forn.
  %
  %  This function takes a change of coordinates and a point/velocity,
  %  and returns the transformation of that point/velocity under the change
  %  of coordinates.  
  %  
  %  Alternatively, one can think of the change of coordinates as a 
  %  transformation of the point to somewhere else, e.g., a displacement 
  %  of the point.  It all depends on one's perspective of the 
  %  operation/situation.
  %
  %=============================== leftact ===============================
  function x2 = leftact(g, x)
  if ( (size(x,1) == 2) )
    x2 = g.M*[x ; ones([1, size(x,2)])];
    x2 = x2(1:2,:);
  elseif ( (size(x,1) == 3) )
    % BELOW IS NOT CORRECT. IF YOU MODIFIED, TALK TO ME - PAV.
    %L = eye(3);
    %L(1:2,1:2) = g.M(1:2,1:2);
    x2 = g.M*x;
  end
  end
  
  %================================== log ==================================
  %
  %  function xi = log(g, tau)
  %
  %  Take the logarithm of the group element g.  If the time period of
  %  the action is not given, it is assumed to be unity.
  %
  %================================== log ==================================
  function xi = log(g, tau)

  if ( (nargin < 2) || isempty(tau) )
    tau = 1;
  end

  xi = zeros([3 1]);			% Specify size/dimensions of xi.

  %--(1) Obtain the angular velocity.
  R = g.getRotation();
  xi(3)  = atan2(R(2,1), R(1,1))/tau;

  %--(2) Compute the linear velocity.
  if (xi(3) == 0)				% If no rotation, pure translation.
    xi(1:2) = g.getTranslation()/tau;
  else					% else, use logarithm equations.
    JJ = [0 1;-1 0];
    xi(1:2) = xi(3)*JJ*inv(eye(2)-R)*g.getTranslation();
  end
  end
  
  %================================ mtimes ===============================
  %
  %  function g = mtimes(g1, g2)
  %
  %
  %  Computes and returns the product of g1 with g2.
  %
  %  Can also be typed as:  >> g3 = g1*g2
  %
  %================================ mtimes ===============================
  function g = mtimes(g1, g2)
  g = SE2();
  g.M = g1.M * g2.M;
  end
  
  %================================== plot =================================
  %
  %  function plot(g, label, linecolor, sc)
  %
  %  Plots the coordinate frame associated to g.  The figure is cleared, 
  %  so this will clear any existing graphic in the figure.  To plot on
  %  top of an existing figure, set hold to on.  The label is the name
  %  of label given to the frame (if given is it writen out).  The 
  %  linecolor is a valid plot linespec character.  Finally sc is the
  %  specification of the scale for plotting.  It will rescale the
  %  line segments associated with the frame axes and also with the location
  %  of the label, if there is a label.
  %
  %  Inputs:
  %    g		- The SE2 coordinate frame to plot.
  %    label	- The label to assign the frame.
  %    linecolor  - The line color to use for plotting.  (See `help plot`) 
  %    sc		- scale to plot things at.
  %		  a 2x1 vector, first element is length of axes.
  %		    second element is a scalar indicating roughly how far
  %		    from the origin the label should be placed.
  %
  %  Output:
  %    The coordinate frame, and possibly a label,  is plotted.
  %
  %================================== plot =================================
  function plot(g, flabel, lcol, sc)

  if ( (nargin < 2) )
    flabel = '';
  end

  if ( (nargin < 3) || isempty(lcol) )
    lcol = 'b';
  end

  if ( (nargin < 4) || isempty(sc) )
    sc = [1 1];
  elseif (size(sc,2) == 1)
    sc = [sc 2];
  end

  o = g.getTranslation();           % Origin of the frame at g's object.

  ex = g.getRotation()*[sc(1);0];   % unit axes in object's frame.
  ey = g.getRotation()*[0;sc(1)];

  isheld = ishold;

  lspec = [lcol '-.'];              % Line specification for plotting.
  pts = [o-0.5*ex , o+ex];              % Points of x-axis.
  plot(pts(1,:), pts(2,:), lspec);
  hold on;
  pts = [o-0.5*ey , o+ey];              % Points of y-axis.
  plot(pts(1,:), pts(2,:), lspec);
  plot(o(1), o(2), [lcol 'o'],'MarkerSize',7);
                                    % Plot marker at origin.

  if (~isempty(flabel))             % If label desired, then offset and plot.
    pts = o - (sc(2)/sc(1))*(ex+ey)/4;
    text(pts(1), pts(2), flabel);
  end

  if (~isheld)
    hold off;
  end

  axis equal;
  end

  function plotY(g, flabel, lcol, sc)

  if ( (nargin < 2) )
    flabel = '';
  end

  if ( (nargin < 3) || isempty(lcol) )
    lcol = 'b';
  end

  if ( (nargin < 4) || isempty(sc) )
    sc = [1 1];
  elseif (size(sc,2) == 1)
    sc = [sc 2];
  end

  o = g.getTranslation();           % Origin of the frame at g's object.

  ex = g.getRotation()*[sc(1);0];   % unit axes in object's frame.
  ey = g.getRotation()*[0;sc(1)];

  isheld = ishold;

  lspec = [lcol '-.'];              % Line specification for plotting.
  pts = [o-ex , o+ex];              % Points of x-axis.
  plot(pts(1,:), pts(2,:), lspec);
  hold on;
  pts = [o-ey , o+2*ey];            % Points of y-axis.
  plot(pts(1,:), pts(2,:), lspec);
  plot(o(1), o(2), [lcol 'o'],'MarkerSize',7);
                                    % Plot marker at origin.

  if (~isempty(flabel))             % If label desired, then offset and plot.
    pts = o - (sc(2)/sc(1))*(ex+ey)/4;
    text(pts(1), pts(2), flabel);
  end

  if (~isheld)
    hold off;
  end

  axis equal;
  end

  %============================ velocityPlot ===========================
  %
  %  Plots a vector velocity of SE(2) as a vector and a rotation.
  %  Assumes that this is not given in body coordinates, but in the
  %  world frame (so it is in mixed frames for velocities, so to speak).
  %
  function velocityPlot(g, vect, rad)

  if (nargin < 3)
    rad = 0.5;
  end

  basePt = g.getTranslation();      % Get base point of twist.

  rotAngle = g.getAngle();          % Get rotation angle for velocity.
  thetaVals = rotAngle + linspace(0, 3*pi/6, 20);
  if (vect(3) < 0)
    thetaVals = -thetaVals;
  end
  arcPts = rad*[cos(thetaVals) ; sin(thetaVals)] ...
                + repmat(basePt, [1, length(thetaVals)]);
  arcVec = diff(arcPts(:,end-1:end),1,2);

  wasHeld = ishold;

  hold on;
  if ((vect(1) ~= 0) || (vect(2) ~= 0))
    quiver(basePt(1), basePt(2), vect(1), vect(2), ...
                                        'LineWidth',2, 'Color', [0, 0, 1]);
  end

  if (vect(3) ~= 0)
    plot(arcPts(1,1:end-1), arcPts(2,1:end-1), 'LineWidth', 2);
    qh = quiver(arcPts(1,end-1), arcPts(2,end-1), arcVec(1), arcVec(2), ...
           2, 'LineWidth', 2, 'MaxHeadSize', 100, 'Color', [0, 0, 1]);
  end

  if (~wasHeld)
    hold off;
  end

  end

  %============================= twistPlot =============================
  %
  %  Plots a vector velocity of SE(2) as a vector and a rotation at
  %  the object frame defined by g, presuming that the group element
  %  is given in terms of the frame of reference to plot in.  Typically
  %  will be the body frame in the world frame if it is the body
  %  velocity.  Otherwise, should be the identity element to plot as the
  %  spatial velocity.
  %
  %  When rotation is positive, arc will be in first quadrant.
  %  When negative, arc will be in second quadrant.
  %  When zero, no arc.
  %
  %  Likewise, if linear velocity is zero, then no vector.
  %
  %  TODO: Modify so that linespec can be adjusted.
  %
  function twistPlot(g, xi, rad)

  if (nargin < 3)
    rad = 0.5;
  end

  basePt = g.getTranslation();      % Get base point of twist.
  rotMat = g.getRotation();         % Get rotation matrix to transform ...
  vect = rotMat * xi(1:2);          %   twist vector to obs frame.

  rotAngle = g.getAngle();          % Get rotation angle for velocity.
  thetaVals = rotAngle + linspace(0, 3*pi/6, 20);
  if (xi(3) < 0)
    thetaVals = -thetaVals;
  end
  arcPts = rad*[cos(thetaVals) ; sin(thetaVals)]
                + repmat(basePt, [1, length(thetaVals)]);
  arcVec = diff(arcPts(:,end-1:end),1,2);

  wasHeld = ishold;

  hold on;
  if ((xi(1) ~= 0) || (xi(2) ~= 0))
    quiver(basePt(1), basePt(2), vect(1), vect(2), ...
                                        'LineWidth',2, 'Color', [0, 0, 1]);
  end

  if (xi(3) ~= 0)
    plot(arcPts(1,1:end-1), arcPts(2,1:end-1), 'LineWidth', 2);
    qh = quiver(arcPts(1,end-1), arcPts(2,end-1), arcVec(1), arcVec(2), ...
           0, 'LineWidth', 2, 'MaxHeadSize', 20, 'Color', [0, 0, 1]);
    get(qh)
  end

  if (~wasHeld)
    hold off;
  end

  end

  %================================ pole ===============================
  %
  %
  %================================ pole ===============================
  function qpole = pole(g)
  qpole = inv(eye(2) - g.getRotation())*g.getTranslation();
  end
  
  %=============================== times ===============================
  %
  %  function p2 = times(g, p)
  %
  %
  %  This function is the operator overload that implements the left action
  %  of g on the point p.
  %
  %  Can also be typed as:  >> p2 = g.*p
  %
  %================================= times =================================
  function p2 = times(g, p)
  p2 = leftact(g,p);
  end

  %============================== toHomog ==============================
  %
  %  Return homogeneous form.
  %
  function M = toHomog(g)
  M = g.M;
  end
  
  %============================== toVector =============================
  %
  %  Return vector form.
  %
  function gvec = toVector(g)
  gvec = [g.M(1:2,3); getAngle(g)];
  end

end


methods(Static)
  
  %================================ hat ===============================
  %
  %  Takes a vector form of se(2) and hats it to get the homogeneous
  %  matrix form.
  %
  function xiHat = hat(xiVec)
  JJ = [0 1;-1 0];
  xiHat = [ -JJ*xiVec(3), xiVec(1:2); 0 0 0];
  end

  %=============================== unhat ==============================
  %
  %  Takes a vector form of se(2) and hats it to get the homogeneous
  %  matrix form.
  %
  function xiVec = unhat(xiHat)
  xiVec = [xiHat(1:2,3) ; xiHat(2,1)]; 
  end

  %============================ vec2ProdMat ===========================
  %
  %  Converts the vector form of SE(2) into a producct matrix for use
  %  against vector elements (of points or twists/se(2)).
  %
  function pMat = vec2ProdMat(gVec)
  pMat = [cos(g(3)), -sin(g(3)), 0; sin(g(3)), cos(g(3)), 0; 0 0 1];
  end

  %========================== vec2AdjointMat ==========================
  %
  %  Converts the vector form of SE(2) into an Adjoint matrix for use
  %  against twists/se(2).
  %
  function AdjMat = vec2AdjointMat(gVec)
  tVec = [0, 1;-1, 0]* gVec(1:2);
  AdjMat = [cos(g(3)), -sin(g(3)), tVec(1); ...
          sin(g(3)),  cos(g(3)), tVec(2); 0 0 1];
  end

  %================================ exp ===============================
  %
  %  Computes the exponential of a twist in se(2).
  %
  function gexp = exp(xi, tau)

  if (size(xi,2) == 1)
    xi = SE2.hat(xi);
  end

  if (nargin < 2)
    tau = 1;
  end

  expMat = expm(xi*tau);
  gexp = SE2(expMat(1:2,3), expMat(1:2,1:2));

  end


end

end