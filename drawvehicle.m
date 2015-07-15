
function drawVehicle(uu,V,F,patchcolors)

    % process inputs to function
    pn       = uu(1);       % inertial North position     
    pe       = uu(2);       % inertial East position
    pd       = uu(3);           
    u        = uu(4);       
    v        = uu(5);       
    w        = uu(6);       
    phi      = uu(7);       % roll angle         
    theta    = uu(8);       % pitch angle     
    psi      = uu(9);       % yaw angle     
    p        = uu(10);       % roll rate
    q        = uu(11);       % pitch rate     
    r        = uu(12);       % yaw rate    
    t        = uu(13);       % time
    
    persistent vehicle_handle;
    persistent Vertices
    persistent Faces
    persistent facecolors
    
    % first time function is called, initialize plot and persistent vars
    if t==0,
        figure(1), clf
        [Vertices,Faces,facecolors] = defineVehicleBody;
        vehicle_handle = drawVehicleBody(Vertices,Faces,facecolors,...
                                               pn,pe,pd,phi,theta,psi,...
                                               [],'normal');
        title('Vehicle')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        view(32,47)  % set the vieew angle for figure
        axis([-100,100,-100,100,-100,100]);
        hold on
        
    % at every other time step, redraw base and rod
    else 
        drawVehicleBody(Vertices,Faces,facecolors,...
                           pn,pe,pd,phi,theta,psi,...
                           vehicle_handle);
    end
end

  
%=======================================================================
% drawVehicle
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawVehicleBody(V,F,patchcolors,...
                                     pn,pe,pd,phi,theta,psi,...
                                     handle,mode)
  V = rotate(V, phi, theta, psi);  % rotate vehicle
  V = translate(V, pn, pe, pd);  % translate vehicle
  % transform vertices from NED to XYZ (for matlab rendering)
  R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
  V = R*V;
  
  if isempty(handle),
  handle = patch('Vertices', V', 'Faces', F,...
                 'FaceVertexCData',patchcolors,...
                 'FaceColor','flat',...
                 'EraseMode', mode);
  else
    set(handle,'Vertices',V','Faces',F);
    drawnow
  end
end

%%%%%%%%%%%%%%%%%%%%%%%
function pts=rotate(pts,phi,theta,psi)

  % define rotation matrix (right handed)
  R_roll = [...
          1, 0, 0;...
          0, cos(phi), sin(phi);...
          0, -sin(phi), cos(phi)];
  R_pitch = [...
          cos(theta), 0, -sin(theta);...
          0, 1, 0;...
          sin(theta), 0, cos(theta)];
  R_yaw = [...
          cos(psi), sin(psi), 0;...
          -sin(psi), cos(psi), 0;...
          0, 0, 1];
  R = R_roll*R_pitch*R_yaw;  
  R = R'; %for right handed rotation

  % rotate vertices
  pts = R*pts;
  
end
% end rotateVert

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function pts = translate(pts,pn,pe,pd)

  pts = pts + repmat([pn;pe;pd],1,size(pts,2));
  
end

% end translate


%=======================================================================
% defineVehicleBody
%=======================================================================
function [V,F,facecolors] = defineVehicleBody

% Define the vertices (physical location of vertices
V = [...
%wing coordinates
      0 0 -0.5;... %1
      0 15 -0.5;...%2
      0 -15 -0.5;...%3
      -1.6 -15 -0.5;...%4
      -3 0 -0.5;...%5
      -1.6 15 -0.5;...%6
% inverted v tail coordinates
      -7.58 0 -4.45;... %root 7
      -9.2 0 -4.45;... %root 8
      -7.58 4 0;... %starboard 9
      -10 4 0;... %10
      -7.58 -4 0;...%11
      -10 -4 0;...%12
 % body coordinates
       2.67 0 -1.05;... %13
       0.47 0.68 0;... %14
       0.47 -.68 0;... %15
       0.47 .65 -1.1;... %16
       0.47 -.65 -1.1;... %17
       -4.05 .17 0;... %18
       -4.05 -0.17 0;... %19
       -4.05 .21 -.29;... %20
       -4.05 -.21 -.29;... %21
    ]';

% define faces as a list of vertices numbered above
  F = [...
          2,6,5,1;...
          1,5,4,3;...
          9,10,8,7;...
          7,8,12,11;...
          13,14,15,13;...
          13,14,16,13;...
          13,15,17,13;...
          13,16,17,13;...
          14,18,20,16;...
          16,20,21,17;...
          17,21,19,15;...
          15,19,18,14;...
        ];

% define colors for each face    
  myred = [1, 0, 0];
  mygreen = [0, 1, 0];
  myblue = [0, 0, 1];
  myyellow = [1, 1, 0];
  mycyan = [0, 1, 1];

  facecolors = [...
    mygreen;...    % left wing
    mygreen;...    % right wing
    myblue;...     % tail
    myblue;...
    myred;...
    myred;...
    myred;...
    myred;...
    myred;...
    myred;...
    myred;...
    myred;...
    ];
end
  
