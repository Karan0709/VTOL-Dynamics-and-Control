
function out = forces_moments(x, delta, P)

%inputs
% ------------------------------------------------------------------------- 
    pn      = x(1);
    pe      = x(2);
    pd      = x(3);
    u       = x(4);
    v       = x(5);
    w       = x(6);
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);
    p       = x(10);
    q       = x(11);
    r       = x(12);
    delta_e = delta(1);
    delta_a = delta(2);
    Fr      = delta(3);
    Fl      = delta(4);
    Beta_x  = delta(5);
    Fb      = delta(6);
    delta_r = delta(7);
%     w_ns    = wind(1); % steady wind - North
%     w_es    = wind(2); % steady wind - East
%     w_ds    = wind(3); % steady wind - Down
%     u_wg    = wind(4); % gust along body x-axis
%     v_wg    = wind(5); % gust along body y-axis    
%     w_wg    = wind(6); % gust along body z-axis
    
    % compute wind data in NED
     w_n = 0;
     w_e = 0;
     w_d = 0;
     
% ------------------------------------------------------------------------- 

%gust model not implemented yet.

% -------------------------------------------------------------------------     
     
    it=90*pi/180;  %tilt angle

    % compute air data 
    V = sqrt(u^2 + v^2 +w^2);
    alpha = atan2(w,u);
    beta  = atan2(v,sqrt(u^2+w^2));
    qbar = 0.5*P.rho*V;
    ca    = cos(alpha);
    sa    = sin(alpha);
    cit   = cos(it);
    sit   = sin(it);
    cb    = cos(Beta_x);
    sb    = sin(Beta_x);     
% -------------------------------------------------------------------------   
% external forces and torques on aircraft
    
     % compute gravitaional forces
    Force(1) = -P.mass*P.gravity*sin(theta);
    Force(2) =  P.mass*P.gravity*cos(theta)*sin(phi);
    Force(3) =  P.mass*P.gravity*cos(theta)*cos(phi);
% -------------------------------------------------------------------------     
     % compute Lift and Drag forces
    tmp1 = exp(-P.M*(alpha-P.alpha0));
    tmp2 = exp(P.M*(alpha+P.alpha0));
    sigma = (1+tmp1+tmp2)/((1+tmp1)*(1+tmp2));
    CL = (1-sigma)*(P.C_L_0+P.C_L_alpha*alpha);
    AR = 13.1;
    e = .9;
    CD = P.C_D_0 + 1/pi/e/AR*(P.C_L_0+P.C_L_alpha*alpha)^2;
    if alpha>=0, 
        CL = CL + sigma*2*sa*sa*ca;
    else
        CL = CL - sigma*2*sa*sa*ca;
    end
% -------------------------------------------------------------------------     
     % compute aerodynamic forces
    Force(1) = Force(1) + qbar*P.S_wing*(-CD*ca*u + CL*sa*u);
    Force(1) = Force(1) + qbar*P.S_wing*(-P.C_D_q*ca + P.C_L_q*sa)*P.c*q/(2);
    
    Force(2) = Force(2) + qbar*P.S_wing*(P.C_Y_0 + P.C_Y_beta*beta)*u;
    Force(2) = Force(2) + qbar*P.S_wing*(P.C_Y_p*p + P.C_Y_r*r)*P.b/(2);
    
    Force(3) = Force(3) + qbar*P.S_wing*(-CD*sa - CL*ca)*u;
    Force(3) = Force(3) + qbar*P.S_wing*(-P.C_D_q*sa - P.C_L_q*ca)*P.c*q/(2);
% -------------------------------------------------------------------------     
      % compute aerodynamic torques
    
    Torque(1) = qbar*P.S_wing*P.b*(P.C_ell_0 + P.C_ell_beta*beta)*u;
    Torque(1) = Torque(1) + qbar*P.S_wing*P.b*(P.C_ell_p*p + P.C_ell_r*r)*P.b/(2);

    Torque(2) = qbar*P.S_wing*P.c*(P.C_M_0 + P.C_M_alpha*alpha)*u;
    Torque(2) = Torque(2) + qbar*P.S_wing*P.c*P.C_M_q*P.c*q/(2);

    
    Torque(3) = qbar*P.S_wing*P.b*(P.C_n_0 + P.C_n_beta*beta)*u;
    Torque(3) = Torque(3) + qbar*P.S_wing*P.b*(P.C_n_p*p + P.C_n_r*r)*P.b/(2);
% -------------------------------------------------------------------------     
      % compute control forces
    Force(1) = Force(1) + qbar*P.S_wing*(-P.C_D_delta_e*ca+P.C_L_delta_e*sa)*delta_e*u;
    Force(2) = Force(2) + qbar*P.S_wing*(P.C_Y_delta_a*delta_a + P.C_Y_delta_r*delta_r)*u;
    Force(3) = Force(3) + qbar*P.S_wing*(-P.C_D_delta_e*sa-P.C_L_delta_e*ca)*delta_e*u;
% -------------------------------------------------------------------------      
    % compute control torques
    Torque(1) = Torque(1) + qbar*P.S_wing*P.b*(P.C_ell_delta_a*delta_a + P.C_ell_delta_r*delta_r)*u;
    Torque(2) = Torque(2) + qbar*P.S_wing*P.c*P.C_M_delta_e*delta_e*u;
    Torque(3) = Torque(3) + qbar*P.S_wing*P.b*(P.C_n_delta_a*delta_a + P.C_n_delta_r*delta_r)*u;
% -------------------------------------------------------------------------     
     % compute propulsion forces
%   motor_temp = P.k_motor^2*delta_t^2-u^2;
    Force(1) = Force(1) + (Fr+Fl)*ca*sit - Fb*sa*cb;
    Force(2) = Force(2) + Fb*sb*ca; 
    Force(3) = Force(3) - Fb*cb*ca - (Fr+Fl)*ca*cit ; 
% -------------------------------------------------------------------------    
    %compute propulsion torques
    Torque(2) = Torque(2) + (Fr+Fl)*cit*ca*P.Lf - Fb*cb*P.Lb*ca - (((Fb^3/2)/(2*P.rho*P.S_prop)^0.5)/(712))*sb;
    Torque(3) = Torque(3) + (((Fb^3/2)/(2*P.rho*P.S_prop)^0.5)/(712))*cb - Fb*sb*P.Lb*ca ; %- (((Fr^3/2)/(2*P.rho*P.S_prop)^0.5)/(712))*cit +(((Fl^3/2)/(2*P.rho*P.S_prop)^0.5)/(712))*cit 
    
% ------------------------------------------------------------------------- 
%     Cruise model
% ------------------------------------------------------------------------- 

%    CXalpha    = -P.CD_alpha * cos(alpha)    + P.CL_alpha * sin(alpha); 
%    CX0        = -P.CD0 * cos(alpha)        + P.CL_0 * sin(alpha);
%    CXq        = -P.CD_q * cos(alpha)        + P.CL_q * sin(alpha);
%    CX_delta_e = -P.CD_delta_e * cos(alpha) + P.CL_delta_e * sin(alpha);
%    
%    CZ0        = -P.CD0 * sin(alpha)        - P.CL_0 * cos(alpha);
%    CZalpha    = -P.CD_alpha * sin(alpha)    - P.CL_alpha * cos(alpha);
%    CZq        = -P.CD_q * sin(alpha)        - P.CL_q * cos(alpha);
%    CZ_delta_e = -P.CD_delta_e * sin(alpha) - P.CL_delta_e * cos(alpha);
   
%     Force(1) =  -P.mass*P.gravity*sin(theta)+ 2*0.5*P.rho*P.S_prop *P.Cprop *((P.k_motor*delta_t)^2 - V^2)*sin(it)*cos(alpha)- Fb*cos(Beta_x)*sin(alpha) + 0.5*P.rho*V^2*P.S_wing * ( (CXalpha * alpha)  + ((CXq * P.c * q)/(2*V))   + (CX_delta_e*delta_e) ) ;
%     Force(2) =  P.mass*P.gravity*cos(theta)*sin(phi) + Fb*sin(Beta_x) + 0.5*P.rho*V^2*P.S_wing * ( P.CY_0  + (P.CY_beta * beta) + ((P.CY_p * P.b * p)/(2*V)) + ((P.CY_r * P.b * r)/(2*V)) + (P.CY_delta_a * delta_a) + (P.CY_delta_r * delta_r) );
%     Force(3) =  P.mass*P.gravity*cos(theta)*cos(phi)  - 2*(0.5*P.rho*P.S_prop*1*((80*delta_t)^2-V^2))*cos(it)*cos(alpha) - Fb*cos(Beta_x)*cos(alpha) + 0.5*P.rho*V^2*P.S_wing * ( (CZalpha * alpha)  + ((CZq * P.c * q)/(2*V))   + (CZ_delta_e*delta_e) );
    
%     Torque(1) = -(P.k_T_P*(P.k_Omega*delta_t)^2)*sin(it) + 0.5*P.rho*V^2*P.S_wing * (P.b) * ( P.CL_0  + (P.Cell_beta * beta)   + (P.Cell_p * P.b * p)/(2*V)  + (P.Cell_r * P.b * r)/(2*V) + (P.Cell_delta_a * delta_a) + (P.Cell_delta_r * delta_r) );
%     Torque(2) =  2*(0.5*P.rho*P.S_prop*1*((80*delta_t)^2-V^2))*cos(it)*P.Lf  - Fb*cos(Beta_x)*P.Lb + 0.5*P.rho*V^2*P.S_wing * P.c  * ( P.Cm_0  + (P.Cm_alpha * alpha) + ((P.Cm_q * P.c * q)/(2*V)) + (P.Cm_delta_e * delta_e) ); %(((Fb^3/2)/(2*P.rho*P.S_prop)^0.5)/(712))*sin(Beta_x)
%     Torque(3) =  (((Fb^3/2)/(2*P.rho*P.S_prop)^0.5)/(712))*cos(Beta_x) - Fb*sin(Beta_x)*P.Lb + 0.5*P.rho*V^2*P.S_wing * (P.b) * ( P.Cn_0  + (P.Cn_beta * beta)   + (P.Cn_p * P.b * p)/(2*V)  + (P.Cn_r * P.b * r)/(2*V) + (P.Cn_delta_a * delta_a) + (P.Cn_delta_r * delta_r) ) ;


% ------------------------------------------------------------------------- 
%     Hover model
%     Trim values are Fr = 25.9608N, Fb= 16.8861 and Beta_x = 1.0156
% ------------------------------------------------------------------------- 
%     motor_temp = P.k_motor^2*delta_t^2-V^2 ;
%     Fr = 0.5*P.rho*P.S_prop*P.C_prop*motor_temp*ca;
%     Force(1) =  -P.mass*P.gravity*sin(theta);
%     Force(2) =  0;
%     Force(3) =  P.mass*P.gravity*cos(theta)*cos(phi) - Fr-Fl-Fb*cos(Beta_x);
%    
%     Torque(1) = -Fr*P.Ls + Fl*P.Ls;
%     Torque(2) =  Fr*P.Lf + Fl*P.Lf - Fb*cos(Beta_x)*P.Lb - (((Fb^3/2)/(2*P.rho*P.S_prop)^0.5)/(712))*sin(Beta_x);     
%     Torque(3) = (((Fb^3/2)/(2*P.rho*P.S_prop)^0.5)/(712))*cos(Beta_x)-Fb*sin(Beta_x)*P.Lb; 
   
     out = [Force'; Torque'; V; alpha; beta;delta;p;q;r;u;phi;theta;psi;];
     
end



