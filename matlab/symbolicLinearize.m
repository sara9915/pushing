%% Symbolic Manipulation
clear variables
close all
clc

a = 0.09;
b = 0.09;
nu_p = 0.3;
nu_g = .35;
m = 1.05;
g = 9.81;
fmax = nu_g*m*g; 
mmax = m_max_funct(nu_g,m);
c = (fmax/mmax);

% Declare Equilibrium variables
u_star = [0.05;0];
ry_star= [0];
x_eq= [0;0;0];
u_lower_bound = [-0.01; 0.1];
u_upper_bound = [0.1; 0.1];
x_lower_bound = [100; 100; 100; 100];
x_upper_bound = [100; 100; 100; 100];

%Symbolic variables
syms x y theta a xp yp
syms ry  u1 u2
%Build states                        
x_state = [x;y;theta;ry];
u_state = [u1;u2];
%Kinematics
Cbi = Helper.C3_2d(theta);
rx = -0.09/2;
rbpb = [rx;ry];

  
%% Stick: vo = vp, Slide Up: vo = vo_up, Slide Down: vo = vo_down
%Define gamma=vt/vn
gamma_top    = (nu_p*c^2 - rx*ry + nu_p*rx^2)/(c^2 + ry^2 - nu_p*rx*ry);
gamma_bottom = (-nu_p*c^2 - rx*ry - nu_p*rx^2)/(c^2 + ry^2 + nu_p*rx*ry);
C_top = jacobian(gamma_top,x_state);
C_top = subs(C_top, ry, {ry_star});
C_top = double(C_top);
C_bottom = jacobian(gamma_bottom,x_state);
C_bottom = subs(C_bottom, ry, {ry_star});
C_bottom = double(C_bottom);
C_top_linear = double(C_top);
C_bottom_linear = double(C_bottom);
gammaTop_star    =    double(subs(gamma_top, ry, ry_star));
gammaBottom_star = double(subs(gamma_bottom, ry, ry_star));
%Determine effective pusher velocity equation
for lv1=1:3
    if lv1 == 1;%Sticking
        %Define motion cone boundary vector
        vo = [u1;u2];
        vox = u1;
        voy = u2;
    elseif lv1==2;%Sliding up
        v_MC = [1;gamma_top];
        vo = (u1/v_MC(1))*v_MC; 
        vox = vo(1);
        voy = vo(2);
    else
        v_MC = [1;gamma_bottom];
        vo = (u1/v_MC(1))*[v_MC(1);v_MC(2)];
        vox = vo(1);
        voy = vo(2);
    end
    %Body frame kinematics
    dx_b{lv1} = ((c^2+rx^2)*vox + rx*ry*voy)/(c^2+rx^2+ry^2);
    dy_b{lv1} = ((c^2+ry^2)*voy + rx*ry*vox)/(c^2+rx^2+ry^2);
    dtheta{lv1} = 1/c^2*(rx*dy_b{lv1} - ry*dx_b{lv1});
    %Kinematics
    drbbi{lv1} = [dx_b{lv1};dy_b{lv1}];
    dribi{lv1} = transpose(Cbi)*drbbi{lv1};
    drbpb{lv1} = [u1;u2] - [dx_b{lv1};dy_b{lv1}] - Helper.cross3d([dtheta{lv1}], [rbpb]);
    dry{lv1}   = simplify(drbpb{lv1}(2))  ;%[u1;u2] - vo;%
    %Build nonlinear function
    f{lv1} = [dribi{lv1};dtheta{lv1};dry{lv1}];
    %Build jacobians
    A{lv1} = jacobian(f{lv1},x_state);
    B{lv1} = jacobian(f{lv1},u_state);
    % Substitute equilibrium states
    A{lv1} = subs(A{lv1},{x,y,theta ry},{x_eq(1),x_eq(2),x_eq(3), ry_star});
    A{lv1} = subs(A{lv1},{u1,u2},{u_star(1),u_star(2)});
    B{lv1} = subs(B{lv1},{x,y,theta ry},{x_eq(1),x_eq(2),x_eq(3), ry_star});
    B{lv1} = subs(B{lv1},{u1,u2},{u_star(1),u_star(2)});
    %Convert to double type
    A{lv1}=double(A{lv1});
    B{lv1}=double(B{lv1});
%         Compute LQR solution
%         K = lqr(A,B,obj.Q_LQR,obj.R_LQR);
%         obj.K = K;  
    %Set properties
    A_linear{lv1} = double(A{lv1});
    B_linear{lv1} = double(B{lv1});
end


function n_f = m_max_funct(nu, m)     
            n_f_integrand = @(p1, p2) (nu * m * Helper.g / PusherSlider.A) * sqrt([p1; p2; 0]' * [p1; p2; 0]);
            n_f = Helper.DoubleGaussQuad(n_f_integrand, -PusherSlider.a / 2, PusherSlider.a / 2, -PusherSlider.b / 2, PusherSlider.b / 2);
end 
