function [P_eng, P_elec, w_eng_rpm, G_cvt, fuel_rate] = fcn(v_veh, P_trac, SOC, param, eng_map)
%#codegen
%inputs:
    % v_veh, m/s
    % P_trac, W
    % SOC
    % param, structure of parameters
    % eng_map

% outputs:
    %   P_gen, P_elec in W
    %   w_eng_rpm, rpm
    %   G_cvt (cvt ratio)
    %   Fuel rate, grams/hr

v_veh_min = param.v_veh_min;    % minimum vehicle speed for engine to stay engaged, in m/s
P_eng_min = param.P_eng_min;    % minimum engine power in W
P_eng_max = param.P_eng_max ;   % maximum engine power
G_cvt_min = param.G_cvt_min;    % minimum cvt  ratio
G_diff = param.G_diff;          % differential gear ratio
r_wheel = param.r_wheel;        % wheel radius in m

if (v_veh < v_veh_min)  % disengage clutch, idle engine, electric propulsion
   P_elec = P_trac;
   P_eng = 0;
   fuel_rate = 0; % g/hr;
   w_eng_rpm = 1000; % rpm
   G_cvt = G_cvt_min;
   return  
end

% if here, v_veh > v_veh_min
if(P_trac < P_eng_min)   % clutch engaged but engine idling
   P_elec = P_trac;
   fuel_rate = 0;
   P_eng = 0;
   w_eng_rpm =  1000; % rpm
   w_eng = w_eng_rpm * pi / 30; % rad/s
   % set G_cvt so engine speed is 1000 rpm
   G_cvt = v_veh/G_diff/w_eng/r_wheel;
   return     
end

if(P_trac > P_eng_max) % high-speed boost
   P_elec = P_trac - P_eng_max;
   P_eng = P_eng_max;
   bsfc = interp1(eng_map(:,2), eng_map(:,3), P_eng/1000, 'pchip', 'extrap');
   fuel_rate = bsfc*P_eng/1000; % grams/hr
   w_eng_rpm = interp1(eng_map(:,2), eng_map(:,1), P_eng/1000, 'pchip', 'extrap');
   w_eng = w_eng_rpm * pi / 30; % convert to rad/s
   G_cvt =  v_veh/r_wheel/G_diff/w_eng; % required CVT ratio
   return   
end

% if here, v_veh > v_veh_min and P_eng_min < P_trac < P_eng_max
% try to get SOC back to 0.5

P_elec = 20000*(SOC - 0.5); 
if(P_elec > 4000)
    P_elec = 4000;
end
if(P_elec < -4000)
    P_elec = -4000;
end

P_eng = P_trac - P_elec;


if(P_eng < P_eng_min) 
   % clutch engaged, but no fuel
   P_eng = 0;
   P_elec = P_trac;
   fuel_rate = 0;
   w_eng_rpm = 1000;
   w_eng = w_eng_rpm * 2 * pi / 60; % in rad/s
   % set G_cvt so engine speed is 1000 rpm
   G_cvt = v_veh/G_diff/w_eng/r_wheel;
   return
end

if(P_eng > P_eng_max)
    P_eng = P_eng_max;
    P_elec = P_trac - P_eng_max;
end

bsfc = interp1(eng_map(:,2), eng_map(:,3), P_eng/1000, 'pchip', 'extrap');
fuel_rate = bsfc*P_eng/1000; % grams/hr

w_eng_rpm = interp1(eng_map(:,2), eng_map(:,1), P_eng/1000, 'pchip', 'extrap');
if(w_eng_rpm < 1000)
    w_eng_rpm = 1000;
end
w_eng = w_eng_rpm * pi / 30;  % convert to rad/s
G_cvt =  v_veh/r_wheel/G_diff/w_eng;

if (G_cvt < G_cvt_min) % set G_cvt = G_cvt_min, recalculate w_eng, P_eng, and P_elec
   G_cvt = G_cvt_min;
   w_eng = v_veh/G_diff/r_wheel/G_cvt;
   w_eng_rpm = w_eng*30/pi;  % in rpm
   P_eng = 1000*interp1(eng_map(:,1), eng_map(:,2), w_eng, 'pchip', 'extrap'); % in W
   if(P_eng < 0)
       P_eng = 0;
   end
   P_elec = P_trac - P_eng;
   bsfc = interp1(eng_map(:,1), eng_map(:,3), w_eng, 'pchip', 'extrap');
   fuel_rate = bsfc*P_eng/1000; % grams/hr
 end

