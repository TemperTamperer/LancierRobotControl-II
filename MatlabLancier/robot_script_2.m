%% init
close all
clear
%%
% motor inerti
ankerRadius = 0.0115;
ankerLength = 0.025;
ankerVolumen = ankerRadius^2*pi*ankerLength
ankerMass = 8000 * ankerVolumen
ankerInerti = 0.5 * ankerMass * ankerRadius^2
%% make control time series
% format: 
% [time(sec) , longitudinal velocity (m/s), rotation velocity (rad/sec)]
vel = [0, 0, 0; ...
       0.1, 1, 0;
       0.2, 2, 0; 
       0.3, 3.5, 0; 
       0.4, 5, 0; 
       0.5, 7.5, 0;
       0.6, 5, 0;
       0.8, 3.5, 0;
       1.0, 7, 0;
       1.2, 1, 0;...
       1.5, 0,   0; ...
       ]
% [time(sec) , longitudinal velocity (m/s) , heading (rad)]
vel2 = [0, 0, 0; ...
        0.1, 5, 0;
        2.1, 0, 0;
       ]

vel3 = [0, 0, 0; ...
       ]