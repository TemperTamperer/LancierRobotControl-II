close all
clear
%%
% % logfile from REGBOT:
% % Motor test log
% % 1 time stamp (sec)
% % 2 Motor test type (0 = CV, 1 = CCV)
% % 3 Left motor voltage
% % 4 Left motor current
% %  5 Left motor velocity
% % 6 Right motor voltage
% % 7 Right motor current
% % 8 Right motor velocity
% 11.9932 0 0 0.00 -0.000 -0.00  0.00 0.000 0.00
data_00 = load('log_motortest_00.txt');
data_01 = load('log_motortest_01.txt');
% Motor test log
% 1 time stamp (sec)
% 2 Motor test type (0 = CV, 1 = CCV)
% 3 sample number
% 4 Left motor voltage
% 5 Left motor current
% 6 Left motor velocity
% 7 Right motor voltage
% 8 Right motor current
% 9 Right motor velocity
% 10 left current AD
% 11 AD[0]
% 12 right current AD
% 13 AD[1]
data_10 = load('log_motortest_10.txt');
data_11 = load('log_motortest_11.txt');
data_12 = load('log_motortest_12.txt'); 
data_14 = load('log_motortest_14.txt'); % 3 x måling af left motor current (left right of ad[1]
data_15 = load('log_motortest_15.txt'); % 3 x måling af left motor current (left right of ad[1]
data_17 = load('log_motortest_17.txt'); % 3 x måling af left motor current (left right of ad[1]
data_18 = load('log_motortest_18.txt'); % back to normal

%% plot pose
% velocities
dd = data_18;
fig = 1000 + 1000 * 18;
%%
figure(fig + 1)
hold off
plot(dd(:,1), dd(:,4)/10)
grid on
hold on
plot(dd(:,1), dd(:,5))
%plot(dd(1:end-1,1), diff(dd(:,6)))
title('left')
%axis([0,1,-1.2,1.2])
legend('Voltage','current A' , 'velocity')
%%
figure(fig + 10)
hold off
plot(dd(:,1), dd(:,4))
grid on
hold on
plot(dd(:,1), dd(:,10))
plot(dd(:,1), dd(:,11))
plot(dd(:,1), dd(:,12))
plot(dd(:,1), dd(:,13))
title('AD debug')
%axis([0,1,-1.2,1.2])
legend('Voltage','ad raw' , 'AD1', 'right current AD', 'AD2')
%%
figure(fig + 2)
hold off
hold off
plot(dd(:,1), dd(:,7)/10)
grid on
hold on
plot(dd(:,1), dd(:,8))
%plot(dd(1:end-1,1), diff(dd(:,9)))
title('right')
%axis([0,1,-1.2,1.2])
legend('Voltage','current A' , 'velocity')
%% calculation
s1 = 700;
s2 = 1200;
e1 = 999;
e2 = 1499;
n1 = s1:e1;
n2=  s2:e2;
llowCur = mean(dd(n1,5));
rlowCur = mean(dd(n1,8));
lHighCur = mean(dd(n2,5));
rHighCur = mean(dd(n2,8));
llowvel = (dd(e1,6) - dd(s1,6))/(e1-s1+1);
lHighvel = (dd(e2,6) - dd(s2,6))/(e2-s2+1);
rlowvel = (dd(e1,9) - dd(s1,9))/(e1-s1+1);
rHighvel = (dd(e2,9) - dd(s2,9))/(e2-s2+1);
rlowvel = mean(dd(n1,9));
rHighvel = mean(dd(n2,9));
lLowVolt = mean(dd(n1,4));
lHighVolt = mean(dd(n2,4));
rLowVolt = mean(dd(n1,7));
rHighVolt = mean(dd(n2,7));
%% calculation
lcurRel = lHighCur/llowCur
lhvdif = lHighvel - llowvel * lcurRel
lK = (lHighVolt - lLowVolt * lcurRel)/lhvdif
rcurRel = lHighCur/llowCur
rhvdif = rHighvel - rlowvel * rcurRel
rK = (rHighVolt - rLowVolt * rcurRel)/rhvdif
