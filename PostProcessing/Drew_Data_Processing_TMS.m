close all
clear all
clc

Drew_Data = readtable('C:\Users\Trenton\Desktop\SIO Courses\Winter24\MAE223\yei_team4.txt', 'TextType','string');

length = size(Drew_Data,1);

Acc_Data_Raw = zeros(length,3);
for i = 1:length
    Acc_Data_Raw(i,:) = str2num(string(Drew_Data.Var8{i}));
end

Acc_Data_Process = zeros(length,3);
for i = 1:length
    Acc_Data_Process(i,:) = str2num(string(Drew_Data.Var5{i}));
end



fig1 = figure(1)
subplot(3,1,1)
plot((Drew_Data.Var3(:)-Drew_Data.Var3(1))/1000000 ,Acc_Data_Raw(:,1))
subplot(3,1,2)
plot((Drew_Data.Var3(:)-Drew_Data.Var3(1))/1000000 ,Acc_Data_Raw(:,2))
subplot(3,1,3)
plot((Drew_Data.Var3(:)-Drew_Data.Var3(1))/1000000 ,Acc_Data_Raw(:,3))
sgtitle('RAW')


figure(2)
subplot(3,1,1)
plot((Drew_Data.Var3(:)-Drew_Data.Var3(1))/1000000 ,Acc_Data_Process(:,1))
subplot(3,1,2)
plot((Drew_Data.Var3(:)-Drew_Data.Var3(1))/1000000 ,Acc_Data_Process(:,2))
subplot(3,1,3)
plot((Drew_Data.Var3(:)-Drew_Data.Var3(1))/1000000 ,Acc_Data_Process(:,3))
sgtitle('Processed')

figure(fig1)
subplot(3,1,1)
hold on
plot((Drew_Data.Var3(:)-Drew_Data.Var3(1))/1000000 ,Acc_Data_Process(:,1),'--r')
subplot(3,1,2)
hold on
plot((Drew_Data.Var3(:)-Drew_Data.Var3(1))/1000000 ,Acc_Data_Process(:,2),'--r')
subplot(3,1,3)
hold on
plot((Drew_Data.Var3(:)-Drew_Data.Var3(1))/1000000 ,Acc_Data_Process(:,3),'--r')
sgtitle('Processed')


%% Our Data

Team4_Data = readtable('C:\Users\Trenton\Documents\GitHub\MAE223_Drifter\DrewSwung.csv');

figure
subplot(3,1,1)
plot(Team4_Data.millis/1000 ,Team4_Data.accX)
subplot(3,1,2)
plot(Team4_Data.millis/1000 ,Team4_Data.accY)
subplot(3,1,3)
plot(Team4_Data.millis/1000 ,Team4_Data.accZ)
sgtitle('Team 4 Data')