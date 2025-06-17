log = ardupilotreader('00000203.BIN');

IMUmsg  = readMessages(log,'MessageName',{'IMU'});
% IMUmsg  = readMessages(log,'MessageName',{'XKF1'});


%%

imu0_msgs = IMUmsg(IMUmsg.InstanceID==0,:);
allData = vertcat( imu0_msgs.MsgData{:} );


TimeUS = allData.TimeUS;  % cell array of char or string array

% 1) Parse to a duration (hours/minutes/seconds + fractional)
ts_nano = uint64( round(milliseconds(TimeUS) * 1e6));


out = table( ...
    ts_nano, ...
    allData.GyrX, allData.GyrY, allData.GyrZ, ...
    allData.AccX, allData.AccY, allData.AccZ, ...
    'VariableNames',{'timestamp_ns','GyrX','GyrY','GyrZ','AccX','AccY','AccZ'} );

writetable(out,'imudata_203.csv');
