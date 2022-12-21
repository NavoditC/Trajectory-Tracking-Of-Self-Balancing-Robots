%% Connect to the HC-06 module

bt_connected = 0; % change it to 1 after connecting to bluetooth
if (bt_connected == 0)
    clear; 
    bt = bluetooth("HC-06",1); % connects to the bluetooth module as a serial communication object
    disp("connected")
    fopen(bt); % initialize the serial object
end

nx=4;
ny=1;
nu=1;
nlobj = nlmpc(nx,ny,nu);
Ts = 0.005;
nlobj.Ts=Ts;
nlobj.PredictionHorizon = 4;
nlobj.ControlHorizon = 4;

nlobj.Model.StateFcn = "tumbllerDT0";
nlobj.Model.IsContinuousTime = false;
nlobj.Model.NumberOfParameters = 1;
nlobj.Model.OutputFcn = 'tumbllerOutputFcn';
nlobj.Jacobian.OutputFcn = @(x,u,Ts) [1 0 0 0];

nlobj.Weights.OutputVariables = 1000;
nlobj.Weights.ManipulatedVariablesRate = 1;
nlobj.OV(1).Min = -Inf; %Constraints need to be imposed
nlobj.OV(1).Max = Inf;
nlobj.MV.Min = -Inf; %Constraints need to be imposed
nlobj.MV.Max = Inf;
nloptions = nlmpcmoveopt;
nloptions.Parameters = {Ts};
x = [0;0;-pi;0]; %Initial state
EKF = extendedKalmanFilter(@tumbllerStateFcn, @tumbllerMeasurementFcn);
y = 0.2; %Initial distance
EKF.State = x;
mv=0;
%% Stream and send data to HC-06

% endless loop to keep streaming data from the robot
while (1)
    flushinput(bt); % flush the buffer stored till now
    fscanf(bt); % flushing buffer clips the data stream so have to discard the next input
    rawdata = fscanf(bt); % Store the current data streamed from the robot
    disp(rawdata);
    rawdata = parsedata(rawdata); % parse the raw data and store in a numeric array
    distance = rawdata(1)*0.01;
    [voltage,x_updated] = mpc_func(nlobj,distance,x); % send input data to the mpc function
    flushoutput(bt); % flush output buffer
    fprintf(bt,num2str(voltage*255/5)); % send the output variables to the robot in a string format
    x = x_updated;
end
