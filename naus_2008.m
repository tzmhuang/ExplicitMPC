%% Main
ts_ = 0.05;
Model = GetPlantModel(ts_);
Model.C = eye(4);
Model.D = zeros(4,1);
dis_plant = ss(Model.A,Model.B,Model.C,Model.D, ts_);

A = [0 1 0 -0.5*ts_;
     0 0 0 -1;
     0 0 0 1;
     0 0 0 0;];
B = [0;
      0;
      0;
      1];
C = eye(4);
D = zeros(4,1);
con_plant = ss(A, B, C, D);
% dis_plant = ss(A, B, C, D, ts_);

con_plant.InputName = {'delta_u'};
con_plant.OutputName = {'x_r', 'v_r', 'v_h', 'u'};
con_plant.StateName = {'x_r', 'v_r', 'v_h', 'u'};
con_plant.InputGroup.MV = 1;
con_plant.OutputGroup.MO = 4;

MPCobj = mpc(dis_plant, ts_)
% Settinge:
% Prediction Horizon and ControlHorizon
MPCobj.PredictionHorizon=10;
MPCobj.ControlHorizon=2;

% Input constraint
MPCobj.MV.Min = -1;
MPCobj.MV.Max = 1;

% State Constraints
MPCobj.OutputVariables(1).Min = 0;
MPCobj.OutputVariables(1).Max = 100;

MPCobj.OutputVariables(2).Min = -10;
MPCobj.OutputVariables(2).Max = 50;

MPCobj.OutputVariables(3).Min = -0.5;
MPCobj.OutputVariables(3).Max = 50;

MPCobj.OutputVariables(4).Min = -3;
MPCobj.OutputVariables(4).Max = 3;

% Weights
MPCobj.W.OutputVariables=[1 1 0 1];
MPCobj.W.ManipulatedVariablesRate=0;

% Sim
% T = 500;
% r = zeros(100,4);
% r(:,1) = 0;
% r(100,:) = [1,0,0,0];
% sim(MPCobj,T,r)
%% Generate EMPC

range = generateExplicitRange(MPCobj);
range.State.Min(:) = -100;
range.State.Max(:) = 100;

range.Reference.Min = [0; 0; 0; 0];
range.Reference.Max = [100; 0; 0; 0];
range.ManipulatedVariable.Min = [MPCobj.ManipulatedVariables.Min] - 1;
range.ManipulatedVariable.Max = [MPCobj.ManipulatedVariables.Max] + 1;

EMPCobj = generateExplicitMPC(MPCobj, range);
% display(EMPCobj)
%% 
function M = GetPlantModel(ts)
    M.A = eye(4) + [0 ts 0 -0.5*ts^2;
                    0 0  0      -ts;
                    0 0  0       ts;
                    0 0  0        0];
    M.B = [0 0 0 1]';
end