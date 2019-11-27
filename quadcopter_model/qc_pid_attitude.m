function varargout = qc_pid_attitude(ts)
%% Controller tuning for LPV inner loop
% kettos integratorok szabalyozasa

z = tf('s');
G = c2d(1/z^2,ts);
C0 = tunablePID('C','pd', ts);
C0.Tf.Value = ts;
C0.Tf.Free = 0;
AP = AnalysisPoint('u');

T0 = feedback(G*AP*C0,1);
T0.InputName = 'r';
T0.OutputName = 'y';

Req1 = TuningGoal.Tracking('r','y',0.15);
Req2 = TuningGoal.Margins('u',6,60);

Reqs = [Req1 Req2];
rng('default')
Options = systuneOptions('RandomStart',3);
[T,fSoft] = systune(T0,Reqs,Options);

C = getBlockValue(T,'C');
G_roll = C;
G_pitch = C;
G_yaw = C;

varargout{1} = G_roll;
varargout{2} = G_pitch;
varargout{3} = G_yaw;

%out = [ G_roll G_pitch G_yaw ];
%%clearvars z G C0 AP T0 Req1 Req2 Options T fSoft C Reqs