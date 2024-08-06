figure;plot(Simulation.Input.Measurements.Ref_Pos(1:end,3),Simulation.Input.Measurements.Ref_Pos(1:end,2))
hold on
plot(Simulation.Output.ESKF.Pos_m(:,2),Simulation.Output.ESKF.Pos_m(:,1),'r')

for i=1:3
  figure;plot(Simulation.Input.Measurements.Ref_Pos(1:end,i+1))
hold on
plot(Simulation.Output.ESKF.Pos_m(:,i),'r')
end

figure;plot(Real_Measurement.Heading_interp(:,2))
hold on;plot(Simulation.Output.ESKF.O_corrected(:,9)*180/pi,'r')

figure
plot(Simulation.Output.ESKF.O_corrected(:,13:15))

figure;plot(Simulation.Output.Kalman_mtx.P_diag(:,13:15))

