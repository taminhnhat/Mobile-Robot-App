load exampleMaps.mat
startLoc = [5 10];
goalLoc = [32 10];
thisMap = complexMap;
open_system('pathPlanningSimulinkModel.slx')
simulation = sim('pathPlanningSimulinkModel.slx');
