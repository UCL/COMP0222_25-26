% This script shows how we can do the same using the MainLoop class, which
% bundles everything together

import ebe.core.*;
import ebe.graphics.*;
import l3s.*;

% Find, load and parse the configuration file
config = ebe.utils.readJSONFile('config/activity2.json');

% Create the mainloop object, which manages everything
mainLoop = ebe.MainLoop(config);

% Create the simulator and register it
simulator = trianglebot.Simulator(config);
mainLoop.setEventGenerator(simulator);

% Set up the figure in which we draw everything
fig = FigureManager.getFigure("Scenario Output");
clf
hold on
axis([-20 20 -20 20])
axis square

% Set up the views which show the output of the simulator
simulatorViewer = ebe.graphics.ViewManager(config);
simulatorViewer.addView(trianglebot.SimulatorView(config, simulator));

% Register the viewer with the mainloop
mainLoop.addViewer(simulatorViewer);

% Run the main loop until it terminates
mainLoop.run();
