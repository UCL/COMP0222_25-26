classdef MainLoop < ebe.core.ConfigurableComponent
    % MainLoop summary of MainLoop
    % A class which manages the entire loop for an ebe session,
    % from starting up estimators and graphics, to running the main loop
    % with the event generator, to shutting the system back down again.

    properties(Access = protected)

        % Flag shows if the main loop is running
        isRunning;

        % The event generator
        eventGenerator;

        % The set of estimators
        estimators;

        % The set of post step activities
        resultsAccumulators;

        % The set of viewers
        viewers;

        % The set of post draw activities
        postDrawActions;

        % Flag to show if graphics are enabled
        enableGraphics;

        % Timestep between saving results off
        accumulateResultsUpdatePeriod;

        % Timestep between updating the graphics
        graphicsUpdatePeriod;

    end

    methods(Access = public)

        function obj = MainLoop(config)
            % MainLoop Constructor for MainLoop
            %
            % Syntax:
            %   mainLoop = MainLoop(config);
            %
            % Description:
            %   Constructor for an instance of a MainLoop object.
            %
            % Inputs:
            %   config - (struct)
            %       The configuration structure
            %
            % Outputs:
            %   mainLoop - (handle)
            %      An instance of a MainLoop

            % Call the base class constructor
            obj@ebe.core.ConfigurableComponent(config);

            % Clear out all the entries and set defaults
            obj.enableGraphics = true;
            obj.isRunning = false;
            obj.estimators = {};
            obj.resultsAccumulators = {};
            obj.viewers = {};
            obj.postDrawActions = {};
            obj.graphicsUpdatePeriod = 1;
            obj.accumulateResultsUpdatePeriod = 1;
        end

        function setEventGenerator(obj, eventGenerator)
            % SETEVENTGENERATOR set the event generator for the mainloop
            %
            % Syntax:
            %   mainLoop.setEventGenerator(eventGenerator)
            %
            % Description:
            %   The main loop must have a single event generator which is
            %   responsible for generating events. This method registers
            %   the generator. Note the generator can only be set while the
            %   main loop is not running
            %
            % Inputs:
            %   eventGenerator - (ebe.core.EventGenerator)
            %       The event generator object

            % First check the object is of the right type
            assert(isa(eventGenerator, 'ebe.core.EventGenerator'), ...
                'ebe:mainloop:invalidestimatortype', ...
                'The event generator should inherit from ebe.core.EventGenerator');

            % Second check we aren't running
            assert(obj.isRunning == false, 'ebe:mainloop:setwhilerunning', ...
                'The method was called while the mainloop is running')
            obj.eventGenerator = eventGenerator;
        end

        function addEstimator(obj, estimator)
            % ADDESTIMATOR add an estimator to the mainloop
            %
            % Syntax:
            %   mainLoop.addEstimator(estimator)
            %
            % Description:
            %   Add the estimator to the list of estimators which will be
            %   fired when events are available. No order is specified in
            %   which esimators are fired. Note an estimator can only be
            %   added while the main loop is not running
            %
            % Inputs:
            %   eventGenerator - (ebe.core.EventBasedEstimator)
            %       The estimator
            %
            % See Also:
            %   ebe.core.EventBasedEstimator

            % First check the object is of the right type
            assert(isa(estimator, 'ebe.core.EventBasedEstimator'), ...
                'ebe:mainloop:invalideventgeneratortype', ...
                'The event generator should inherit from ebe.core.EventBasedEstimator');

            % Second check we aren't running
            assert(obj.isRunning == false, 'ebe:mainloop:setwhilerunning', ...
                'The method was called while the mainloop is running')

            obj.estimators{end+1} = estimator;
        end

        function addResultsAccumulator(obj, resultsAccumulator)
            % ADDRESULTSACCUMULATOR add a results accumulator to the mainloop
            %
            % Syntax:
            %   mainLoop.addResultsAccumulator(resultsAccumulator);
            %
            % Description:
            %   Add a results accumulator to the mainloop. Results
            %   accumulators collect data from the event generator and the
            %   estimator to do things like compute errors. Note an 
            %   accumulator can only be added while the main loop is not
            %   running
            %
            % Inputs:
            %   eventGenerator - (ebe.core.ResultsAccumulator)
            %       The estimator
            %
            % See Also:
            %   ebe.core.ResultsAccumulator
           
            % First check the object is of the right type
            assert(isa(resultsAccumulator, 'ebe.core.ResultsAccumulator'), ...
                'ebe:mainloop:invalidresultsaccumulatortype', ...
                'The event generator should inherit from ebe.core.ResultsAccumulator');

            % Second check we aren't running
            assert(obj.isRunning == false, 'ebe:mainloop:setwhilerunning', ...
                'The method was called while the mainloop is running')

            obj.resultsAccumulators{end+1} = resultsAccumulator;
        end

        function addViewer(obj, viewer)
            % ADDVIEWER Add a graphical viewer
            %
            % Syntax:
            %   success = mainLoop.addViewer(viewer)
            %
            % Description:
            %   Add a viewer to the mainloop. A graphical viewer
            %   is responsible for viewing aspects of a system. Typically a
            %   viewer maps to a single matlab figure, but can contain
            %   multiple graphical elements, such as all the graphics
            %   needed to show the ground truth and the state of an
            %   estimator. Note a viewer can only be added while the main
            %   loop is not running
            %
            % Inputs:
            %   viewer - (ebe.graphics.ViewManager)
            %       The event to be processed
            %
            % See Also:
            %   ebe.graphics.ViewManager

            % First check the object is of the right type
            assert(isa(viewer, 'ebe.graphics.ViewManager'), ...
                'ebe:mainloop:invalidviewertype', ...
                'The event generator should inherit from ebe.graphics.ViewManager');

            % Second check we aren't running
            assert(obj.isRunning == false, 'ebe:mainloop:setwhilerunning', ...
                'The method was called while the mainloop is running')

            obj.viewers{end+1} = viewer;
        end

        function addPostDrawAction(obj, action)
            % ADDPOSTDRAWACTION Register a post draw action.
            %
            % Syntax:
            %   mainLoop.addPostDrawAction(action);
            %
            % Description:
            %   Register a post draw action which is called after all the
            %   other steps in the loop have been completed.
            %
            % Inputs:
            %   action - (ebe.graphics.PostDrawAction)
            %       The action to be added.

            % First check the object is of the right type
            assert(isa(estimator, 'ebe.graphics.PostDrawAction'), ...
                'ebe:mainloop:invalidpostdrawactiontype', ...
                'The event generator should inherit from ebe.graphics.PostDrawAction');

            obj.postDrawActions{end+1} = action;
        end        

        function setGraphicsUpdatePeriod(obj, graphicsUpdatePeriod)
            % SETGRAPHICSUPDATEPERIOD Set the frequency with which the
            % graphics are updated
            %
            % Syntax:
            %   mainLoop.setGraphicsUpdatePeriod(graphicsUpdatePeriod)
            %
            % Description:
            %   Updating the graphics can be very costly and slow the
            %   simulation down significantly. This method sets a variable
            %   which determines the rate at which updates happen.
            %   Specifically. an update happens if the event count mod the
            %   graphicsUpdatePeriod is 0.
            %
            % Inputs:
            %   graphicsUpdatePeriod - (int)
            %       The number of simulator timesteps between updating the
            %       graphics.
            
            obj.graphicsUpdatePeriod = graphicsUpdatePeriod;
        end

        function setAccumulateResultsUpdatePeriod(obj, accumulateResultsUpdatePeriod)
            % SETACCUMULATEREESULTSUPDATEPERIOD Set the frequency with which the
            % results (ground truth and estimates) are stored.
            %
            % Syntax:
            %   mainLoop.setAccumulateResultsUpdatePeriod(accumulateResultsUpdatePeriod)
            %
            % Description:
            %   Getting results from some estimators (I'm looking at you
            %   factor graphs) can take time. Therefore, this method makes
            %   it posible to modify the frequency with which the results
            %   accumulator is called.
            %
            % Inputs:
            %   graphicsUpdatePeriod - (int)
            %       The number of simulator timesteps between updating the
            %       graphics.
            
            obj.accumulateResultsUpdatePeriod = accumulateResultsUpdatePeriod;
        end

        function estimatorSet = getEstimators(obj)
            % GETESTIMATORS Return the set of estimators which are registered
            % with the main loop.
            %
            % Syntax:
            %   estimators = mainLoop.getEstimators()
            %
            % Description:
            %   Return the set of estimators registered with this main
            %   loop. The estimators appear in the order registered with
            %   the mainloop.
            %
            % Outputs:
            %   estimators - (cell array of ebe.core.EventBasedEstimator)
            %       The cell array of the estimators
            estimatorSet = obj.estimators;
        end

        function run(obj)
            % RUN execute the mainloop
            %
            % Syntax:
            %   mainLoop.run();
            %
            % Description:
            %   Run the mainloop from start to finish. After starting all
            %   the components, the runOneStep method is called repeatedly
            %   until the event generator says its done. At that point, all
            %   the registered components are stopped.
            %
            % See Also:
            %   runOneStep, start, stop
            
            % Start all the components
            obj.start();

            % Keep executing the inner one step until the event generator
            % says its done
            while (obj.eventGenerator.keepRunning()  == true)
                obj.runOneStep();
            end
        
            % Stop all the components
            obj.stop();

        end

        function runOneStep(obj)
            % RUNONESTEP run a single step in the mainloop
            %
            % Syntax:
            %   mainLoop.runOneStep();
            %
            % Description:
            %   Run a single step in the mainloop: the event generator
            %   yields a set of events which are dispatched to the
            %   estimators. The results are collected and the graphics
            %   updated. Note that if this method is used, start and stop
            %   will have to be called externally.
            %
            % See Also:
            %   run, start, stop

            % Step the event generator
            obj.eventGenerator.step();

            % Get the events
            events = obj.eventGenerator.events();

            % Handle if there are any estimators
            for e = 1 : numel(obj.estimators)
                obj.estimators{e}.processEvents(events);
            end

            % Handle if there are any results accumulators
            if (rem(obj.eventGenerator.stepCount(), obj.accumulateResultsUpdatePeriod) ==0)
                for p = 1 : numel(obj.resultsAccumulators)
                    obj.resultsAccumulators{p}.collectResults();
                end
            end

            % Finish here if graphics is disabled
            if (obj.enableGraphics == false)
                return
            end

            % Do not update if we haven't reached the next graphics update
            % step
            if (rem(obj.eventGenerator.stepCount(), obj.graphicsUpdatePeriod) ~= 0)
                return
            end

            % Run the visualizers
            for v = 1 : numel(obj.viewers)
                obj.viewers{v}.visualize(events);
            end
            drawnow

            % Handle if there are any post visualize activities
            for p = 1 : numel(obj.postDrawActions)
                obj.postDrawActions{p}.run();
            end
        end

        function start(obj)
            % START start all the registered components
            %
            % Syntax:
            %   mainLoop.start();
            %
            % Description:
            %   The start method is called on the event generator, the
            %   estimators, results accumulators, graphics viewers and post
            %   draw actions. This is done automatically by the run method,
            %   or must be called manually if the runOneStep method is
            %   used.
            %
            % See Also:
            %   run, runOneStep, stop            
            
            % Flag that the component is running, so you can't change
            % anything
            obj.isRunning = true;

            % Start the event generator
            obj.eventGenerator.start();
            
            % Handle if there are any estimators
            for e = 1 : numel(obj.estimators)
                obj.estimators{e}.start();
            end

            % Handle if there are any post step activities
            for p = 1 : numel(obj.resultsAccumulators)
                obj.resultsAccumulators{p}.setEventGeneratorAndEstimators(obj.eventGenerator, obj.estimators);
                obj.resultsAccumulators{p}.start();
            end

            % Handle if there are any viewers
            if (obj.enableGraphics == false)
                return
            end
            for v = 1 : numel(obj.viewers)
                obj.viewers{v}.start();
            end

            % Handle if there are any post visualize activities
            for p = 1 : numel(obj.postDrawActions)
                obj.postDrawActions{p}.start();
            end
        end

        function stop(obj)
            % STOP stop all the registered components
            %
            % Syntax:
            %   mainLoop.stop();
            %
            % Description:
            %   The stop method is called on the event generator, the
            %   estimators, results accumulators, graphics viewers and post
            %   draw actions. This is done automatically by the run method,
            %   or must be called manually if the runOneStep method is
            %   used.
            %
            % See Also:
            %   run, runOneStep, start
            
            % Stop the event generator
            obj.eventGenerator.stop();
            
            % Handle if there are any estimators
            for e = 1 : numel(obj.estimators)
                obj.estimators{e}.stop();
            end

            % Handle if there are any post step activities
            for p = 1 : numel(obj.resultsAccumulators)
                obj.resultsAccumulators{p}.stop();
            end

            % Handle if there are any viewers
            if (obj.enableGraphics == false)
                return
            end
            
            for v = 1 : numel(obj.viewers)
                obj.viewers{v}.stop();
            end

            % Handle if there are any post visualize activities
            for v = 1 : numel(obj.postDrawActions)
                obj.postDrawActions{p}.stop();
            end

            % Show that this is no longer running
            obj.isRunning = false;

        end
    end
end