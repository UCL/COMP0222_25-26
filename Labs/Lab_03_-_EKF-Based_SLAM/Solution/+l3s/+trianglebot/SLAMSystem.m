classdef SLAMSystem < ebe.slam.SLAMSystem
    % SLAMSystem summary of SLAMSystem
    %
    % The SLAMSystem implements SLAM for the trianglebot using EKF-SLAM.

    properties(Access = protected)

        % Although in the lectures we presented three types of states
        % (estimated, predicted, partial) from an implementation point of
        % view it's easier just to store the one set
        x;
        P;

        % The most recent version of the odometry together with its
        % measurement covariance
        u;
        sigmaU;

        % Map stores landmark ID with the indices in the state vector
        landmarkIDStateVectorMap;

        % The linear system used to predict the state and the observation
        systemModel;

        % The map
        scenario;

        % Store of the mean and covariance values
        timeStore;
        xStore;
        PStore;

        % For activity 3
        updateKnownLandmarks;
    end

    methods(Access = public)

        function obj = SLAMSystem(config)
            % SLAMSystem Constructor for SLAMSystem
            %
            % Syntax:
            %   slamSystem = SLAMSystem(config)
            %
            % Description:
            %   Creates an instance of a SLAMSystem object. The system model
            %   and is constructed at this time and a first set of event
            %   handlers are scheduled.
            %
            % Inputs:
            %   config - (struct)
            %       The configuration structure
            %
            % Outputs:
            %   slamSystem - (handle)
            %       An instance of a SLAMSystem

            % Call base class
            obj@ebe.slam.SLAMSystem(config);

            % Set up the discrete time system for prediction
            obj.systemModel = l3s.trianglebot.SystemModel(config);

            % Update landmarks by default
            obj.updateKnownLandmarks = true;

            % Set up the event handlers
            obj.registerEventHandler('init', @obj.handleInitializationEvent);
            obj.registerEventHandler('null_obs', @obj.handleNoUpdate);
            obj.registerEventHandler('gps', @obj.handleGPSObservationEvent);
            obj.registerEventHandler('slam', @obj.handleSLAMObservationEvent);
            obj.registerEventHandler('odom', @obj.handleUpdateOdometryEvent);
            obj.registerEventHandler('compass', @obj.handleCompassObservationEvent);

            % Set the name
            obj.setName('SLAMSystem');
        end

        function setUpdateKnownLandmarks(obj, updateKnownLandmarks)
            % SETUPDATEKNOWNLANDMARKS Set the flag to determine if known
            % landmarks should be updated
            %
            % Syntax:
            %   simulator.setUpdateKnownLandmarks(updateKnownLandmarks)
            %
            % Description:
            %   In this lab, some of the activities involve looking at the
            %   initialization step only. To do this, the update of known
            %   landmarks must be skipped. By default, known landmarks are
            %   updated.
            %
            % Inputs:
            %   updateKnownLandmarks - (bool)
            %       If false, known landmarks will not be updated.

            obj.updateKnownLandmarks = updateKnownLandmarks;
        end

        function success = start(obj)
            % START Start the SLAM system
            %
            % Syntax:
            %   slamSystem.start()
            %
            % Description:
            %   Start the simulator. This includes clearing any results
            %   history and any map.

            start@ebe.slam.SLAMSystem(obj);

            % Set up initial store of results
            obj.timeStore = [];
            obj.xStore = zeros(l3s.trianglebot.SystemModel.NP, 0);
            obj.PStore = zeros(l3s.trianglebot.SystemModel.NP, 0);

            % Set the dictionary which maps landmark ID to coefficient in
            % the state estimate.
            obj.landmarkIDStateVectorMap = configureDictionary("uint32", "double");

            % Get the map data
            if (isfield(obj.config, 'scenario'))
                obj.scenario = obj.config.scenario;
            end

            success = true;
        end

        function [x,P] = platformEstimate(obj)
            x = obj.x(1:l3s.trianglebot.SystemModel.NP);
            P = obj.P(1:l3s.trianglebot.SystemModel.NP, 1:l3s.trianglebot.SystemModel.NP);
        end
        
        function [T, X, PX] = platformEstimateHistory(obj)
            T = obj.timeStore;
            X = obj.xStore;
            PX = obj.PStore;
        end
        
        function [m, Pmm, landmarkIds] = landmarkEstimates(obj)
            % LANDMARKESTIMATES Return the current mean and covariance of
            % each landmark estimate.
            %
            % Syntax:
            %   [m, Pmm, landmarkIds] = slamSystem.platformEstimate()
            %
            % Description:
            %   Return the current estimates of the landmarks and the IDs.
            %   The landmark covariances are just the blocks on the
            %   diagonals. The full landmark covariance block is not
            %   returned.
            %
            %   At a given time, there are Nk landmarks. The dimension of
            %   each landmark is 2
            %
            % Outputs:
            %   m - (2xN_k vector)
            %       A column vector which contains the estimated mean of
            %       each landmark position.
            %   PX - (2x2xN_k double psd matrix)
            %       A three dimensional matrix which stores the landmark
            %       estimates. The covariance of landmark 4, for example,
            %       is given by P(:,:,4)

            % Get the number of landmarks
            landmarkIds = keys(obj.landmarkIDStateVectorMap);
            numberOfLandmarks = numel(landmarkIds);

            % Build the mean and covariance matrices and extract
            m = NaN(l3s.trianglebot.SystemModel.NL, numberOfLandmarks);
            Pmm = NaN(l3s.trianglebot.SystemModel.NL, l3s.trianglebot.SystemModel.NL, numberOfLandmarks);
            
            for l = 1 : numberOfLandmarks
                landmarkId = landmarkIds(l);
                offset = lookup(obj.landmarkIDStateVectorMap, landmarkId);
                idx = offset + [1;2];
                m(:, l) = obj.x(idx);
                Pmm(:, :, l) = obj.P(idx, idx);
            end
            
        end
    end

    methods(Access = protected)

        function success = handleNoPrediction(obj)
            % HANDLENOPREDICTION Handle the case when no prediction is
            % needed.
            %
            % Syntax:
            %   slamSystem.handleNoPrediction();
            %
            % Description:
            %   This method is called when the time difference between two
            %   events is so small that running the time prediction step
            %   isn't required.

            success = true;
        end

        function success = handleNoUpdate(obj, ~)
            % HANDLENOUPDATE Handle the case when no update is needed.
            %
            % Syntax:
            %   slamSystem.handleNoPrediction();
            %
            % Description:
            %   This method is called when no update to the SLAM system is
            %   required. For example, this is the case with heartbeat and
            %   null_obs events.

            success = true;
        end

        function success = handlePredictForwards(obj, dT)
            % HANDLEPREDICTFORWARDS Predict the system state forwards
            %
            % Syntax:
            %   slamSystem.handlePredictForwards(dT);
            %
            % Description:
            %   Predict the system state forward by a time step dT. The
            %   process model is called with the current platform state
            %   estimate and odometry value. The estimated state and
            %   Jacobians are returned. The covariance estimate is updated
            %   from the Jacobians.
            %
            % Inputs:
            %   dT - (double)
            %       Length of the prediction step.

            % Convenience value
            NP = l3s.trianglebot.SystemModel.NP;

            % Update the platform state
            [obj.x(1:NP), gradFx, gradFv] = obj.systemModel.predictState(obj.x(1:NP), obj.u, dT);

            % Update the top left block of the platform state from
            % gradF*Pxx*gradF'+gradV*Q*gradV'
            obj.P(1:NP,1:NP) = gradFx * obj.P(1:NP, 1:NP) * gradFx' + gradFv * obj.sigmaU * gradFv';

            % Do the platform landmark-prediction blocks
            obj.P(1:NP, NP+1:end) = gradFx * obj.P(1:NP, NP+1:end);
            obj.P(NP+1:end, 1:NP) = obj.P(1:NP, NP+1:end)';

            success = true;
        end

        function success = handleInitializationEvent(obj, event)
            % HANDLEINITIALIZATIONEVENT Handle the initialization event.
            %
            % Syntax:
            %   slamSystem.handleInitializationEvent(event);
            %
            % Description:
            %   Handle the initialization event. data is assumed to be x0,
            %   and covariance P0. The initialized flag is also set to true
            %   to enable the estimator to run in full mode.
            %
            % Inputs:
            %   event - (ebe.core.Event)
            %       The intialization event.
            %
            % See Also:
            %   SIMULATOR

            obj.x = event.data;
            obj.P = event.covariance;
            obj.initialized = true;
            success = true;
        end

        % Handle a set of measurements of landmarks
        function success = handleSLAMObservationEvent(obj, event)
            % HANDLESLAMOBSERVATIONEVENT Handle the SLAM observation event.
            %
            % Syntax:
            %   slamSystem.handleSLAMObservationEvent(event);
            %
            % Description:
            %   Handle SLAM observations. This follows the code in the
            %   lectures: any known landmarks are updated first. Unknown
            %   landmarks are augmented at the end. Each landmark is
            %   processed individually rather than batching them together.
            %
            % Inputs:
            %   event - (ebe.core.Event)
            %       The SLAM observation event.
            %
            % See Also:
            %   SIMULATOR
            
            assert(obj.stepNumber == event.eventGeneratorStepNumber)

            % Store useful values
            NL = l3s.trianglebot.SystemModel.NL;
            NP = l3s.trianglebot.SystemModel.NP;

            % Get the list of landmarks we know about
            knownLandmarkIDs = obj.landmarkIDStateVectorMap.keys();

            % Find the intersection of the observed landmarks and the ones
            % known in the map; this produces a list of known landmarks
            % which have been observed this time.
            [existingLandmarks, idx] = intersect(event.info, knownLandmarkIDs);

            % If updating landmarks is disabled, we clear the index of
            % known landmarks. This is an easy way to cause the next block
            % to be skipped
            if (obj.updateKnownLandmarks == false)
                idx = [];
            end

            % Update all the known landmarks
            for o = 1 : numel(idx)
                % Look up the landmark and figure out its position
                offset = lookup(obj.landmarkIDStateVectorMap, existingLandmarks(o));
                landmarkIdx = offset + (1:NL);

                % Predicted observation and Jacobians
                [zPred, gradHx, gradHm, gradHw] = ...
                    obj.systemModel.predictSLAMObservation(obj.x(1:NP), ...
                    obj.x(landmarkIdx));

                % Work out the innovation, including angle wrapping
                nu = event.data(:, idx(o)) - zPred;
                nu(2) = atan2(sin(nu(2)), cos(nu(2)));

                % Assemble the observation matrix
                HS = zeros(2, numel(obj.x));
                HS(:, 1:NP) = gradHx;
                HS(:, landmarkIdx) = gradHm;

                % Kalman filter update steps
                C = obj.P * HS';
                S = HS * C + gradHw * event.covariance() * gradHw';
                K = C / S;
                obj.x = obj.x + K * nu;
                obj.P = obj.P - K * S * K';

                % Wrap the heading estimate
                obj.x(3) = atan2(sin(obj.x(3)), cos(obj.x(3)));

            end

            % Find the mutual complement  of the observed landmarks and the
            % ones known in the map; this produces a list of new landmarks
            % which have not been seen before.
            [newLandmarks, idx] = setdiff(event.info, existingLandmarks);

            % Augment all the known landmarks
            for o = 1 : numel(newLandmarks)

                % Figure out the index for the new state estimate and
                % insert it into the look up table
                stateDim = length(obj.x);
                landmarkIdx = stateDim + (1:NL);
                obj.landmarkIDStateVectorMap = ...
                    insert(obj.landmarkIDStateVectorMap, newLandmarks(o), stateDim);

                % Use the inverse observation model to estimate the
                % landmark position and compute the Jacobian
                [m, gradGx, gradGw] = ...
                    obj.systemModel.predictLandmarkFromSLAMObservation(obj.x, event.data(:, idx(o)));

                % Expand the state with the initial estimate of the
                % landmark position
                obj.x(landmarkIdx) = m;

                % Add the predicted landmark covariance. We do this first
                % because it forces obj.P to be resized to the correct
                % dimension
                obj.P(landmarkIdx, landmarkIdx) = gradGx * obj.P(1:NP, 1:NP) * gradGx' ...
                    + gradGw * event.covariance() * gradGw';

                % Compute the cross correlation
                obj.P(landmarkIdx, 1:end-NL) = gradGx * obj.P(1:NP, 1:end-NL);

                % Copy over tranposed version
                obj.P(1:end-NL,landmarkIdx) = obj.P(landmarkIdx, 1:end-NL)';
            end
            
            success = true;
        end

        function success = handleGPSObservationEvent(obj, event)
            % HANDLEGPSOBSERVATIONEVENT Handle the GPS observation event.
            %
            % Syntax:
            %   slamSystem.handleGPSObservationEvent(event);
            %
            % Description:
            %   Handle the GPS observation.
            %
            % Inputs:
            %   event - (ebe.core.Event)
            %       The GPS observation event.
            %
            % See Also:
            %   SIMULATOR

            % Predicted observation and Jacobians
            [zPred, gradHx, gradHw] = ...
                obj.systemModel.predictGPSObservation(obj.x(1:2));

            % Compute the innovation
            nu = event.data - zPred;

            % Assemble the observation matrix
            HS = zeros(2, numel(obj.x));
            HS(:, 1:l3s.trianglebot.SystemModel.NP) = gradHx;

            % Kalman Filter Update
            C = obj.P * HS';
            S = HS * C + gradHw * event.covariance() * gradHw';
            W = C / S;
            obj.x = obj.x + W * nu;
            obj.P = obj.P - W * S * W';

            % Wrap the heading estimate
            obj.x(3) = atan2(sin(obj.x(3)), cos(obj.x(3)));

            success = true;
        end

        function success = handleCompassObservationEvent(obj, event)
            % HANDLECOMPASSOBSERVATIONEVENT Handle the compass observation event.
            %
            % Syntax:
            %   slamSystem.handleCompassObservationEvent(event);
            %
            % Description:
            %   Handle the compass observation.
            %
            % Inputs:
            %   event - (ebe.core.Event)
            %       The compass observation event.
            %
            % See Also:
            %   SIMULATOR

            % Predicted observation and Jacobians
            [zPred, gradHx, gradHw] = ...
                obj.systemModel.predictCompassObservation(obj.x(1:l3s.trianglebot.SystemModel.NP));

            % Compute the innovation
            nu = event.data - zPred;
            nu = atan2(sin(nu), cos(nu));

            % Expand to the full state
            HS = zeros(1, numel(obj.x));
            HS(:, 1:l3s.trianglebot.SystemModel.NP) = gradHx;

            % Kalman Filter Update
            C = obj.P * HS';
            S = HS * C + gradHw * event.covariance() * gradHw';
            W = C / S;
            obj.x = obj.x + W * nu;
            obj.P = obj.P - W * S * W';

            % Wrap the heading estimate
            obj.x(3) = atan2(sin(obj.x(3)), cos(obj.x(3)));

            success = true;
        end


        function success = handleBearingObservationEvent(obj, event)
            % HANDLEBEARINGOBSERVATIONEVENT Handle the bearing observation event.
            %
            % Syntax:
            %   slamSystem.handleBearingObservationEvent(event);
            %
            % Description:
            %   Handle the bearing observation event. For each observation,
            %   we extract the sensor ID and from that work out the bearing
            %   sensor position and orientation. This is then used to carry
            %   out the update.
            %
            % Inputs:
            %   event - (ebe.core.Event)
            %       The bearing observation event.
            %
            % See Also:
            %   SIMULATOR

            % Update each measurement separately
            for s = 1 : numel(event.info)

                % Predicted observation and Jacobians
                sensor = obj.map.sensors.bearing.sensors(event.info(s));
                [zPred, gradHx, gradHw] = ...
                obj.systemModel.predictBearingObservation(obj.x(1:2), ...
                    sensor.position, sensor.orientation);

                % Compute the innovation
                nu = event.data(s) - zPred;
                nu = atan2(sin(nu), cos(nu));

                % Expand to full state
                HS = zeros(1, numel(obj.x));
                HS(:, 1:l2s.trianglebot.SystemModel.NP) = gradHx;
                
                % Kalman Filter Update
                C = obj.P * HS';
                S = HS * C + gradHw * event.covariance() * gradHw';
                W = C / S;
                obj.x = obj.x + W * nu;
                obj.P = obj.P - W * S * W';

                % Wrap the heading estimate
                obj.x(3) = atan2(sin(obj.x(3)), cos(obj.x(3)));
            end

            success = true;
        end

        function success = handleUpdateOdometryEvent(obj, event)
            % HANDLEUPDATEODOMETRYEVENT Handle receving new odometry
            % information.
            %
            % Syntax:
            %   slamSystem.handleUpdateOdometryEvent(event);
            %
            % Description:
            %   Handle the odometry event. The internal value of odometry
            %   is updated to the value in the event. The odometry is held
            %   constant until the next handle event is received.
            %
            % Inputs:
            %   event - (ebe.core.Event)
            %       The odometry event.
            %
            % See Also:
            %   SIMULATOR

                assert(obj.stepNumber == event.eventGeneratorStepNumber);
                obj.u = event.data;
                obj.sigmaU = event.covariance;
                success = true;
            end

        function storeStepResults(obj)
            % Store the estimate for the future
            obj.timeStore(:, obj.stepNumber + 1) = obj.currentTime;
            obj.xStore(:, obj.stepNumber + 1) = obj.x(1:l3s.trianglebot.SystemModel.NP);
            obj.PStore(:, obj.stepNumber + 1) = diag(obj.P(1:l3s.trianglebot.SystemModel.NP, ...
                1:l3s.trianglebot.SystemModel.NP));
        end
    end
end