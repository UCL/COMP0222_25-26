classdef SLAMSystemView < ebe.graphics.EstimatorView

    properties(Access = protected)

        % Geometry for the platform
        platformStateDrawer;
        platformPoseDrawer;

        % Geometry for the landmarks
        landmarkEstimateDrawer;

    end

    methods(Access = public)

        function obj = SLAMSystemView(config, eventGenerator)
            obj@ebe.graphics.EstimatorView(config, eventGenerator);
        end
        
        function start(obj)

            % Drawer for the platform state
            platformColour = ebe.graphics.DistinguishableColours.assignColour('estimated slam platform');
            obj.platformStateDrawer = ebe.graphics.MeanCovarianceDrawer(platformColour);
            obj.platformPoseDrawer = ebe.graphics.OrientedTriangleDrawer(platformColour);

            % Drawer for the landmark states
            landmarkColour = platformColour;%ebe.graphics.DistinguishableColours.assignColour('SLAM Ground Truth');

            obj.landmarkEstimateDrawer = ebe.graphics.MultipleMeanCovarianceDrawer(landmarkColour);
        end

        function visualize(obj, events)

            [x,P] = obj.estimator.platformEstimate();

            obj.platformStateDrawer.update(x(1:2) ,P(1:2, 1:2));
            obj.platformPoseDrawer.update(x);

            [xl, Pl] = obj.estimator.landmarkEstimates();

            obj.landmarkEstimateDrawer.update(xl, Pl);

        end


    end

end