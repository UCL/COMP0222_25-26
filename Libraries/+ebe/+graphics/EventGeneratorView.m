classdef EventGeneratorView < ebe.graphics.View
    % EventGeneratorView summary of EventGeneratorView
    %
    % Subclasses of this object are used to visualize an event generator

    properties(Access = protected)

        % The event generator to be visualized
        eventGenerator;
    end

    methods(Access = public)

        function obj = EventGeneratorView(config, eventGenerator)
            % EventGeneratorView Constructor for EventGeneratorView
            %
            % Syntax:
            %   eventGeneratorView = EventGeneratorView(config, eventGenerator);
            %
            % Description:
            %   Constructor for an instance of a View object for an event
            %   generator.
            %
            % Inputs:
            %   config - (struct)
            %       The configuration structure
            %   eventGenerator - (ebe.core.EventGenerator)
            %       The event generator to be visualized

            % Run the base class constructor
            obj@ebe.graphics.View(config);

            % Save the event generator data
            obj.eventGenerator = eventGenerator;
        end
            
    end
end