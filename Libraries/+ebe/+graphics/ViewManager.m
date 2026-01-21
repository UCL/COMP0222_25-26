classdef ViewManager < ebe.core.ConfigurableComponent
    % ViewManager summary of ViewManager
    %
    % Given the complexity of the systems used, ebe supports a hierarchical
    % model for figure management. A ViewManager, typically associated with
    % a single figure, is responsible for managing a set of "Views". Each
    % View typically involves visualizing the state of a component. For
    % example, a SLAM system viewer, which maps to a single figure, would
    % include the View which shows the ground truth, together with a View
    % which shows the status of the estimator.
    %
    % See Also:
    %   ebe.graphics.View

    properties(Access = protected)

        % The set of views
        views;

        % The figure name associated with this viewer
        figureName;

        % Handle for the figure
        figureState;

    end

    methods(Access = public)

        function obj = ViewManager(config, figureInfo)
            % ViewManager Constructor for ViewManager
            %
            % Syntax:
            %   viewManager = ViewManager(config, figureInfo);
            %
            % Description:
            %   Create a view manager. The manager encapsulates a set of "views"
            %   and is associated with a single figure. If figureInfo is
            %   provided, it must either be a string (which is then
            %   searched for) or the figure state directly. If the figure
            %   state is provided, the figure name is queried from it. If
            %   nothing is provided, the view manager will bind to the
            %   current figure returned by gcf.
            %
            % Inputs:
            %   config - (struct)
            %       The configuration structure
            %   figureInfo - (string or ebe.graphics.FigureState)
            %       The event generator to be visualized
            %
            % Outputs:
            %   obj - (ebe.graphics.ViewManager)
            %       An instance of the ViewManager
            %
            % See Also:
            %   ebe.graphics.FigureState

            % Run the base class constructor
            obj@ebe.core.ConfigurableComponent(config);

            if (nargin == 2)
                % If the type if a string, copy the name over but leave the
                % figure state blank. If the type is a figure state, assign
                % the figure state directly and copy the figure name.
                if (isa(figureInfo, 'char') == true)
                    obj.figureName = figureInfo;
                elseif (isa(figureInfo, 'ebe.graphics.FigureState') == true)
                    obj.figureName = figureInfo.name();
                    obj.figureState = figureInfo;
                else
                    error('viewmanager:constructor:wrongfigureinfo', ...
                        ['figureInfo is of the wrong type; it ' ...
                        'should be a string or ebe.graphics.FigureState ' ...
                        'but is of class %s'], class(figureInfo));
                end
            end

            obj.views = {};
        end

        function addView(obj, view)
            % ADDVIEW Add a view to this view manager
            %
            % Syntax:
            %   viewManager.addView(viewInstance)
            %
            % Description:
            %   Adds a view to this view manager. These should be added
            %   before the view manager is started (no error checking!)
            %   Also, these should be only registered with a single view
            %   manager (again, no error checking!)
            %
            % Inputs:
            %   view - (ebe.graphics.View)
            %       An instance of the view object to be added
            %
            %
            % See also:
            %   ebe.graphics.View

            assert(isa(view, 'ebe.graphics.View'), ...
                'ebe:graphics:viewmanager:wrongviewtype', ...
                ['The view is the wrong type; it should inherit from '...
                'ebe.graphics.View but is of type %s'], class(view));
            obj.views{end+1} = view;
        end

        function start(obj)
            % START Start the view manager
            %
            % Syntax:
            %   viewManager.start()
            %
            % Description:
            %   The view manager will first create / open the registered
            %   figure. It will then start all the views associated with
            %   this view manager. It accumulates the legend information
            %   from any of the views and uses these to assemble a legend
            %   table which is displayed.
            %
            % See also:
            %   ebe.graphics.View
            
            % If the figure state isn't selected already, get one. The
            % only way the figure state can be set is through the
            % constructor
            if (isempty(obj.figureName) == false)
                if (isempty(obj.figureState) == true)
                    obj.figureState = ebe.graphics.FigureManager.getFigure(obj.figureName);
                end
                obj.figureState.select();
            end

            % Start everything, pulling legends as we go
            legendHandles = [];
            legendEntries = {};
            for v = 1 : numel(obj.views)
                obj.views{v}.start();
                [handles, entries] = obj.views{v}.legendEntries();
                if (isempty(entries) == false)
                    legendHandles(end + 1) = handles;
                    legendEntries{end + 1} = entries;
                end
            end
            if (isempty(legendEntries) == false)
                legend(legendHandles, legendEntries);
            end
        end

        function stop(obj)
            for v = 1 : numel(obj.views)
                obj.views{v}.stop();
            end
            obj.figureState = [];
        end

        function visualize(obj, eventArray)
            for v = 1 : numel(obj.views)
                obj.views{v}.visualize(eventArray);
            end
        end
    end
end