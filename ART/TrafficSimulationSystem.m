classdef TrafficSimulationSystem < matlab.System
    % TrafficSimulationSystem - Enhanced SUMO client with traffic light control
    % and induction loop detection
    %
    % Inputs:
    %   trafficLight_J0 - Integer: phase number for junction J0
    %   trafficLight_J1 - Integer: phase number for junction J1  
    %   trafficLight_J2 - Integer: phase number for junction J2
    %   trafficLight_J3 - Integer: phase number for junction J3
    %   trafficLight_J4 - Integer: phase number for junction J4
    %   trafficLight_J5 - Integer: phase number for junction J5
    %   clk             - Clock signal
    %
    % Outputs:
    %   detectorStates    - Vector of 40 boolean values (e1_0 to e1_39)
    %   areaVehicleCount  - Vector of 13 integers: vehicle count in each area detector
    %   areaMeanSpeed     - Vector of 13 doubles: mean speed in each area detector

    properties(Nontunable)
        Host = "127.0.0.1";   % SUMO server host
        Port = 55000;         % SUMO server port
        NumDetectors = 29;    % Number of induction loop detectors
        NumAreaDetectors = 13; % Number of area detectors
        NumTrafficLights = 6;  % Number of traffic lights
    end

    properties(Access = private)
        Client              % tcpclient object
        PreviousTrafficLightStates % Previous states of all traffic lights
        LastDetectorStates  % Hold last detector states
        LastAreaVehicleCount % Hold last area detector vehicle counts
        LastAreaMeanSpeed   % Hold last area detector mean speeds
        AreaDetectorIDs     % Area detector ID mapping
        TrafficLightIDs     % Traffic light junction IDs
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Create TCP connection
            if ~isempty(obj.Client) && isvalid(obj.Client)
                clear obj.Client;
            end
            obj.Client = tcpclient(obj.Host, obj.Port);
            
            % Initialize traffic light states to invalid values
            obj.PreviousTrafficLightStates = -1 * ones(obj.NumTrafficLights, 1);
            obj.LastDetectorStates = false(obj.NumDetectors, 1);
            obj.LastAreaVehicleCount = zeros(obj.NumAreaDetectors, 1);
            obj.LastAreaMeanSpeed = zeros(obj.NumAreaDetectors, 1);
            
            % Define area detector IDs in correct order
            obj.AreaDetectorIDs = ["e2_0", "e2_1", "e2_10", "e2_12", "e2_13", ...
                                  "e2_2", "e2_3", "e2_4", "e2_5", "e2_6", ...
                                  "e2_7", "e2_8", "e2_9"];
            
            % Define traffic light junction IDs
            obj.TrafficLightIDs = ["J0", "J1", "J2", "J3", "J4", "J5"];
        end

        function [detectorStates, areaVehicleCount, areaMeanSpeed] = stepImpl(obj, ...
                trafficLight_J0, trafficLight_J1, trafficLight_J2, ...
                trafficLight_J3, trafficLight_J4, trafficLight_J5, clk)
            
            % Collect all traffic light states
            trafficLightStates = [trafficLight_J0, trafficLight_J1, trafficLight_J2, ...
                                trafficLight_J3, trafficLight_J4, trafficLight_J5];
            
            % Send traffic light commands if states changed
            obj.sendTrafficLightCommands(trafficLightStates);

            % Read simulation data if available
            if obj.Client.NumBytesAvailable > 0
                raw = readline(obj.Client);
                try
                    data = jsondecode(raw);

                    % Extract detector states
                    if isfield(data, "detectors")
                        obj.LastDetectorStates = obj.parseDetectorStates(data.detectors);
                    end

                    % Extract area detector data
                    if isfield(data, "area_detectors")
                        [obj.LastAreaVehicleCount, obj.LastAreaMeanSpeed] = obj.parseAreaDetectorData(data.area_detectors);
                    end

                catch ME
                    warning(ME.identifier, "JSON decode error: %s", ME.message);
                end
            end

            % Always output the last known values
            detectorStates = obj.LastDetectorStates;
            areaVehicleCount = obj.LastAreaVehicleCount;
            areaMeanSpeed = obj.LastAreaMeanSpeed;
        end
        
        function [vehicleCount, meanSpeed] = parseAreaDetectorData(obj, areaDetectorData)
            % Parse area detector data from JSON
            vehicleCount = zeros(obj.NumAreaDetectors, 1);
            meanSpeed = zeros(obj.NumAreaDetectors, 1);
            
            for i = 1:obj.NumAreaDetectors
                detectorID = obj.AreaDetectorIDs(i);
                if isfield(areaDetectorData, detectorID)
                    detData = areaDetectorData.(detectorID);
                    if isfield(detData, 'vehicle_count')
                        vehicleCount(i) = double(detData.vehicle_count);
                    end
                    if isfield(detData, 'mean_speed')
                        meanSpeed(i) = double(detData.mean_speed);
                    end
                end
            end
        end

        function sendTrafficLightCommands(obj, trafficLightStates)
            % Send traffic light control commands to SUMO server
            
            for i = 1:obj.NumTrafficLights
                currentState = trafficLightStates(i);
                
                % Check if state changed
                if obj.PreviousTrafficLightStates(i) ~= currentState
                    junctionID = obj.TrafficLightIDs(i);
                    command = struct('type', 'trafficlight', 'junction', junctionID, 'phase', num2str(currentState));
                    obj.sendCommand(command);
                    obj.PreviousTrafficLightStates(i) = currentState;
                    fprintf('Sent command: Junction %s -> Phase %d\n', junctionID, currentState);
                end
            end
        end

        function sendCommand(obj, command)
            % Send command to SUMO server
            try
                commandStr = jsonencode(command);
                writeline(obj.Client, commandStr);
            catch ME
                warning(ME.identifier,"Failed to send command: %s", ME.message);
            end
        end

        function detectorStates = parseDetectorStates(obj, detectorData)
            % Parse detector states from JSON data
            detectorStates = false(obj.NumDetectors, 1); 
            
            for i = 1:obj.NumDetectors
                detectorID = sprintf("e1_%d", i-1); % e1_0 to e1_39
                if isfield(detectorData, detectorID)
                    detectorStates(i) = logical(detectorData.(detectorID) > 0.1);
                end
            end
        end

        function releaseImpl(obj)
            % Close connection when simulation stops
            clear obj.Client
        end

        function resetImpl(obj)
            % Reset state variables
            obj.PreviousTrafficLightStates = -1 * ones(obj.NumTrafficLights, 1);
        end

        %% ----- Interface specifications -----
        function num = getNumInputsImpl(~)
            num = 7; % 6 traffic lights + clk
        end

        function num = getNumOutputsImpl(~)
            num = 3; % detectorStates, areaVehicleCount, areaMeanSpeed
        end

        function [sz1, sz2, sz3] = getOutputSizeImpl(obj)
            sz1 = [obj.NumDetectors 1];     % detectorStates vector
            sz2 = [obj.NumAreaDetectors 1]; % areaVehicleCount vector
            sz3 = [obj.NumAreaDetectors 1]; % areaMeanSpeed vector
        end

        function [dt1, dt2, dt3] = getOutputDataTypeImpl(~)
            dt1 = 'boolean';    % detectorStates
            dt2 = 'double';     % areaVehicleCount
            dt3 = 'double';     % areaMeanSpeed
        end

        function [c1, c2, c3] = isOutputComplexImpl(~)
            c1 = false; c2 = false; c3 = false;
        end

        function [f1, f2, f3] = isOutputFixedSizeImpl(~)
            f1 = true; f2 = true; f3 = true;
        end

        function [sz1, sz2, sz3, sz4, sz5, sz6, sz7] = getInputSizeImpl(~)
            sz1 = [1 1]; % trafficLight_J0
            sz2 = [1 1]; % trafficLight_J1
            sz3 = [1 1]; % trafficLight_J2
            sz4 = [1 1]; % trafficLight_J3
            sz5 = [1 1]; % trafficLight_J4
            sz6 = [1 1]; % trafficLight_J5
            sz7 = [1 1]; % clk
        end

        function [dt1, dt2, dt3, dt4, dt5, dt6, dt7] = getInputDataTypeImpl(~)
            dt1 = 'int32'; % trafficLight_J0
            dt2 = 'int32'; % trafficLight_J1
            dt3 = 'int32'; % trafficLight_J2
            dt4 = 'int32'; % trafficLight_J3
            dt5 = 'int32'; % trafficLight_J4
            dt6 = 'int32'; % trafficLight_J5
            dt7 = 'double';  % clk
        end

        function [c1, c2, c3, c4, c5, c6, c7] = isInputComplexImpl(~)
            c1 = false; c2 = false; c3 = false; c4 = false; 
            c5 = false; c6 = false; c7 = false;
        end

        function [f1, f2, f3, f4, f5, f6, f7] = isInputFixedSizeImpl(~)
            f1 = true; f2 = true; f3 = true; f4 = true;
            f5 = true; f6 = true; f7 = true;
        end

        function s = saveObjectImpl(obj)
            % Save public properties
            s = saveObjectImpl@matlab.System(obj);
            
            % Save private properties
            s.PreviousTrafficLightStates = obj.PreviousTrafficLightStates;
            s.LastDetectorStates  = obj.LastDetectorStates;
            s.LastAreaVehicleCount = obj.LastAreaVehicleCount;
            s.LastAreaMeanSpeed   = obj.LastAreaMeanSpeed;
            s.AreaDetectorIDs     = obj.AreaDetectorIDs;
            s.TrafficLightIDs     = obj.TrafficLightIDs;
            
            % Client handling
            if ~isempty(obj.Client) && isvalid(obj.Client)
                s.ClientInitialized = true;
            else
                s.ClientInitialized = false;
            end
        end

        function loadObjectImpl(obj, s, wasLocked)
            % Load public properties
            loadObjectImpl@matlab.System(obj, s, wasLocked);
            
            % Restore private properties
            obj.PreviousTrafficLightStates = s.PreviousTrafficLightStates;
            obj.LastDetectorStates  = s.LastDetectorStates;
            obj.LastAreaVehicleCount = s.LastAreaVehicleCount;
            obj.LastAreaMeanSpeed   = s.LastAreaMeanSpeed;
            obj.AreaDetectorIDs     = s.AreaDetectorIDs;
            obj.TrafficLightIDs     = s.TrafficLightIDs;
            
            % Recreate client if necessary
            if wasLocked && s.ClientInitialized
                obj.Client = tcpclient(obj.Host, obj.Port);
            end
        end
    end
end