classdef TrafficSimulationSystem < matlab.System
    % TrafficSimulationSystem - SUMO client with inductive loops, area detectors,
    % instant induction loops (pedestrian detection), and 6 traffic lights (J1–J6).
    %
    % Inputs:
    %   trafficLight_J1 - Phase number for junction J1
    %   trafficLight_J2 - Phase number for junction J2
    %   trafficLight_J3 - Phase number for junction J3
    %   trafficLight_J4 - Phase number for junction J4
    %   trafficLight_J5 - Phase number for junction J5
    %   trafficLight_J6 - Phase number for junction J6
    %   clk             - Clock
    %
    % Outputs:
    %   detectorStates
    %   areaVehicleCount
    %   areaMeanSpeed
    %   pedestrianCount

    properties(Nontunable)
        Host = "127.0.0.1";
        Port = 55000;

        NumDetectors = 4;
        NumInstantLoops = 8;
        NumAreaDetectors = 18;

        NumTrafficLights = 6;    % <-- updated to 6
    end

    properties(Access = private)
        Client
        PreviousTrafficLightStates

        LastDetectorStates
        LastPedestrianCount
        LastAreaVehicleCount
        LastAreaMeanSpeed

        AreaDetectorIDs
        InstantLoopIDs
        TrafficLightIDs
    end

    methods(Access = protected)

        %% ============================================================
        %                          SETUP
        %% ============================================================
        function setupImpl(obj)

            % Create TCP connection
            if ~isempty(obj.Client) && isvalid(obj.Client)
                clear obj.Client;
            end
            obj.Client = tcpclient(obj.Host, obj.Port);

            % Initialize state buffers
            obj.PreviousTrafficLightStates = -1 * ones(obj.NumTrafficLights, 1);
            obj.LastDetectorStates = false(obj.NumDetectors, 1);
            obj.LastPedestrianCount = zeros(obj.NumInstantLoops, 1);
            obj.LastAreaVehicleCount = zeros(obj.NumAreaDetectors, 1);
            obj.LastAreaMeanSpeed = zeros(obj.NumAreaDetectors, 1);

            % Area detector IDs
            obj.AreaDetectorIDs = arrayfun(@(i) sprintf("e2_%d", i), 0:17, "UniformOutput", false);

            % Instant loops
            obj.InstantLoopIDs = arrayfun(@(i) sprintf("e1i_%d", i), 0:7, "UniformOutput", false);

            % Traffic light IDs J1–J6
            obj.TrafficLightIDs = ["J1","J2","J3","J4","J5","J6"];
        end

        %% ============================================================
        %                           STEP
        %% ============================================================
        function [detectorStates, areaVehicleCount, areaMeanSpeed, pedestrianCount] = stepImpl(obj, ...
                trafficLight_J1, trafficLight_J2, trafficLight_J3, ...
                trafficLight_J4, trafficLight_J5, trafficLight_J6, clk)

            % Collect signals into vector
            trafficLightStates = [
                trafficLight_J1;
                trafficLight_J2;
                trafficLight_J3;
                trafficLight_J4;
                trafficLight_J5;
                trafficLight_J6
            ];

            % Send traffic light updates
            obj.sendTrafficLightCommands(trafficLightStates);

            % Read SUMO packets if available
            if obj.Client.NumBytesAvailable > 0
                raw = readline(obj.Client);

                try
                    data = jsondecode(raw);

                    if isfield(data, "detectors")
                        obj.LastDetectorStates = obj.parseDetectorStates(data.detectors);
                    end

                    if isfield(data, "instant_loops")
                        obj.LastPedestrianCount = obj.parseInstantLoopData(data.instant_loops);
                    end

                    if isfield(data, "area_detectors")
                        [obj.LastAreaVehicleCount, obj.LastAreaMeanSpeed] = ...
                            obj.parseAreaDetectorData(data.area_detectors);
                    end

                catch ME
                    warning("JSONError:decode", "JSON decode error: %s", ME.message);
                end
            end

            % Output the cached data
            detectorStates = obj.LastDetectorStates;
            areaVehicleCount = obj.LastAreaVehicleCount;
            areaMeanSpeed = obj.LastAreaMeanSpeed;
            pedestrianCount = obj.LastPedestrianCount;
        end

        %% ============================================================
        %              PARSING HELPERS
        %% ============================================================
        function pedestrianCount = parseInstantLoopData(obj, instantLoopData)
            pedestrianCount = zeros(obj.NumInstantLoops, 1);
            for i = 1:obj.NumInstantLoops
                id = obj.InstantLoopIDs{i};
                if isfield(instantLoopData, id)
                    pedestrianCount(i) = double(instantLoopData.(id));
                end
            end
        end

        function [vehicleCount, meanSpeed] = parseAreaDetectorData(obj, areaDetectorData)
            vehicleCount = zeros(obj.NumAreaDetectors, 1);
            meanSpeed = zeros(obj.NumAreaDetectors, 1);

            for i = 1:obj.NumAreaDetectors
                id = obj.AreaDetectorIDs{i};
                if isfield(areaDetectorData, id)
                    det = areaDetectorData.(id);

                    if isfield(det, 'vehicle_count')
                        vehicleCount(i) = double(det.vehicle_count);
                    end
                    if isfield(det, 'mean_speed')
                        meanSpeed(i) = double(det.mean_speed);
                    end
                end
            end
        end

        function detectorStates = parseDetectorStates(obj, detectorData)
            detectorStates = false(obj.NumDetectors, 1);
            for i = 1:obj.NumDetectors
                id = sprintf("e1_%d", i-1);
                if isfield(detectorData, id)
                    detectorStates(i) = logical(detectorData.(id) > 0.1);
                end
            end
        end

        %% ============================================================
        %               TRAFFIC LIGHT COMMAND HANDLING
        %% ============================================================
        function sendTrafficLightCommands(obj, trafficLightStates)
            for i = 1:obj.NumTrafficLights
                current = trafficLightStates(i);

                if obj.PreviousTrafficLightStates(i) ~= current
                    cmd = struct( ...
                        'type', 'trafficlight', ...
                        'junction', obj.TrafficLightIDs(i), ...
                        'phase', num2str(current) ...
                    );

                    obj.sendCommand(cmd);
                    obj.PreviousTrafficLightStates(i) = current;

                    fprintf("Sent: %s -> Phase %d\n", obj.TrafficLightIDs(i), current);
                end
            end
        end

        function sendCommand(obj, command)
            try
                writeline(obj.Client, jsonencode(command));
            catch ME
                warning("SendError", "Failed to send command: %s", ME.message);
            end
        end

        %% ============================================================
        %                       Housekeeping
        %% ============================================================
        function releaseImpl(obj)
            try
                clear obj.Client
            end
        end

        function resetImpl(obj)
            obj.PreviousTrafficLightStates = -1 * ones(obj.NumTrafficLights, 1);
            obj.LastPedestrianCount = zeros(obj.NumInstantLoops, 1);
        end

        %% ============================================================
        %           INPUT/OUTPUT DEFINITIONS
        %% ============================================================
        function num = getNumInputsImpl(~)
            num = 7; % 6 TL + clk
        end

        function num = getNumOutputsImpl(~)
            num = 4;
        end

        function [sz1, sz2, sz3, sz4, sz5, sz6, sz7] = getInputSizeImpl(~)
            sz1 = [1 1];
            sz2 = [1 1];
            sz3 = [1 1];
            sz4 = [1 1];
            sz5 = [1 1];
            sz6 = [1 1];
            sz7 = [1 1];
        end

        function [dt1, dt2, dt3, dt4, dt5, dt6, dt7] = getInputDataTypeImpl(~)
            dt1 = 'int32';
            dt2 = 'int32';
            dt3 = 'int32';
            dt4 = 'int32';
            dt5 = 'int32';
            dt6 = 'int32';
            dt7 = 'double';
        end

        function [c1, c2, c3, c4, c5, c6, c7] = isInputComplexImpl(~)
            c1 = false; c2 = false; c3 = false;
            c4 = false; c5 = false; c6 = false; c7 = false;
        end

        function [f1, f2, f3, f4, f5, f6, f7] = isInputFixedSizeImpl(~)
            f1 = true; f2 = true; f3 = true;
            f4 = true; f5 = true; f6 = true; f7 = true;
        end

        function [sz1, sz2, sz3, sz4] = getOutputSizeImpl(obj)
            sz1 = [obj.NumDetectors 1];
            sz2 = [obj.NumAreaDetectors 1];
            sz3 = [obj.NumAreaDetectors 1];
            sz4 = [obj.NumInstantLoops 1];
        end

        function [dt1, dt2, dt3, dt4] = getOutputDataTypeImpl(~)
            dt1 = 'boolean';
            dt2 = 'double';
            dt3 = 'double';
            dt4 = 'double';
        end

        function [c1, c2, c3, c4] = isOutputComplexImpl(~)
            c1 = false; c2 = false; c3 = false; c4 = false;
        end

        function [f1, f2, f3, f4] = isOutputFixedSizeImpl(~)
            f1 = true; f2 = true; f3 = true; f4 = true;
        end

        %% ============================================================
        %                  SAVE / LOAD
        %% ============================================================
        function s = saveObjectImpl(obj)
            s = saveObjectImpl@matlab.System(obj);
            s.PreviousTrafficLightStates = obj.PreviousTrafficLightStates;
            s.LastDetectorStates = obj.LastDetectorStates;
            s.LastPedestrianCount = obj.LastPedestrianCount;
            s.LastAreaVehicleCount = obj.LastAreaVehicleCount;
            s.LastAreaMeanSpeed = obj.LastAreaMeanSpeed;
            s.AreaDetectorIDs = obj.AreaDetectorIDs;
            s.InstantLoopIDs = obj.InstantLoopIDs;
            s.TrafficLightIDs = obj.TrafficLightIDs;
            s.ClientInitialized = ~isempty(obj.Client) && isvalid(obj.Client);
        end

        function loadObjectImpl(obj, s, wasLocked)
            loadObjectImpl@matlab.System(obj, s, wasLocked);

            obj.PreviousTrafficLightStates = s.PreviousTrafficLightStates;
            obj.LastDetectorStates = s.LastDetectorStates;
            obj.LastPedestrianCount = s.LastPedestrianCount;
            obj.LastAreaVehicleCount = s.LastAreaVehicleCount;
            obj.LastAreaMeanSpeed = s.LastAreaMeanSpeed;

            obj.AreaDetectorIDs = s.AreaDetectorIDs;
            obj.InstantLoopIDs = s.InstantLoopIDs;
            obj.TrafficLightIDs = s.TrafficLightIDs;

            if wasLocked && s.ClientInitialized
                obj.Client = tcpclient(obj.Host, obj.Port);
            end
        end
    end
end
