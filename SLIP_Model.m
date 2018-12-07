classdef SLIP_Model
    % SLIP_MODEL: a class of the simulation of the SLIP model itself
    %   This class is for being able to call a SLIP model simulation to run
    %   and gather data for use of the Ernst, Geyer, and Reinhard (EGB)
    %   controller paired with a Raibert controller
    
    properties
        Property1
    end
    
    methods
        function obj = SLIP_Model(inputArg1,inputArg2)
            %SLIP_MODEL Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

