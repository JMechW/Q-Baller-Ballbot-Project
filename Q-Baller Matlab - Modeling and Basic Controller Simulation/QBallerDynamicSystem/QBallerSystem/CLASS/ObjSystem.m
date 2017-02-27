classdef ObjSystem
    properties
        ODE;
        Simulator;
        Observer;
        Inputer;
        
        ObjectElement=cell(1);
        SensorElement=cell(1);
        
        Name='Name';
        Description='Description';
        
        Timer;
        TimerFunc;
        
        TrueState;
        ObserveState;
        Input;
        
        OtherInfo_;
        OtherFunc_=cell(1,1);    
    end
    
    methods
        function obj=ObjSystem(DataSize,StateNum,InputNum,varargin)
                        obj.TrueState=struct('Int',zeros(StateNum,DataSize),...
                             'Pos',zeros(StateNum,DataSize),...
                             'Vel',zeros(StateNum,DataSize),...
                             'Acc',zeros(StateNum,DataSize));
                        obj.ObserveState=struct('Int',zeros(StateNum,DataSize),...
                             'Pos',zeros(StateNum,DataSize),...
                             'Vel',zeros(StateNum,DataSize),...
                             'Acc',zeros(StateNum,DataSize));
                        obj.Input=zeros(InputNum,DataSize);   
                        obj.Timer=struct('Point',zeros(1,DataSize),...
                             'Step',zeros(1,DataSize),...
                             'Seg',5,...
                             'Cnt',0);
                         
            if rem(length(varargin),2)~=0
                error('Missing Input Component');
            else
               for ii=1:2:length(varargin)
                   switch varargin{ii}
                       case 'Name'          
                           obj.Name=varargin{ii+1};
                       case 'Description'   
                           obj.Description=varargin{ii+1};
                       otherwise
                           error('Wrong Component Name');
                   end
               end
            end
        end
        
        
        %%%%
        function obj=UpdateObjectForce(obj,varargin)
            if nargin>1
                exobj=varargin;
                for ii=1:length(obj.ObjectElement)
                    obj.ObjectElement{ii}.Force=obj.ObjectElement{ii}.ForceFunc(obj,exobj);
                end
            else
                for ii=1:length(obj.ObjectElement)
                    obj.ObjectElement{ii}.Force=obj.ObjectElement{ii}.ForceFunc(obj);
                end                
            end
        end
        
        
        
        function obj=UpdateObjectTorque(obj,varargin)
            if nargin>1
                exobj=varargin;
                for ii=1:length(obj.ObjectElement)
                    obj.ObjectElement{ii}.Torque=obj.ObjectElement{ii}.TorqueFunc(obj,exobj);
                end
            else
                for ii=1:length(obj.ObjectElement)
                    obj.ObjectElement{ii}.Torque=obj.ObjectElement{ii}.TorqueFunc(obj);
                end                
            end
        end
        
        
        function obj=UpdateObjectAttitude(obj,varargin)
            if nargin>1
                exobj=varargin;
                for ii=1:length(obj.ObjectElement)
                    obj.ObjectElement{ii}.Attitude=obj.ObjectElement{ii}.AttFunc(obj,exobj);
                end
            else
                for ii=1:length(obj.ObjectElement)
                    obj.ObjectElement{ii}.Attitude=obj.ObjectElement{ii}.AttFunc(obj);
                end                
            end
        end
        
        function obj=UpdateObjectPosition(obj,varargin)
            if nargin>1
                exobj=varargin;
                for ii=1:length(obj.ObjectElement)
                    obj.ObjectElement{ii}.Position=obj.ObjectElement{ii}.PosFunc(obj,exobj);
                end
            else
                for ii=1:length(obj.ObjectElement)
                    obj.ObjectElement{ii}.Position=obj.ObjectElement{ii}.PosFunc(obj);
                end                
            end
        end
        
        function obj=UpdateSensorState(obj,varargin)
            if nargin>1
                exobj=varargin;
                for ii=1:length(obj.SensorElement)
                    obj.SensorElement{ii}.State=obj.SensorElement{ii}.StateFunc(obj,exobj);
                end
            else
                for ii=1:length(obj.SensorElement)
                    obj.SensorElement{ii}.State=obj.SensorElement{ii}.StateFunc(obj);
                end                
            end
        end
        
        function obj=UpdateSensorNoise(obj,varargin)
            if nargin>1
                exobj=varargin;
                for ii=1:length(obj.SensorElement)
                    obj.SensorElement{ii}.State=obj.SensorElement{ii}.NoiseFunc(obj,exobj);
                end
            else
                for ii=1:length(obj.SensorElement)
                    obj.SensorElement{ii}.State=obj.SensorElement{ii}.NoiseFunc(obj);
                end                
            end
        end
        
        %%%%
        function obj=SimulationUpdate(obj,varargin)
            if nargin>1
                exobj=varargin;
                    obj.TrueState=obj.Simulator(obj,exobj);
            else
                    obj.TrueState=obj.Simulator(obj);
            end
        end
        
        function obj=ObserverUpdate(obj,varargin)
            if nargin>1
                exobj=varargin;
                for ii=1:length(obj.SensorElement)
                    obj.ObserveState=obj.Observer(obj,exobj);
                end
            else
                for ii=1:length(obj.SensorElement)
                    obj.ObserveState=obj.Observer(obj);
                end                
            end
        end
        
        function obj=InputUpdate(obj,varargin)
            if nargin>1
                exobj=varargin;
                    obj.Input=obj.Inputer(obj,exobj);
            else
                    obj.Input=obj.Inputer(obj);
            end
        end
               
        
        %%%%
        function obj=UpdateTimer(obj,NewTime)
            obj.Timer.Point=[NewTime,obj.Timer.Point(1:end-1)];
            obj.Timer.Step=[NewTime-obj.Timer.Point(2),obj.Timer.Step(1:end-1)];
            obj.Timer.Cnt=obj.Timer.Cnt+1;
        end
    end
end