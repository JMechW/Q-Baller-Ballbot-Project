classdef ObjSensor
    
    properties
        InFunc;
        OutFunc;

        Name='Name';
        Description='Description';

        State;
        StateMax;
        
        StateFunc;
        NoiseFunc;
        
        OtherInfo_;
        OtherFunc_;        
    end

    methods
        function obj=ObjSensor(DataSize,EleNum,varargin)
            obj.State=zeros(EleNum,DataSize);
            obj.StateMax=zeros(EleNum,1);             

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
    end
end