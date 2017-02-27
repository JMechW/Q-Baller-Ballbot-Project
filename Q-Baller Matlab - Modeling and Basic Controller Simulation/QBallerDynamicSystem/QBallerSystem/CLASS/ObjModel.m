classdef ObjModel
    properties
        AttView;
        TrackView;
        
        InFunc;
        OutFunc;
        
        
        Name='Name';
        Description='Description';
        
        COG=zeros(3,1);%Center of Gravity
        OD=zeros(3,1);%Origin Deviation (Graphic Purpose Only)
        Attitude=zeros(3,1);
        Position=zeros(3,1);
        AttFunc;
        PosFunc;
        
        Force=zeros(3,1);
        Torque=zeros(3,1);
        ForceFunc;
        TorqueFunc;

        OtherInfo_;
        OtherFunc_;    
    end
    
    
    %Functions Defined
    methods
        function obj=ObjModel(DataSize,...Number of Data Storage Allowed
                              varargin)
                          
            obj.AttView=   struct('Model3D',cell(1,1),...
                                  'ModelSize',zeros(1,2),...
                                  'Color',zeros(1,3),...
                                  'Edge',zeros(1,1),...
                                  'Axes',cell(1,1),...
                                  'FigElement',cell(1,1));
                              
            obj.TrackView= struct('Axes',cell(1,1),...
                                  'Color',':k',...
                                  'Width',2,...
                                  'FigElement',cell(1,1));
                              
            obj.AttView.Model3D=cell(1);
            obj.AttView.Axes=cell(1);
            obj.AttView.FigElement=cell(1);    
            obj.TrackView.Axes=cell(1);
            obj.TrackView.FigElement=cell(1);
            
            obj.Attitude=zeros(3,DataSize);
            obj.Position=zeros(3,DataSize);
            obj.Force=zeros(3,DataSize);
            obj.Torque=zeros(3,DataSize);
            
            if rem(length(varargin),2)~=0
                error('Missing Input Component');
            else
               for ii=1:2:length(varargin)
                   switch varargin{ii}
                       case 'Name'          
                           obj.Name=varargin{ii+1};
                       case 'Description'   
                           obj.Description=varargin{ii+1};
                       case 'OD'            
                           obj.OD=varargin{ii+1};
                       case 'COG'           
                           obj.COG=varargin{ii+1};
                       otherwise
                           error('Wrong Component Name');
                   end
               end
            end
        end
        
        
        %%%%%
        function Output1=EulerAngle(obj,n,t,varargin)
            if nargin>3
                A=varargin{1}(1);
                B=varargin{1}(2);
                C=varargin{1}(3);
            else
                A=obj.Attitude(1,t);
                B=obj.Attitude(2,t);
                C=obj.Attitude(3,t);
            end
            switch n
                case 'G2L'
                    Output1=eul2rotm([C,B,A]);  %G2L
                    return;
                case 'L2G'  
                    Output1=eul2rotm([C,B,A])';  %L2G
                    return;                    
                case 'G2O'
                    Output1=eul2rotm([C,0,0]);  %G2O
                    return;                    
                case 'O2G'
                    Output1=eul2rotm([C,0,0])';  %O2G
                    return;                    
                case 'O2L'
                     Output1=eul2rotm([0,B,A]);  %O2L
                    return;                   
                case 'L2O'
                     Output1=eul2rotm([0,B,A])';  %L2O
                    return;                   
                case 'MJ'
                     Output1=[1 0 -sin(B); 0 cos(A) sin(A)*cos(B); 0 -sin(A) cos(B)*cos(A)];  %MJ
                    return;                   
                case 'MJInv'
                     Output1=inv([1 0 -sin(B); 0 cos(A) sin(A)*cos(B); 0 -sin(A) cos(B)*cos(A)]);  %MJInv
                    return;
            end
        end
        
        
        
        

        %%%%%
        function obj=VisualAttitude(obj,simplify0,samefig0)
            hold(obj.AttView.Axes,'on');
            GConv=gpuArray(obj.EulerAngle('G2L',1));
            
            if simplify0~=0
                for ii=1:length(obj.AttView.Model3D)
                    delete(obj.AttView.FigElement{ii});
                    GModel=obj.AttView.Model3D{ii};
                    GVisualM=GConv*GModel;
                    for jj=1:3
                        GVisualM(jj,:)=GVisualM(jj,:)+obj.OD(jj,1)+obj.Position(jj,1)*samefig0;
                    end
                    GX=reshape(GVisualM(1,:),obj.AttView.ModelSize(ii,1),[]);
                    GY=reshape(GVisualM(2,:),obj.AttView.ModelSize(ii,1),[]);
                    GZ=reshape(GVisualM(3,:),obj.AttView.ModelSize(ii,1),[]);
                    obj.AttView.FigElement{ii}=surf(obj.AttView.Axes,GX,GY,GZ,'FaceColor',obj.AttView.Color(ii,:), 'EdgeAlpha',obj.AttView.Edge(ii));
                end
            else
                    delete(obj.AttView.FigElement{1})
                    originsymbol=[obj.OD,obj.OD+2*obj.COG,obj.OD+2*obj.COG+[0.5*norm(obj.COG);0;0],obj.OD+2*obj.COG+[0.5*norm(obj.COG);0;0]+[-0.5*norm(obj.COG);0;+0.25*norm(obj.COG)]];
                    newOD=GConv*originsymbol;
                    for jj=1:3
                        newOD(jj,:)=newOD(jj,:)+obj.Position(jj,1)*samefig0;
                    end
                    obj.AttView.FigElement{1}=plot3(obj.AttView.Axes,newOD(1,:),newOD(2,:),newOD(3,:),'r','LineWidth',obj.TrackView.Width);
            end
            hold(obj.AttView.Axes,'off');
            drawnow;
        end

        function obj=VisualTrack(obj,dimension)
            hold(obj.TrackView.Axes,'on');
            delete(obj.TrackView.FigElement{1});
            TX=gpuArray(obj.Position(1,:));
            TY=gpuArray(obj.Position(2,:));
            if(dimension<3)
                obj.TrackView.FigElement{1}=plot(obj.TrackView.Axes,TX,TY,obj.TrackView.Color,'LineWidth',obj.TrackView.Width);
            else
                TZ=gpuArray(obj.Position(3,:));
                obj.TrackView.FigElement{1}=plot3(obj.TrackView.Axes,TX,TY,TZ,obj.TrackView.Color,'LineWidth',obj.TrackView.Width);
            end
            hold(obj.TrackView.Axes,'off');
            drawnow;
        end
        
        function obj=VisualAll(obj,simplify0,samefig0,dimension)
            obj=VisualAttitude(obj,simplify0,samefig0);
            obj=VisualTrack(obj,dimension);
        end
    end
end