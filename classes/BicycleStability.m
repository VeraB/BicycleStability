classdef BicycleStability < ThreeD
    %
     properties% (Access = protected)
        f_length;
    end
    
    methods (Static)
        
        function [SM_t] = calculateStabilityMargin_t(btm_t,COM_0)
           SM_t = zeros(1,length(btm_t));
            parfor i = 1: length(btm_t)
                tm = btm_t{i};
                H_f_to_h = tm.getH_f_to_h();
                H_f_0 = btm_t{i}.getH;
                COM_h = (H_f_to_h \(H_f_0)) *  COM_0;
                SM_t(i) = COM_h(2);
            end
        end
        
        
    end
    
    methods
        function btm = BicycleStability(tm,f_length)
            % The COnstructor
            %
            quat = tm.getQ;
            btm@ThreeD(quat);
            btm.f_length = f_length;
        end
        function lengthAdams = getlength(btm)
            % this function can be used to calculate the length of the rear
            % frame sensor to the ground, when processing Adams data.
            % length = f_z0/sin(rollangle)
            currentRPY = btm.getRPY(false);
            roll_angle = currentRPY(1);
            %TO be done: get the z-coordinate of the origin of the rear
            %frame sensor from the Adams input data...
        end
            
            
        function [H_f_to_h] = getH_f_to_h (btm)
            % Calculate the H matrix from the rear frame sensor to the heading frame (projected coordinate system of the rear frame on the ground)
            % input are the length (origin of rear frame coordinate system to the ground) and the roll angle
            currentRPY = btm.getRPY(false);
            roll_angle = currentRPY(1);
%             if (sin(roll_angle) == 0)
%                 if roll_angle == pi 
%                     translation_f_to_h = [0,0,-btm.l];
%                 else 
%                     translation_f_to_h = [0,0,btm.l];
%                 end
%             elseif cos(roll_angle) == 0
%                 if roll_angle == pi/2
%                    translation_f_to_h = [0,btm.l,0];
%                 else
%                    translation_f_to_h = [0,-btm.l,0];
%                 end
%             else
            
            translation_f_to_h = [0, btm.f_length*sin(roll_angle), btm.f_length*cos(roll_angle)];
                      H_f_to_h = rotx(roll_angle);
            H_f_to_h(1:3,4)=translation_f_to_h';
        end
    end
    
end
    
    
    