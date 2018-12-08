
function delta
%%some global parameters

theta_constr = [[-80 120];[-80 120];[-80 120]];
scale = 0.8;
scale_b = 1;
scale_p = 4;
sb = 567*scale*scale_b;
sp = 76*scale*scale_p;
L = 524*scale;
l = 1244*scale;
h = 131*scale;
wb = 164*scale*scale_b;
ub = 327*scale*scale_b;
wp = 22*scale*scale_p;
up = 44*scale*scale_p;
psi = 2*sb/h;
%% two platforms positions
b1 = [sb/2, -wb, 0, 1];
b2 = [0, ub, 0, 1];
b3 = [-sb/2, -wb, 0, 1];
b0 = [b1;b2; b3]';

p1 = [0 -up 0 1];
p2 = [sp/2 wp 0 1];
p3 = [-sp/2 wp 0 1];
p0 = [p1; p2; p3]';

theta_home = [10 10 10];


%%home initiation
InitHome
%%

%
% Create the push buttons: pos is: [left bottom width height]
demo = uicontrol(fig_1,'String','Demo','callback',@demo_button_press,...
    'Position',[20 5 60 20]);

rnd_demo = uicontrol(fig_1,'String','Random Move','callback',@rnd_demo_button_press,...
    'Position',[100 5 80 20]);

clr_trail = uicontrol(fig_1,'String','Clr Trail','callback',@clr_trail_button_press,...
    'Position',[            200 5 60 20      ]);
%
home = uicontrol(fig_1,'String','Home','callback',@home_button_press,...
    'Position',[            280 5 70 20      ]);
%
% Kinematics Panel
%
K_p = uipanel(fig_1,...
    'units','pixels',...
    'Position',[            20 45 265 120      ],...
    'Title','Kinematics','FontSize',11);


%%



LD = 110; % Left, used to set the GUI.
HT = 18;  % Height
BT = 156-80; % Bottom

%%  GUI buttons for Theta 1.  pos is: [left bottom width height]
t1_slider = uicontrol(K_p,'style','slider',...
    'Max',theta_constr(1,2),'Min',theta_constr(1,1),'Value',0,...
    'SliderStep',[0.05 0.2],...
    'callback',@t1_slider_button_press,...
    'Position',[LD BT 120 HT]);
t1_min = uicontrol(K_p,'style','text',...
    'String',int2str(theta_constr(1,1)),...
    'Position',[LD-30 BT+1 25 HT-4]); % L, from bottom, W, H
t1_max = uicontrol(K_p,'style','text',...
    'String',strcat('+',int2str(theta_constr(1,2))),...
    'Position',[LD+125 BT+1 30 HT-4]); % L, B, W, H
t1_text = uibutton(K_p,'style','text',...  % Nice program Doug. Need this
    'String','\theta_1 (r)',...                % due to no TeX in uicontrols. 
    'Position',[LD-100 BT 20 HT]); % L, B, W, H
% t1_text = uicontrol(K_p,'style','text',... % when matlab fixes uicontrol
%     'String','t1',...                      % for TeX, then I can use this.  
%     'Position',[LD-100 BT 20 HT]); % L, B, W, H
t1_edit = uicontrol(K_p,'style','edit',...
    'String',0,...
    'callback',@t1_edit_button_press,...
    'Position',[LD-75 BT 30 HT]); % L, B, W, H
%
%%  GUI buttons for Theta 2.
BT = 126-80;   % Bottom
t2_slider = uicontrol(K_p,'style','slider',...
    'Max',theta_constr(2,2),'Min',theta_constr(2,1),'Value',0,...        % Mech. stop limits !
    'SliderStep',[0.05 0.2],...
    'callback',@t2_slider_button_press,...
    'Position',[LD BT 120 HT]);
t2_min = uicontrol(K_p,'style','text',...
    'String',int2str(theta_constr(2,1)),...
    'Position',[LD-30 BT+1 25 HT-4]); % L, from bottom, W, H
t2_max = uicontrol(K_p,'style','text',...
    'String',strcat('+',int2str(theta_constr(2,2))),...
    'Position',[LD+125 BT+1 30 HT-4]); % L, B, W, H
t2_text = uibutton(K_p,'style','text',...
    'String','\theta_2 (g)',...
    'Position',[LD-100 BT 20 HT]); % L, B, W, H
t2_edit = uicontrol(K_p,'style','edit',...
    'String',0,...
    'callback',@t2_edit_button_press,...
    'Position',[LD-75 BT 30 HT]); % L, B, W, H
%
%%  GUI buttons for Theta 3.
BT = 96-80;   % Bottom
t3_slider = uicontrol(K_p,'style','slider',...
    'Max',theta_constr(3,2),'Min',theta_constr(3,1),'Value',0,...
    'SliderStep',[0.05 0.2],...
    'callback',@t3_slider_button_press,...
    'Position',[LD BT 120 HT]);
t3_min = uicontrol(K_p,'style','text',...
    'String',int2str(theta_constr(3,1)),...
    'Position',[LD-30 BT+1 25 HT-4]); % L, from bottom, W, H
t3_max = uicontrol(K_p,'style','text',...
    'String',strcat('+',int2str(theta_constr(3,2))),...
    'Position',[LD+125 BT+1 30 HT-4]); % L, B, W, H
t3_text = uibutton(K_p,'style','text',...
    'String','\theta_3 (b)',...
    'Position',[LD-100 BT 20 HT]); % L, B, W, H
t3_edit = uicontrol(K_p,'style','edit',...
    'String',0,...
    'callback',@t3_edit_button_press,...
    'Position',[LD-75 BT 30 HT]); % L, B, W, H

%% Slider for Theta 1 motion.
%
    function t1_slider_button_press(h,dummy)
        slider_value = round(get(h,'Value'));
        set(t1_edit,'string',slider_value);
        T_Old = getappdata(0,'ThetaOld');
        t2old = T_Old(2); t3old = T_Old(3); 
        deltaANI([slider_value+theta_home(1),t2old,t3old],10,'n')
    end
%
%% Slider for Theta 2 motion.
%
    function t2_slider_button_press(h,dummy)
        slider_value = round(get(h,'Value'));
        set(t2_edit,'string',slider_value);
        T_Old = getappdata(0,'ThetaOld');
        t1old = T_Old(1); t3old = T_Old(3);
        deltaANI([t1old,slider_value+theta_home(2),t3old],10,'n')
    end
%
%% Slider for Theta 3 motion.
    function t3_slider_button_press(h,dummy)
        slider_value = round(get(h,'Value'));
        set(t3_edit,'string',slider_value);
        T_Old = getappdata(0,'ThetaOld');
        t1old = T_Old(1); t2old = T_Old(2);
        deltaANI([t1old,t2old,slider_value+theta_home(3)],10,'n')
    end

%% Edit box for Theta 1 motion.
%
    function t1_edit_button_press(h,dummy)
        user_entry = check_edit(h,theta_constr(1,1),theta_constr(1,2),0,t1_edit);
        set(t1_slider,'Value',user_entry);  % slider = text box.
        T_Old = getappdata(0,'ThetaOld');   % Current pose    
        %
        t2old = T_Old(2); t3old = T_Old(3);
        %
        deltaANI([user_entry+theta_home(1),t2old,t3old],10,'n')
    end
%
%% Edit box for Theta 2 motion.
%
    function t2_edit_button_press(h,dummy)
        user_entry = check_edit(h,theta_constr(2,1),theta_constr(2,2),0,t2_edit);
        set(t2_slider,'Value',user_entry);  % slider = text box.
        T_Old = getappdata(0,'ThetaOld');   % Current pose    
        %
        t1old = T_Old(1); t3old = T_Old(3);
        %
        deltaANI([t1old,user_entry+theta_home(2),t3old],10,'n')
    end
%% Edit box for Theta 3 motion.
%
    function t3_edit_button_press(h,dummy)
        user_entry = check_edit(h,theta_constr(2,1),theta_constr(3,2),0,t3_edit);
        set(t3_slider,'Value',user_entry);  % slider = text box.
        T_Old = getappdata(0,'ThetaOld');   % Current pose    
        %
        t1old = T_Old(1); t2old = T_Old(2);
        %
        deltaANI([t1old,t2old,user_entry+theta_home(3)],10,'n')
    end
%%
    function user_entry = check_edit(h,min_v,max_v,default,h_edit)
        % This function will check the value typed in the text input box
        % against min and max values, and correct errors.
        %
        % h: handle of gui
        % min_v min value to check
        % max_v max value to check
        % default is the default value if user enters non number
        % h_edit is the edit value to update.
        %
        user_entry = str2double(get(h,'string'));
        if isnan(user_entry)
            errordlg(['You must enter a numeric value, defaulting to ',num2str(default),'.'],'Bad Input','modal')
            set(h_edit,'string',default);
            user_entry = default;
        end
        %
        if user_entry < min_v
            errordlg(['Minimum limit is ',num2str(min_v),' degrees, using ',num2str(min_v),'.'],'Bad Input','modal')
            user_entry = min_v;
            set(h_edit,'string',user_entry);
        end
        if user_entry > max_v
            errordlg(['Maximum limit is ',num2str(max_v),' degrees, using ',num2str(max_v),'.'],'Bad Input','modal')
            user_entry = max_v;
            set(h_edit,'string',user_entry);
        end
    end
%
%% Demo button's callback
    function demo_button_press(h,dummy)
        %
        % disp('pushed demo bottom');
        %         R = 500;
        %         x = 1000;
        n = 2;    % demo ani steps
        num = 30; % home to start, and end to home ani steps
        %         j = 1;
        %         M = 1000;
        for t = 0:.1:5*pi
            Px = 30*t*cos(t);
            Pz = -1200+300*t*(t)/(50*pi);
            Py = 30*t*sin(t);
            theta = deltaIK([Px,Py,Pz]);
            if t==0 %move to start of demo
                deltaANI(theta,num,'n')
            end
            % Theta 4, 5 & 6 are zero due to plotting at wrist origen.
            deltaANI(theta,n,'y')
            set(t1_edit,'string',round(theta(1))); % Update slider and text.
            set(t1_slider,'Value',round(theta(1)));
            set(t2_edit,'string',round(theta(2)));
            set(t2_slider,'Value',round(theta(2)));
            set(t3_edit,'string',round(theta(3)));
            set(t3_slider,'Value',round(theta(3)));
        end
        gohome
        pause(1.5);
        clr_trail_4me;

        
         for t = 0:.05:2*pi
            a = 200;
            Py = -2*a*(1-cos(t))*cos(t);
            Pz = -900;
            Px = 2*a*(1-cos(t))*sin(t);
            theta = deltaIK([Px,Py,Pz]);
            if t==0 %move to start of demo
                deltaANI(theta,num,'n')
            end
            % Theta 4, 5 & 6 are zero due to plotting at wrist origen.
            deltaANI(theta,n,'y')
            set(t1_edit,'string',round(theta(1))); % Update slider and text.
            set(t1_slider,'Value',round(theta(1)));
            set(t2_edit,'string',round(theta(2)));
            set(t2_slider,'Value',round(theta(2)));
            set(t3_edit,'string',round(theta(3)));
            set(t3_slider,'Value',round(theta(3)));
         end
        deltaANI([-30 -30 -30],20,'n')
        pause(1.5)
        clr_trail_4me;
        gohome
    end
%
%
%%
    function home_button_press(h,dummy)
        %disp('pushed home bottom');
        gohome
    end
%
%%
function clr_trail_4me()
        %disp('pushed clear trail bottom');
        handles = getappdata(0,'patch_h');           %
        Tr = handles(8);
        %
        setappdata(0,'xtrail',0); % used for trail tracking.
        setappdata(0,'ytrail',0); % used for trail tracking.
        setappdata(0,'ztrail',0); % used for trail tracking.
        %
        set(Tr,'xdata',0,'ydata',0,'zdata',0);
end

    function clr_trail_button_press(h,dummy)
        %disp('pushed clear trail bottom');
        handles = getappdata(0,'patch_h');           %
        Tr = handles(8);
        %
        setappdata(0,'xtrail',0); % used for trail tracking.
        setappdata(0,'ytrail',0); % used for trail tracking.
        setappdata(0,'ztrail',0); % used for trail tracking.
        %
        set(Tr,'xdata',0,'ydata',0,'zdata',0);
    end
%
%
    function rnd_demo_button_press(h, dummy)
        %disp('pushed random demo bottom');
        % a = 10; b = 50; x = a + (b-a) * rand(5)
        
        theta_constr2 = [[0 90];[0 90];[0 90]];
        
        theta1 = theta_constr2(1,1)+ (theta_constr2(1,2)-theta_constr2(1,1))*rand(1); % offset for home
        theta2 = theta_constr2(2,1)+ (theta_constr2(2,2)-theta_constr2(2,1))*rand(1); % in the UP pos.
        theta3 = theta_constr2(3,1)+ (theta_constr2(3,2)-theta_constr2(3,1))*rand(1);
        %PumaPOS(theta1,theta2,theta3,theta4,theta5,theta6)
        n = 50;
        deltaANI([theta1+theta_home(1),theta2+theta_home(2),theta3+theta_home(3)],n,'y')
        set(t1_edit,'string',round(theta1)); % Update slider and text.
        set(t1_slider,'Value',round(theta1));
        set(t2_edit,'string',round(theta2));
        set(t2_slider,'Value',round(theta2));
        set(t3_edit,'string',round(theta3));
        set(t3_slider,'Value',round(theta3));
    end
        


%%
%Here are the functions used for this robot example:
%
%%
% When called this function will simply initialize a plot of the Puma 762
% robot by plotting it in it's home orientation and setting the current
% angles accordingly.
    function gohome()
        deltaANI([0,0,0],20,'n') % show it animate home
        %deltaANI(0,0,0,0,0,0,'n')  %drive it home, no animate.
        set(t1_edit,'string',0);
        set(t1_slider,'Value',0);  %At the home position, so all
        set(t2_edit,'string',0);   %sliders and input boxes = 0. 
        set(t2_slider,'Value',0);
        set(t3_edit,'string',0);
        set(t3_slider,'Value',0);
        setappdata(0,'ThetaOld',[0,0,0]);
        %setappdata(0,'ThetaOld',[0,0,0,0,0,0]);
    end
%

%%
function theta = deltaIK(p) 
    %% general variables
    %%for calculations
    a = wb - up;
    b = sp/2-sqrt(3)/2*wb;
    c = wp-wb/2;
    x = p(1);
    y = p(2);
    z = p(3);
    %%for easy calculations2
    e = zeros(1,3);
    f= zeros(1,3);
    g= zeros(1,3);
    e(1) = 2*L*(y+a);
    f(1)=2*z*L;
    g(1) = x^2+y^2+z^2+a^2+L^2+2*y*a-l^2;
    
    e(2) = -L*(sqrt(3)*(x+b)+y+c);
    f(2) = 2*z*L;
    g(2) = x^2+y^2+z^2+b^2+c^2+L^2+2*(x*b+y*c)-l^2;
    
    e(3) = L*(sqrt(3)*(x-b)-y-c);
    f(3) = 2*z*L;
    g(3) = x^2+y^2+z^2+b^2+c^2+L^2+2*(-x*b+y*c)-l^2;
    %%
    tm = zeros(1,3);
    tp = zeros(1,3);
    
    for i=1:3
        tm(i) =360*atan( (-f(i)-sqrt(e(i)^2+f(i)^2-g(i)^2))/(g(i)-e(i)))/pi;
        tp(i) =360*(atan (-f(i)+sqrt(e(i)^2+f(i)^2-g(i)^2))/(g(i)-e(i)))/pi;
    end

    theta =tm;
end
%% 
    function deltaANI(theta,n,trail,p)
        % This function will animate the Puma 762 robot given joint angles.
        % n is number of steps for the animation
        % trail is 'y' or 'n' (n = anything else) for leaving a trail.
        %
        ThetaOld = getappdata(0,'ThetaOld');
        %
        theta1old = ThetaOld(1);
        theta2old = ThetaOld(2);
        theta3old = ThetaOld(3);
        %
        t1 = linspace(theta1old,theta(1),n); 
        t2 = linspace(theta2old,theta(2),n); 
        t3 = linspace(theta3old,theta(3),n);% -180;  
        
        [a1, a2, a3] = get_knees_pos(theta);
        if nargin <=3 
             p = get_base(theta);
        end
            
        get_Jacobian(p,theta);
        Tb = makehgtform('translate',p);
        p_plat = Tb*p0;
        pos_analysis(a1,a2,a3,p_plat);
        
        
        n = length(t1);
        for i = 2:1:n
            % Forward Kinematics
            
            theta_temp = [t1(i),t2(i),t3(i)];
            %singularity_recognition(S,theta,1e-8);
            
            handles = getappdata(0,'patch_h');           %
            pa1 = handles(1);
            pa2 = handles(2);
            pa3 = handles(3);
            pl1 = handles(4);
            pl2 = handles(5);
            pl3 = handles(6);
            pp =  handles(7);
            Tr = handles(8);
            %
            
            [a1, a2, a3] = get_knees_pos(theta_temp);
            if nargin <=3 
                p = get_base(theta_temp);
            end
            
            get_Jacobian(p,theta_temp);
            Tb = makehgtform('translate',p);
            p_plat = Tb*p0;
            
            %pos_analysis(a1,a2,a3,p_plat);
        
            [ab1, ab2, ab3] = get_upper_knees_patch_positions(a1,a2,a3);
            [l1, l2, l3] = get_low_knees_patch_positions(a1,a2,a3,p_plat);
            set(pa1,'XData',ab1(1,:),'YData',ab1(2,:),'ZData',ab1(3,:));
            set(pa2,'XData',ab2(1,:),'YData',ab2(2,:),'ZData',ab2(3,:));
            set(pa3,'XData',ab3(1,:),'YData',ab3(2,:),'ZData',ab3(3,:));
            set(pl1,'XData',l1(1,:),'YData',l1(2,:),'ZData',l1(3,:));
            set(pl2,'XData',l2(1,:),'YData',l2(2,:),'ZData',l2(3,:));
            set(pl3,'XData',l3(1,:),'YData',l3(2,:),'ZData',l3(3,:));
            set(pp,'XData',p_plat(1,:),'YData',p_plat(2,:),'ZData',p_plat(3,:));
            
            % store trail in appdata 
            if trail == 'y'
                x_trail = getappdata(0,'xtrail');
                y_trail = getappdata(0,'ytrail');
                z_trail = getappdata(0,'ztrail');
                %
                xdata = [x_trail p(1)];
                ydata = [y_trail p(2)];
                zdata = [z_trail p(3)];
                %
                setappdata(0,'xtrail',xdata); % used for trail tracking.
                setappdata(0,'ytrail',ydata); % used for trail tracking.
                setappdata(0,'ztrail',zdata); % used for trail tracking.
                %
                set(Tr,'xdata',xdata,'ydata',ydata,'zdata',zdata);
            end
            drawnow
        end
        setappdata(0,'ThetaOld',theta);
    end
%%  constrains analysis (to understand does position satisfy constrains)
    function flag = pos_analysis(a1,a2,a3,p_plat)
            l_act = 995.2/0.8*scale;
            Eps = 3;
            if abs(norm(a1'-p_plat(:,1))-l_act)>Eps || abs(norm(a2'-p_plat(:,2))-l_act)>Eps || abs(norm(a3'-p_plat(:,3))-l_act)>Eps
                h = errordlg('Point unreachable due to joint angle constraints.','JOINT ERROR');
                flag = true(1);
            else
                flag = false(1);
            end
    end


    function InitHome
        % Use forward kinematics to place the robot in a specified
        % configuration.
        % Figure setup data, create a new figure for the GUI
        set(0,'Units','pixels')
        dim = get(0,'ScreenSize');
        fig_1 = figure('doublebuffer','on','Position',[0,35,dim(3)-200,dim(4)-110],...
            'MenuBar','none','Name',' 3D Puma Robot Graphical Demo',...
            'NumberTitle','off','CloseRequestFcn',@del_app);
        hold on;
        %light('Position',[-1 0 0]);
        light                               % add a default light
        daspect([1 1 1])                    % Setting the aspect ratio
        view(135,25)
        xlabel('X'),ylabel('Y'),zlabel('Z');
        title('ABB FlexPicker Delta Robot (Modified)');
        axis_limits = [-1500 1500 -1500 1500 -1500 500];
        axis(axis_limits);
        plot3([-1500,1500],[-1500,-1500],[-1500,-1500],'k')
        plot3([-1500,-1500],[-1500,1500],[-1500,-1500],'k')
        plot3([-1500,-1500],[-1500,-1500],[-1500,1500],'k')
        plot3([-1500,-1500],[1500,1500],[-1500,1500],'k')
        plot3([-1500,1500],[-1500,-1500],[1500,1500],'k')
        plot3([-1500,-1500],[-1500,1500],[1500,1500],'k')

        %%
        theta = theta_home;
        [a1, a2, a3] = get_knees_pos(theta);
        p = get_base(theta);
        Tb = makehgtform('translate',p);
        p_plat = Tb*p0;
        
        [ab1, ab2, ab3] = get_upper_knees_patch_positions(a1,a2,a3);
        pa1 = get_patch(ab1,'o','r','r');
        pa2 = get_patch(ab2,'o','g','g');
        pa3 = get_patch(ab3,'o','b','b');
        
        [l1, l2, l3] = get_low_knees_patch_positions(a1,a2,a3,p_plat);
        pl1 = get_patch(l1,'o','r','r');
        pl2 = get_patch(l2,'o','g','g');
        pl3 = get_patch(l3,'o','b','b');
        
        pb = get_patch(b0,'o','k','k');
        pp = get_patch(p_plat,'o','k','k');
        %%
        Tr = plot3(0,0,0,'m.'); % holder for trail paths
        %
        setappdata(0,'patch_h',[pa1, pa2, pa3,pl1, pl2, pl3,pp,Tr])
        %
        setappdata(0,'xtrail',0); % used for trail tracking.
        setappdata(0,'ytrail',0); % used for trail tracking.
        setappdata(0,'ztrail',0); % used for trail tracking.
        %
        setappdata(0,'ThetaOld',theta);
        %
    end








%%
    function del_app(varargin)
        %This is the main figure window close function, to remove any
        % app data that may be left due to using it for geometry.
        rmappdata(0,'ThetaOld');
        rmappdata(0,'patch_h');
        rmappdata(0,'xtrail');
        rmappdata(0,'ytrail');
        rmappdata(0,'ztrail');
        delete(fig_1);
    end
        
%%FUNCTION NOT CHANGED
    function [hout,ax_out] = uibutton(varargin)
        %uibutton: Create pushbutton with more flexible labeling than uicontrol.
        % Usage:
        %   uibutton accepts all the same arguments as uicontrol except for the
        %   following property changes:
        %
        %     Property      Values
        %     -----------   ------------------------------------------------------
        %     Style         'pushbutton', 'togglebutton' or 'text', default =
        %                   'pushbutton'.
        %     String        Same as for text() including cell array of strings and
        %                   TeX or LaTeX interpretation.
        %     Interpreter   'tex', 'latex' or 'none', default = default for text()
        %
        % Syntax:
        %   handle = uibutton('PropertyName',PropertyValue,...)
        %   handle = uibutton(parent,'PropertyName',PropertyValue,...)
        %   [text_obj,axes_handle] = uibutton('Style','text',...
        %       'PropertyName',PropertyValue,...)
        %
        % uibutton creates a temporary axes and text object containing the text to
        % be displayed, captures the axes as an image, deletes the axes and then
        % displays the image on the uicontrol.  The handle to the uicontrol is
        % returned.  If you pass in a handle to an existing uicontol as the first
        % argument then uibutton will use that uicontrol and not create a new one.
        %
        % If the Style is set to 'text' then the axes object is not deleted and the
        % text object handle is returned (as well as the handle to the axes in a
        % second output argument).
        %
        % See also UICONTROL.

        % Version: 1.6, 20 April 2006
        % Author:  Douglas M. Schwarz
        % Email:   dmschwarz=ieee*org, dmschwarz=urgrad*rochester*edu
        % Real_email = regexprep(Email,{'=','*'},{'@','.'})


        % Detect if first argument is a uicontrol handle.
        keep_handle = false;
        if nargin > 0
            h = varargin{1};
            if isscalar(h) && ishandle(h) && strcmp(get(h,'Type'),'uicontrol')
                keep_handle = true;
                varargin(1) = [];
            end
        end

        % Parse arguments looking for 'Interpreter' property.  If found, note its
        % value and then remove it from where it was found.
        interp_value = get(0,'DefaultTextInterpreter');
        arg = 1;
        remove = [];
        while arg <= length(varargin)
            v = varargin{arg};
            if isstruct(v)
                fn = fieldnames(v);
                for i = 1:length(fn)
                    if strncmpi(fn{i},'interpreter',length(fn{i}))
                        interp_value = v.(fn{i});
                        v = rmfield(v,fn{i});
                    end
                end
                varargin{arg} = v;
                arg = arg + 1;
            elseif ischar(v)
                if strncmpi(v,'interpreter',length(v))
                    interp_value = varargin{arg+1};
                    remove = [remove,arg,arg+1];
                end
                arg = arg + 2;
            elseif arg == 1 && isscalar(v) && ishandle(v) && ...
                    any(strcmp(get(h,'Type'),{'figure','uipanel'}))
                arg = arg + 1;
            else
                error('Invalid property or uicontrol parent.')
            end
        end
        varargin(remove) = [];

        % Create uicontrol, get its properties then hide it.
        if keep_handle
            set(h,varargin{:})
        else
            h = uicontrol(varargin{:});
        end
        s = get(h);
        if ~any(strcmp(s.Style,{'pushbutton','togglebutton','text'}))
            delete(h)
            error('''Style'' must be pushbutton, togglebutton or text.')
        end
        set(h,'Visible','off')

        % Create axes.
        parent = get(h,'Parent');
        ax = axes('Parent',parent,...
            'Units',s.Units,...
            'Position',s.Position,...
            'XTick',[],'YTick',[],...
            'XColor',s.BackgroundColor,...
            'YColor',s.BackgroundColor,...
            'Box','on',...
            'Color',s.BackgroundColor);
        % Adjust size of axes for best appearance.
        set(ax,'Units','pixels')
        pos = round(get(ax,'Position'));
        if strcmp(s.Style,'text')
            set(ax,'Position',pos + [0 1 -1 -1])
        else
            set(ax,'Position',pos + [4 4 -8 -8])
        end
        switch s.HorizontalAlignment
            case 'left'
                x = 0.0;
            case 'center'
                x = 0.5;
            case 'right'
                x = 1;
        end
        % Create text object.
        text_obj = text('Parent',ax,...
            'Position',[x,0.5],...
            'String',s.String,...
            'Interpreter',interp_value,...
            'HorizontalAlignment',s.HorizontalAlignment,...
            'VerticalAlignment','middle',...
            'FontName',s.FontName,...
            'FontSize',s.FontSize,...
            'FontAngle',s.FontAngle,...
            'FontWeight',s.FontWeight,...
            'Color',s.ForegroundColor);

        % If we are creating something that looks like a text uicontrol then we're
        % all done and we return the text object and axes handles rather than a
        % uicontrol handle.
        if strcmp(s.Style,'text')
            delete(h)
            if nargout
                hout = text_obj;
                ax_out = ax;
            end
            return
        end

        % Capture image of axes and then delete the axes.
        frame = getframe(ax);
        delete(ax)

        % Build RGB image, set background pixels to NaN and put it in 'CData' for
        % the uicontrol.
        if isempty(frame.colormap)
            rgb = frame.cdata;
        else
            rgb = reshape(frame.colormap(frame.cdata,:),[pos([4,3]),3]);
        end
        size_rgb = size(rgb);
        rgb = double(rgb)/255;
        back = repmat(permute(s.BackgroundColor,[1 3 2]),size_rgb(1:2));
        isback = all(rgb == back,3);
        rgb(repmat(isback,[1 1 3])) = NaN;
        set(h,'CData',rgb,'String','','Visible',s.Visible)

        if nargout
            hout = h;
        end
    end
    % Some functions that I use
    

function p = spheres_intersect(c1,c2,c3)
   p = zeros(1,3); 
   Eps = 1e-1;
   if abs(c1(3)-c3(3))<Eps || abs(c3(3)-c2(3))<Eps  
       %a = 2*(c3(1)-c1(1));
       %b = 2*(c3(2)-c1(2));
       %c = c3(1)^2+c3(2)^2-c1(1)^2-c1(2)^2;
       %d = 2*(c3(1)-c2(1));
       %e = 2*(c3(1)-c2(1));
       %f = c3(1)^2+c3(2)^2-c2(1)^2-c2(2)^2;
       
       %p(1) = (c*e - b*f)/(a*e - b*d);
       %p(2) = (a*f -c*d)/(a*e - b*d);
       
       %B = -2*c1(3);
       %C = c1(3)^2 - l^2 + (p(1)-c1(1))^2+(p(2)-c1(2))^2;
       
       %p(3) = (-B-sqrt(B^2-4*C))/2;
       temp = interx(c1,c2,c3,l,l,l);
       p=temp(1:3);
   else
       a11 = 2*(c3(1)-c1(1));
       a12 = 2*(c3(2)-c1(2));
       a13 = 2*(c3(3)-c1(3));
   
       a21 = 2*(c3(1)-c2(1));
       a22 = 2*(c3(2)-c2(2));
       a23 = 2*(c3(3)-c2(3));
   
       b1 = c3(1)^2+c3(2)^2+c3(3)^2-c1(1)^2-c1(2)^2-c1(3)^2;
       b2 = c3(1)^2+c3(2)^2+c3(3)^2-c2(1)^2-c2(2)^2-c2(3)^2;
   
       a1 = a11/a13-a21/a23;
       a2 = a12/a13-a22/a23;
       a3 = b2/a23-b1/a13;
       a4 = -a2/a1;
       a5 = -a3/a1;
   
       a6 = (-a21*a4-a22)/a23;
       a7 = (b2-a21*a5)/a23;
   
       a = (a4^2+1+a6^2);
       b = 2*a4*(a5-c1(1))-2*c1(2)+2*a6*(a7-c1(3));
       c = a5*(a5-2*c1(1))+a7*(a7-2*c1(3))+c1(1)^2+c1(2)^2+c1(3)^2-l^2;
    
       p(2) = (-b-sqrt(b^2-4*a*c))/2/a;
       p(1) = a4*p(2)+a5;
       p(3) = a6*p(2)+a7;
   
       if p(3) > 0
            p(2) = (-b+sqrt(b^2-4*a*c))/2/a;
            p(1) = a4*p(2)+a5;
            p(3) = a6*p(2)+a7;
       end
   end
   
end
    
function result=interx(X1,X2,X3,r1,r2,r3,pos)
    if(nargin<7), pos=0; end % default value
    x1=X1(1); y1=X1(2); z1=X1(3);
    x2=X2(1); y2=X2(2); z2=X2(3);
    x3=X3(1); y3=X3(2); z3=X3(3);
    % convert in coord sys at [x1 y1 z1] oriented same as global
    % x2=1; y2=1; z2=0.2; x3=2; y3=1; z3=1; r1=2.5; r2=2.6; r3=2.7; % TEST
    x2=x2-x1; y2=y2-y1; z2=z2-z1;
    x3=x3-x1; y3=y3-y1; z3=z3-z1;
    a1=(16*y2^2*z3*y3^2*z2*x3*r1^2*x2-4*y2^3*z3*y3*z2*x3*r1^2*x2+4*y2^3*z3*y3*z2*x3*x2*r3^2-4*y2*y3^3*z2*x2*z3*r1^2*x3+4*y2*y3^3*z2*x2*z3*r2^2*x3+16*z2*x3^2*x2^2*r1^2*y3*y2*z3-4*z2*x3^3*x2*r1^2*y3*y2*z3+4*z2*x3^3*x2*y2*z3*r2^2*y3-4*x2^3*z3*x3*y2*r1^2*z2*y3+4*x2^3*z3*x3*y2*r3^2*z2*y3-4*y2*z3*z2^3*y3*x3*r1^2*x2+4*y2*z3*z2^3*y3*x3*x2*r3^2+8*y2*z3^2*z2^2*y3*x2*r1^2*x3-4*y2*z3^2*z2^2*y3*x2*r2^2*x3+4*x2^2*y3^2*z2*y2^2*z3*x3^2-4*x2^4*z3^2*x3^2*y3*y2-2*z2^2*x3^4*x2^2*y2^2+2*z2^2*x3^3*y2^2*r1^2*x2-2*z2^2*x3^3*y2^2*x2*r3^2+2*z2^2*x3^5*x2*y2*y3+2*x2^5*z3^2*x3*y3*y2+2*z2^3*y3^2*y2^2*z3*x3^2+2*z2^3*y3^2*x2^2*z3*x3^2+2*z2^3*y3^2*x2^2*z3*r1^2-2*z2^3*y3^2*x2^2*z3*r3^2+2*x2^2*y3^4*z2*y2^2*z3-4*x2^2*y3^2*z2^2*x3^2*y2^2+2*x2^4*y3^2*z2*z3*x3^2-2*x2^2*y3^3*z2^2*y2*x3^2+2*x2^2*y3^3*z2^2*y2*z3^2+2*x2^3*y3^2*z2^2*x3*z3^2+2*x2^2*y3^2*z2*y2^2*z3^3+2*x2^2*y3^3*z2^2*y2*r1^2+2*x2^2*y3^2*z2^2*x3^2*r2^2+2*x2^4*y3^2*z2*z3*r1^2-2*x2^4*y3^2*z2*z3*r3^2-2*x2^2*y3^3*z2^2*y2*r3^2+2*x2^3*y3^2*z2^2*x3*r1^2-2*x2^3*y3^2*z2^2*x3*r3^2+2*y2^4*z3*x3^2*y3^2*z2+2*y2^2*z3*x3^4*z2*x2^2-4*y2^2*z3^2*x3^2*x2^2*y3^2+2*y2^3*z3^2*x3^2*z2^2*y3+2*y2^2*z3^2*x3^3*x2*z2^2-2*y2^3*z3^2*x3^2*x2^2*y3+2*y2^3*z3^2*x3^2*r1^2*y3+2*y2^2*z3*x3^4*z2*r1^2-2*y2^2*z3*x3^4*z2*r2^2+2*y2^2*z3^2*x3^2*x2^2*r3^2-2*y2^3*z3^2*x3^2*r2^2*y3+2*y2^2*z3^2*x3^3*x2*r1^2-2*y2^2*z3^2*x3^3*x2*r2^2-2*y2^2*z3^2*y3^2*x2^3*x3-4*y2^4*z3^2*y3^2*x2*x3+2*y2^2*z3^2*y3^2*x2^2*r3^2+4*y2^3*z3^2*y3*x2^3*x3+2*y2^5*z3^2*y3*x2*x3+4*y2*y3^3*z2^2*x3^3*x2+2*y2*y3^5*z2^2*x3*x2-2*y2^2*y3^2*z2^2*x3^3*x2-4*y2^2*y3^4*z2^2*x3*x2+2*y2^2*y3^2*z2^2*x3^2*r2^2-4*z2^2*x3^4*x2^2*y2*y3+2*z2*x3^2*x2^2*y2^2*z3^3+2*z2*x3^2*y2^4*z3*r1^2+2*z2^2*x3^2*y2^3*r1^2*y3-2*z2*x3^2*y2^4*z3*r3^2-2*z2^2*x3^2*y2^3*r3^2*y3-z2^4*y3^2*x3^2*x2^2-z2^4*y3^2*x3^2*y2^2+2*z2^3*y3^4*x2^2*z3+2*z2^3*y3^2*x2^2*z3^3+2*x2^2*y3^5*z2^2*y2-2*x2^2*y3^4*z2^2*y2^2-2*x2^4*y3^2*z2^2*x3^2+2*x2^3*y3^2*z2^2*x3^3+2*x2^4*y3^4*z2*z3+2*x2^3*y3^4*z2^2*x3+2*x2^2*y3^4*z2^2*r2^2-2*y2^4*z3^2*x3^2*y3^2+2*y2^5*z3^2*x3^2*y3+2*y2^4*z3*x3^4*z2+2*y2^2*z3^2*x3^3*x2^3-2*y2^2*z3^2*x3^4*x2^2+2*y2^4*z3^2*x3^3*x2+2*y2^2*z3*x3^4*z2^3-y2^2*z3^4*x3^2*x2^2+2*y2^4*z3^2*x3^2*r3^2-2*y2^2*z3^2*y3^4*x2^2+2*y2^3*z3^2*y3^3*x2^2-y2^2*z3^4*y3^2*x2^2-y2^4*z3^2*y3^2*x2^2+2*y2^3*y3^3*z2^2*x3^2-y2^2*y3^4*z2^2*x3^2-2*y2^4*y3^2*z2^2*x3^2-z2^4*y3^4*x2^2-2*x2^4*y3^4*z2^2-2*y2^4*z3^2*x3^4-y2^4*z3^4*x3^2-2*z2^2*x3^4*y2^4-z2^4*x3^4*y2^2-2*x2^4*z3^2*y3^4-x2^4*z3^4*y3^2-z3^2*y2^6*x3^2-x3^6*z2^2*y2^2-z3^2*x2^6*y3^2-2*y2^4*x3^4*y3^2+2*y2^4*x3^4*r3^2+2*y2^5*x3^4*y3-y2^4*x3^2*r1^4-y2^4*x3^2*y3^4+2*y2^5*x3^2*y3^3-y2^4*x3^2*r3^4-y2^6*x3^2*y3^2-y2^2*x3^4*r1^4-y2^2*x3^4*x2^4+2*y2^2*x3^5*x2^3-y2^2*x3^4*r2^4-y2^2*x3^6*x2^2-2*y2^4*x3^4*x2^2+2*y2^4*x3^4*r2^2+2*y2^4*x3^5*x2+2*x2^4*y3^5*y2-2*x2^4*y3^4*y2^2+2*x2^4*y3^4*r2^2-x2^2*y3^4*r1^4-x2^2*y3^6*y2^2+2*x2^2*y3^5*y2^3-x2^2*y3^4*y2^4-x2^2*y3^4*r2^4-x2^4*y3^2*r1^4-x2^6*y3^2*x3^2+2*x2^5*y3^2*x3^3-x2^4*y3^2*x3^4-x2^4*y3^2*r3^4+2*x2^5*y3^4*x3-2*x2^4*y3^4*x3^2+2*x2^4*y3^4*r3^2-y3^6*z2^2*x2^2-y2^4*x3^6-y2^6*x3^4-x2^6*y3^4-x2^4*y3^6-2*z2*x3^2*r1^2*y2^2*z3*r3^2-2*z2*y3^2*r2^2*x2^2*z3*r1^2+2*z2*y3^2*r2^2*x2^2*z3*r3^2+2*x2^4*y3^2*z2*z3^3+2*y2*r1^4*z2^2*y3*x3*x2+2*x2^2*y3^2*z2*y2^2*z3*r1^2-8*x2^2*y3^3*z2*r1^2*y2*z3-2*x2^2*y3^2*z2*y2^2*z3*r3^2-8*x2^3*y3^2*z2*z3*r1^2*x3+2*y2^2*z3*x3^2*r1^2*y3^2*z2-8*y2^3*z3*x3^2*r1^2*z2*y3-2*y2^2*z3*x3^2*z2*y3^2*r2^2-8*y2^2*z3*x3^3*z2*r1^2*x2-4*y2^2*z3^2*y3^2*x2*z2^2*x3-4*y2^2*z3^2*y3^2*x2*r1^2*x3+4*y2^2*z3^2*y3^2*x2*r2^2*x3-4*y2^3*z3*y3*z2*x3^3*x2-4*y2^3*z3*y3^3*z2*x3*x2-4*y2^3*z3^3*y3*z2*x3*x2+4*y2^3*z3^2*y3*x2*z2^2*x3-4*y2^3*z3^2*y3*x2*r2^2*x3-4*y2*y3^3*z2*x2^3*z3*x3+4*y2*y3^3*z2^2*x3*x2*z3^2-4*y2*y3^3*z2^3*x2*z3*x3-4*y2*y3^3*z2^2*x3*x2*r3^2-4*y2^2*y3^2*z2^2*x3*r1^2*x2+4*y2^2*y3^2*z2^2*x3*x2*r3^2-4*z2^2*x3^2*x2^2*y2*z3^2*y3+2*z2*x3^2*x2^2*y2^2*z3*r1^2-4*z2^2*x3^2*x2^2*y2*r1^2*y3-2*z2*x3^2*x2^2*y2^2*z3*r3^2+4*z2^2*x3^2*x2^2*y2*r3^2*y3-4*z2^3*x3^3*x2*y2*z3*y3+4*z2^2*x3^3*x2*y2*z3^2*y3-4*z2*x3^3*x2^3*y3*y2*z3-4*z2^2*x3^3*x2*y2*r3^2*y3+4*x2^3*z3^2*x3*y2*z2^2*y3-4*x2^3*z3^3*x3*y2*z2*y3-4*x2^3*z3^2*x3*y2*r2^2*y3+2*x2^2*z3*x3^2*r1^2*y3^2*z2-4*x2^2*z3^2*x3^2*r1^2*y3*y2-2*x2^2*z3*x3^2*z2*y3^2*r2^2+4*x2^2*z3^2*x3^2*y2*r2^2*y3-4*y2*z3^3*z2^3*y3*x3*x2+2*y2*z3^2*z2^4*y3*x2*x3+2*y2*z3^4*z2^2*y3*x3*x2-2*r1^2*y3^2*z2*x2^2*z3*r3^2-2*y2^2*z3*r1^2*z2*x3^2*r2^2+2*r1^4*y3*y2*z3^2*x2*x3+2*z2^2*x3^5*y2^2*x2+2*z2^2*x3^4*y2^3*y3+2*z2*x3^2*y2^4*z3^3+2*z2^2*x3^4*y2^2*r2^2-z2^2*x3^4*x2^2*y3^2+2*x2^5*z3^2*x3*y3^2-x2^4*z3^2*x3^2*y2^2-2*x2^4*z3^2*x3^2*y3^2+2*x2^4*z3^2*y3^3*y2+2*x2^4*z3^2*y3^2*r3^2-2*y2^2*x3^4*z2^2*y3^2-2*z2^2*x3^2*x2^2*y3^4-2*x2^2*z3^2*y2^4*x3^2-2*x2^4*y3^2*y2^2*z3^2+2*y2^2*z3^3*z2^3*x3^2-z3^2*y2^2*r1^4*x3^2-z3^2*y2^2*z2^4*x3^2-z3^2*y2^2*r2^4*x3^2-2*z3^2*y2^4*x3^2*z2^2+2*z3^2*y2^4*x3^2*r2^2-2*x3^4*z2^2*y2^2*z3^2+2*x3^4*z2^2*y2^2*r3^2-x3^2*z2^2*y2^2*r1^4-x3^2*z2^2*y2^2*z3^4-x3^2*z2^2*y2^2*r3^4-2*z3^2*x2^4*y3^2*z2^2+2*z3^2*x2^4*y3^2*r2^2-z3^2*x2^2*r1^4*y3^2-z3^2*x2^2*z2^4*y3^2-z3^2*x2^2*r2^4*y3^2+2*y2^3*x3^4*r1^2*y3-2*y2^3*x3^4*x2^2*y3-2*y2^3*x3^4*r2^2*y3+2*y2^4*x3^3*r1^2*x2-2*y2^4*x3^3*x2*y3^2-2*z2^2*x3^2*x2^2*y3^2*z3^2+2*z2^2*x3^2*x2^2*y3^2*r3^2-2*x2^2*z3^2*y2^2*x3^2*z2^2+2*x2^2*z3^2*y2^2*x3^2*r2^2+2*x2^2*y3^2*y2^2*z3^2*r2^2+2*y2^2*z3^3*z2*x3^2*r1^2-2*y2^2*z3^3*z2*x3^2*r2^2+2*z2^3*x3^2*y2^2*z3*r1^2-2*z2^3*x3^2*y2^2*z3*r3^2+2*x2^2*z3^3*r1^2*y3^2*z2-2*x2^2*z3^3*z2*y3^2*r2^2+2*r1^4*y3^2*z2*x2^2*z3+2*y2^2*z3*r1^4*z2*x3^2+2*x2^2*z3*y3^4*r1^2*z2+2*x2^2*z3^2*y3^3*r1^2*y2-2*x2^2*z3*y3^4*z2*r2^2-2*x2^2*z3^2*y3^3*y2*r2^2+2*x2^3*z3^2*y3^2*r1^2*x3-2*x2^3*z3^2*y3^2*r2^2*x3-2*y2^2*z3^2*z2^2*y3^2*x2^2-2*y2^2*x3^2*z2^2*y3^2*z3^2+2*y2^2*x3^2*z2^2*y3^2*r3^2-4*y2*z3^2*z2^2*y3*x3*x2*r3^2-4*y2*z3^3*z2*y3*x2*r1^2*x3+4*y2*z3^3*z2*y3*x2*r2^2*x3-4*r1^4*y3*y2*z3*z2*x3*x2-4*r1^2*y3*y2*z3^2*x2*r2^2*x3-4*y2*r1^2*z2^2*y3*x3*x2*r3^2+4*r1^2*y3*y2*z3*z2*x3*x2*r3^2+4*y2*r1^2*z2*y3*x2*z3*r2^2*x3+2*z2*x3^2*r2^2*y2^2*z3*r3^2+2*y2*z3^2*r2^4*y3*x2*x3+2*y2*r3^4*z2^2*y3*x3*x2+4*y2^2*x3*x2*y3^2*r1^2*r3^2+4*y2^2*x3*x2*y3^2*r1^2*r2^2-4*y2^2*x3*x2*y3^2*r3^2*r2^2+4*y2*x3^2*x2^2*y3*r1^2*r2^2+4*y2*x3^2*x2^2*y3*r1^2*r3^2-4*y2*x3^2*x2^2*y3*r2^2*r3^2-4*y2^3*x3*x2*y3*z3^2*r3^2-4*y2*x3*x2*y3^3*r1^2*r2^2-4*y2^3*x3*x2*y3*r1^2*r3^2-4*y2*x3*x2*y3^3*z2^2*r2^2-4*y2*x3*x2^3*y3*r1^2*r3^2-4*y2*x3^3*x2*y3*r1^2*r2^2-4*y2*x3^3*x2*y3*z2^2*r2^2-4*y2*x3*x2^3*y3*z3^2*r3^2-4*z3^2*y2^2*r1^2*x3^2*z2^2+2*z3^2*y2^2*r1^2*x3^2*r2^2+2*z3^2*y2^2*z2^2*x3^2*r2^2+2*x3^2*z2^2*y2^2*z3^2*r3^2+2*x3^2*z2^2*y2^2*r1^2*r3^2-4*z3^2*x2^2*r1^2*y3^2*z2^2+2*z3^2*x2^2*r1^2*y3^2*r2^2+2*z3^2*x2^2*z2^2*y3^2*r2^2-2*y2^3*x3^2*r1^2*y3*r3^2-2*y2^3*x3^2*x2^2*y3*r1^2+2*y2^3*x3^2*x2^2*y3*r3^2-2*y2^3*x3^2*r1^2*r2^2*y3+2*y2^3*x3^2*r3^2*r2^2*y3-2*y2^2*x3^3*r1^2*x2*r2^2-2*y2^2*x3^3*r1^2*x2*y3^2-2*y2^2*x3^3*r1^2*x2*r3^2+2*y2^2*x3^3*r2^2*x2*y3^2+2*y2^2*x3^3*r2^2*x2*r3^2+2*y2^2*x3^2*r1^2*y3^2*r2^2+4*y2^2*x3^2*x2^2*y3^2*r2^2+2*y2^2*x3^2*r1^2*x2^2*r3^2+4*y2^2*x3^2*x2^2*y3^2*r3^2-8*y2^3*x3^3*r1^2*x2*y3-2*x2^2*y3^3*r1^2*y2*x3^2-2*x2^2*y3^3*r1^2*y2*r3^2-2*x2^2*y3^3*y2*r1^2*r2^2+2*x2^2*y3^3*y2*x3^2*r2^2+2*x2^2*y3^3*y2*r3^2*r2^2-2*x2^3*y3^2*r1^2*y2^2*x3-2*x2^3*y3^2*r1^2*r2^2*x3-2*x2^3*y3^2*r1^2*x3*r3^2+2*x2^3*y3^2*y2^2*x3*r3^2+2*x2^3*y3^2*r2^2*x3*r3^2+2*x2^2*y3^2*y2^2*r1^2*r3^2-4*y2*z3*r2^2*y3*z2*x3*x2*r3^2-4*x2^4*y3^2*r1^2*x3^2+2*x2^4*y3^2*r1^2*r3^2+2*x2^3*y3^2*r1^2*x3^3+2*x2^4*y3^2*x3^2*r2^2-2*x2^5*y3^2*x3*r3^2-2*x2^3*y3^2*r2^2*x3^3+2*x2^4*y3^2*x3^2*r3^2-y3^2*z2^2*r1^4*x2^2-y3^2*z2^2*x2^2*z3^4-y3^2*z2^2*x2^2*r3^4-2*y3^4*z2^2*x2^2*z3^2+2*y3^4*z2^2*x2^2*r3^2+4*y2^3*x3*x2^3*y3^3+4*y2^3*x3^3*x2*y3^3+2*y2*x3*x2^5*y3^3+2*y2^3*x3^5*x2*y3+2*y2^3*x3*x2*y3^5-4*y2^4*x3*x2*y3^4+2*y2^5*x3*x2*y3^3+2*y2*x3^3*x2^5*y3-4*y2*x3^4*x2^4*y3+2*y2^5*x3^3*x2*y3+2*y2*x3^5*x2^3*y3+2*y2*x3*x2^3*y3^5+4*y2^3*x3^3*x2^3*y3+4*y2*x3^3*x2^3*y3^3-2*y2^4*x3^3*x2*r3^2-2*y2^5*x3^2*r3^2*y3+2*y2^4*x3^2*y3^2*r2^2+2*y2^3*x3^2*r1^2*y3^3+2*y2^3*x3^2*r1^4*y3-4*y2^4*x3^2*r1^2*y3^2-3*y2^4*x3^2*x2^2*y3^2+2*y2^4*x3^2*r1^2*r3^2+2*y2^5*x3^2*r1^2*y3+2*y2^4*x3^2*y3^2*r3^2-2*y2^3*x3^2*y3^3*r2^2-y2^2*x3^2*r1^4*y3^2-3*y2^2*x3^2*x2^4*y3^2-y2^2*x3^2*r2^4*y3^2-y2^2*x3^2*r1^4*x2^2-3*y2^2*x3^2*x2^2*y3^4-y2^2*x3^2*x2^2*r3^4+2*y2^2*x3^3*r1^4*x2+2*y2^2*x3^3*r1^2*x2^3-4*y2^2*x3^4*r1^2*x2^2+2*y2^2*x3^4*r1^2*r2^2+2*y2^2*x3^5*r1^2*x2+2*y2^2*x3^4*x2^2*r2^2-2*y2^2*x3^3*x2^3*r3^2-2*y2^2*x3^5*r2^2*x2-3*y2^2*x3^4*x2^2*y3^2+2*y2^2*x3^4*x2^2*r3^2+2*x2^4*y3^3*y2*r1^2-2*x2^4*y3^3*y2*x3^2-2*x2^4*y3^3*y2*r3^2+2*x2^3*y3^4*r1^2*x3-2*x2^3*y3^4*y2^2*x3-2*x2^3*y3^4*r2^2*x3-2*x2^2*y3^3*y2^3*r3^2+2*x2^2*y3^4*y2^2*r2^2+2*x2^2*y3^5*r1^2*y2+2*x2^2*y3^3*r1^4*y2-4*x2^2*y3^4*r1^2*y2^2+2*x2^2*y3^4*r1^2*r2^2+2*x2^2*y3^3*y2^3*r1^2+2*x2^2*y3^4*y2^2*r3^2-2*x2^2*y3^5*y2*r2^2-x2^2*y3^2*y2^2*r1^4-x2^2*y3^2*y2^2*r3^4-x2^2*y3^2*r1^4*x3^2-x2^2*y3^2*r2^4*x3^2+2*x2^3*y3^2*r1^4*x3+2*x2^5*y3^2*r1^2*x3+2*x2^2*y3^2*r1^2*x3^2*r2^2-8*x2^3*y3^3*r1^2*y2*x3+2*y3^2*z2^2*r1^2*x2^2*r3^2+2*y3^2*z2^2*x2^2*z3^2*r3^2+4*y2^4*x3*x2*y3^2*r3^2+4*y2^3*x3*x2*y3^3*z2^2-4*y2^3*x3*x2*y3^3*r2^2-4*y2^2*x3*x2*y3^4*r1^2-4*y2^2*x3*x2*y3^2*r1^4+8*y2^3*x3*x2*y3^3*r1^2+4*y2*x3*x2^3*y3^3*z2^2-4*y2*x3*x2^3*y3^3*r2^2-4*y2^4*x3*x2*y3^2*r1^2+4*y2^3*x3^3*x2*y3*z3^2-4*y2^3*x3^3*x2*y3*r3^2+4*y2^3*x3*x2*y3^3*z3^2-4*y2^3*x3*x2*y3^3*r3^2+4*y2^2*x3*x2*y3^4*r2^2+2*y2*x3*x2*y3^3*r1^4+2*y2^3*x3*x2*y3*r1^4+2*y2^3*x3*x2*y3*z3^4+2*y2^3*x3*x2*y3*r3^4+2*y2*x3*x2*y3^3*z2^4+2*y2*x3*x2*y3^3*r2^4+2*y2*x3*x2^3*y3*r1^4+2*y2*x3^3*x2*y3*r1^4+2*y2*x3^3*x2*y3*z2^4+2*y2*x3^3*x2*y3*r2^4+2*y2*x3*x2^3*y3*z3^4+2*y2*x3*x2^3*y3*r3^4-4*y2*x3^2*x2^2*y3*r1^4-4*y2*x3^2*x2^4*y3*r1^2+8*y2*x3^3*x2^3*y3*r1^2-4*y2*x3^4*x2^2*y3*r1^2+4*y2*x3^3*x2^3*y3*z2^2-4*y2*x3^3*x2^3*y3*r2^2+4*y2*x3^2*x2^4*y3*r3^2+4*y2^3*x3^3*x2*y3*z2^2-4*y2^3*x3^3*x2*y3*r2^2+4*y2*x3^4*x2^2*y3*r2^2+4*y2*x3^3*x2^3*y3*z3^2-4*y2*x3^3*x2^3*y3*r3^2+4*y2*x3*x2^3*y3^3*z3^2-4*y2*x3*x2^3*y3^3*r3^2+16*y2^2*x3^2*x2^2*y3^2*r1^2);
    b11=(-z2^3*y3^2-x2^2*y3^2*z2-y2^2*z3*x3^2-y2^2*z3*y3^2+y2^3*z3*y3+y2*y3^3*z2-y2^2*y3^2*z2-z2*x3^2*x2^2-z2*x3^2*y2^2+z2*x3^3*x2+x2^3*z3*x3-x2^2*z3*x3^2-x2^2*z3*y3^2+y2*z3*z2^2*y3+y2*x3^2*z2*y3+y2*z3^2*z2*y3+z2*x3*x2*y3^2+z2*x3*x2*z3^2+x2*z3*y2^2*x3+x2*z3*z2^2*x3+x2^2*y3*y2*z3-y2^2*z3^3-z2^3*x3^2-x2^2*z3^3-r1^2*y3^2*z2-y2^2*z3*r1^2+r1^2*y3*y2*z3+y2*r1^2*z2*y3+z2*y3^2*r2^2-z2*x3^2*r1^2+z2*x3^2*r2^2-x2^2*z3*r1^2+x2^2*z3*r3^2+y2^2*z3*r3^2-y2*z3*r2^2*y3-y2*r3^2*z2*y3+z2*x3*r1^2*x2-z2*x3*x2*r3^2+x2*z3*r1^2*x3-x2*z3*r2^2*x3);
    c11=(-2*y2*z3*z2*y3-2*z2*x3*x2*z3+z3^2*y2^2+x3^2*z2^2+z3^2*x2^2+y2^2*x3^2+x2^2*y3^2+y3^2*z2^2-2*y2*x3*x2*y3);
    if(a1<0||c11==0), result=[nan;nan;nan]; return; end % coz c is the denominator and a is under a root
    %     error('Error in interx.m at z'); end
    za=-1/2*(b11-a1^(1/2))/c11;
    zb=-1/2*(b11+a1^(1/2))/c11;
    if(za>zb)
        if(pos), z=za; else z=zb; end
    else
        if(pos), z=zb; else z=za; end
    end
    a1=(2*z*z2*x3-2*x2*z*z3+r1^2*x2-r1^2*x3-x2^2*x3-y2^2*x3-z2^2*x3+r2^2*x3+x2*x3^2+x2*y3^2+x2*z3^2-x2*r3^2);
    b11=(-2*y2*x3+2*x2*y3);
    if(b11==0), result=[nan;nan;nan]; return; end % coz b is the denominator in the expression
%     error('Error in interx.m at y'); end
    y=a1/b11;
    if(x2==0), result=[nan;nan;nan]; return; end
    x = 1/2*(r1^2+x2^2-2*y*y2+y2^2-2*z*z2+z2^2-r2^2)/x2;
    result=[x1;y1;z1;1]+[x;y;z;0];
end
    
    
function [ab1, ab2, ab3] = get_upper_knees_patch_positions(a1,a2,a3)
    ab2 = (b0(:,1)+b0(:,2))/2;
    ab1 = (b0(:,1)+b0(:,3))/2;
    ab3 = (b0(:,3)+b0(:,2))/2;
    ab1 = [ab1, a1'];
    ab2 = [ab2, a2'];
    ab3 = [ab3, a3'];
end


    
    function [l1, l2, l3] = get_low_knees_patch_positions(a1,a2,a3,p)
        l2 = [a2'-(b0(:,1)-b0(:,2))/psi,a2',a2'+(b0(:,1)-b0(:,2))/psi,p(:,2)+(b0(:,1)-b0(:,2))/psi,p(:,2),p(:,2)-(b0(:,1)-b0(:,2))/psi];
        l1 = [a1'-(b0(:,1)-b0(:,3))/psi,a1',a1'+(b0(:,1)-b0(:,3))/psi,p(:,1)+(b0(:,1)-b0(:,3))/psi,p(:,1),p(:,1)-(b0(:,1)-b0(:,3))/psi];
        l3 = [a3'-(b0(:,2)-b0(:,3))/psi,a3',a3'+(b0(:,2)-b0(:,3))/psi,p(:,3)+(b0(:,2)-b0(:,3))/psi,p(:,3),p(:,3)-(b0(:,2)-b0(:,3))/psi];
    end
    
    
    function [a1, a2, a3] = get_sphere_centers(theta1)
        theta = theta1/180*pi;
        a1 = [0, -wb-L*cos(theta(1))+up, -L*sin(theta(1)), 1];
        a2 = [sqrt(3)/2*(wb+L*cos(theta(2)))-sp/2, (wb + L*cos(theta(2)))/2-wp, -L*sin(theta(2)), 1];
        a3 = [-sqrt(3)/2*(wb+L*cos(theta(3)))+sp/2, (wb + L*cos(theta(3)))/2-wp, -L*sin(theta(3)), 1];
    end

    function [a1, a2, a3] = get_knees_pos(theta1)
    
        theta = theta1/180*pi;
        a1 = [0 -wb-L*cos(theta(1)) -L*sin(theta(1)) 1];
        a2 = [sqrt(3)/2*(wb+L*cos(theta(2))) (wb + L*cos(theta(2)))/2 -L*sin(theta(2)) 1];
        a3 = [-sqrt(3)/2*(wb+L*cos(theta(3))) (wb + L*cos(theta(3)))/2 -L*sin(theta(3)) 1];
    end
   
        
    function pb = get_patch(b,marker,color_mark,color_edge)
        pb = patch('XData',b(1,:),'YData',b(2,:),'ZData',b(3,:),'Marker',marker,'FaceColor','none','EdgeColor',color_edge,'MarkerFaceColor',color_mark);
    end
    function p = get_base(theta)
        [c1,c2,c3] = get_sphere_centers(theta);
        p = spheres_intersect(c1,c2,c3);
    end

    %%this function computes Jacobian of the platform position using formula
    %on page 15 in the referance [1] (pdf document)
    function J = get_Jacobian(theta,p)
        %definitions for convinience
        a = wb - up;
        b = sp/2-sqrt(3)/2*wb;
        c = wp-wb/2;
        x = p(1);
        y = p(2);
        z = p(3);
        A = [[x, y+a+L*cos(theta(1)), z+L*sin(theta(1))];[2*(x+b)-sqrt(3)*L*cos(theta(2)),2*(y+c)-L*cos(theta(2)),2*(z+L*sin(theta(2)))];[2*(x-b)-sqrt(3)*L*cos(theta(3)),2*(y+c)-L*cos(theta(3)),2*(z+L*sin(theta(3)))]];
        B = zeros(3);
        B(1,1) = L*((y+a)*sin(theta(1))-z*cos(theta(1)));
        B(2,2) = -L*((sqrt(3)*(x+b)+y+c)*sin(theta(2))+2*z*cos(theta(2)));
        B(3,3) = -L*((sqrt(3)*(x-b)-y-c)*sin(theta(3))-2*z*cos(theta(3)));
        if det(A) ~= 0
            J = inv(A)*B;
        else
             warning('Jacobian singularity!')
             J = NaN;
        end
    end
   
end   