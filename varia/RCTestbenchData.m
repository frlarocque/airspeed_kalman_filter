classdef RCTestbenchData
    
    properties
        name
        folder
        t
        pwm
        servo1
        servo2
        servo3
        accX
        accY
        accZ
        Qm
        T
        U
        I
        RPMe
        RPMo
        P_elec
        P_mech
        Eff_mot
        Eff_prop
        Eff_tot
        Vinf
        dP_airspeed
        Eff_fw
        vibration
        Temp_amb
        Notes
        bias
        actuator
    end
    methods
        function this = RCTestbenchData(file_path, det_bias, save_bias)
            if nargin < 3
                save_bias = 0;
            end
            if nargin < 2
                det_bias = 1;
            end
            
            [this.folder, this.name, ext] = fileparts(file_path);
            
            if strcmp(ext,'.csv')
                
                fid = fopen(file_path);
                if fid > 0
                    S = readlines(file_path);
                    columns = length(split(S(1), ','))-1;

                    if contains(this.name, 'StepsTest')
                        format = strcat(sprintf('%s', repmat('%s', 1, columns-1)), '%s');
                        headers = textscan(fid, format, 1, ...
                            'Delimiter', ',');

                        format = strcat(sprintf('%s', repmat('%f', 1, columns-1)), '%s');
                        data = textscan(fid, format, ...
                            'Delimiter', ',', 'Headerlines', 2);
                    elseif contains(this.name, '90PERCENT')
                        format = strcat(sprintf('%s', repmat('%s', 1, columns-3)), '%s%s%s');
                        headers = textscan(fid, format, 1, ...
                            'Delimiter', ',');

                        format = strcat(sprintf('%s', repmat('%f', 1, columns-3)), '%s%s%s');
                        data = textscan(fid, format, ...
                            'Delimiter', ',', 'Headerlines', 2);
                    elseif contains(this.name, 'UDP')
                        format = strcat(sprintf('%s', repmat('%s', 1, columns-3)), '%s%s%s');
                        headers = textscan(fid, format, 1, ...
                            'Delimiter', ',');

                        format = strcat(sprintf('%s', repmat('%f', 1, columns-1)), '%s');
                        data = textscan(fid, format, ...
                            'Delimiter', ',', 'Headerlines', 2);
                    else
                        format = strcat(sprintf('%s', repmat('%s', 1, columns-1)), '%s');
                        headers = textscan(fid, format, 1, ...
                            'Delimiter', ',');

                        format = strcat(sprintf('%s', repmat('%f', 1, columns-1)), '%s');
                        data = textscan(fid, format, ...
                            'Delimiter', ',', 'Headerlines', 2);
                    end

                    fclose(fid);

                    for l = 1:length(headers)
                        switch headers{l}{1}
                            case '﻿Time (s)'
                                variable = 't';
                            case 'Time (s)'
                                variable = 't';
                            case 'ESC signal (µs)'
                                variable = 'pwm';
                            case 'ESC (µs)'
                                variable = 'pwm';
                            case 'Servo 1 (µs)'
                                variable = 'servo1';
                            case 'Servo 2 (µs)'
                                variable = 'servo2';
                            case 'Servo 3 (µs)'
                                variable = 'servo3';
                            case 'AccX (g)'
                                variable = 'accX';
                            case 'AccY (g)'
                                variable = 'accY';
                            case 'AccZ (g)'
                                variable = 'accZ';
                            case 'Torque (N·m)'
                                variable = 'Qm';
                            case 'Thrust (kgf)'
                                variable = 'Tkgf';
                            case 'Thrust (N)'
                                variable = 'T';
                            case 'Voltage (V)'
                                variable = 'U';
                            case 'Current (A)'
                                variable = 'I';
                            case 'Motor Electrical Speed (RPM)'
                                variable = 'RPMe';
                            case 'Motor Optical Speed (RPM)'
                                variable = 'RPMo';
                            case 'Electrical Power (W)'
                                variable = 'P_elec';
                            case 'Mechanical Power (W)'
                                variable = 'P_mech';
                            case 'Motor Efficiency (%)'
                                variable = 'Eff_mot';
                            case 'Propeller Mech. Efficiency (N/W)'
                                variable = 'Eff_prop';
                            case 'Propeller Mech. Efficiency (kgf/W)'
                                variable = 'Eff_prop_kgf';
                            case 'Overall Efficiency (N/W)'
                                variable = 'Eff_tot';
                            case 'Overall Efficiency (kgf/W)'
                                variable = 'Eff_tot_kgf';
                            case 'Airspeed (m/s)'
                                variable = 'Vinf';
                            case 'Airspeed Pressure (Pa)'
                                variable = 'dP_airspeed';
                            case 'Electrical forward flight efficiency (%)'
                                variable = 'Eff_fw';
                            case 'Vibration (g)'
                                variable = 'vibration';
                            case 'Ambient Temp (ºC)'
                                variable = 'Temp_amb';
                            case 'App message'
                                variable = 'Notes';
                            otherwise
                                variable = 'None';
                        end

                        g = 9.81;
                        if strcmp(variable, 'None')
                            % pass
    %                     elseif strcmp(variable, 'T')
    %                         if mean(data{l}) <= 0
    %                             this.(variable) = -data{l};
    %                         else
    %                             this.(variable) = data{l};
    %                         end
                        elseif strcmp(variable, 'Tkgf')
                            this.T = data{l} .* g;
                        elseif strcmp(variable, 'Eff_prop_kgf')
                            this.Eff_prop = data{l} .* g;
                        elseif strcmp(variable, 'Eff_tot_kgf')
                            this.Eff_tot = data{l} .* g;
                        elseif contains(variable, 'acc')
                            this.(variable) = data{l} .* g;
                        else
                            this.(variable) = data{l};
                        end
                    end
                else
                    warning('Could not open file: %s', file_path)
                end
            end
            
            if det_bias
                this = this.det_bias;
            end
            
            if save_bias
                this.save_bias;
            end
        end
        function plt(this, Xname, Yname, LineSpec, MKsize)
            if nargin < 5
                MKsize = 6;
            end
            if nargin < 4
                LineSpec = '.';
            end
            if nargin < 3
                Yname = 'T';
            end
            if nargin < 2
                Xname = 't';
            end
            
            X = this.(Xname);
            Y = this.(Yname);
            
            displayname = strcat(this.name);
            
            h = plot(X, Y, LineSpec, ...
                'DisplayName', displayname, ...
                'Markersize', MKsize);

            legend('Location', 'SouthEast')
            xlabel(this.getLabel(Xname))
            ylabel(this.getLabel(Yname))
        end
        function this = det_bias(this)
            
            % Try to read bias
            file_path = fullfile(this.folder, 'bias', strcat(this.name, '.txt'));
            fid = fopen(file_path, 'r');
            
            if fid > 0
                lines = readlines(file_path);
                
                for n = 1:length(lines)
                    line = lines{n};
                    if ~isempty(line)
                        r = regexp(line, "(?<force>\w+): (?<value>[-]*[0-9]+.[0-9]*)", 'names');
                    
                        this.bias.(r.force) = str2double(r.value);
                    end
                end
                fclose(fid);
            else
                dt = mean(diff(this.t));
                t_buffer = 2;

                variable = {'T', 'Qm'};

                g = zeros(size(this.(variable{1})));

                for n = 1:length(variable)
                    g = g + (this.(variable{n})).^2;
                end

                g_limit = [1, 10, 100];
                
                for n = 1:length(g_limit)
                
                    a = find((diff(g)./dt <= g_limit(n)) == 0);
                
                    if ~isempty(a)
                        a_id_t1 = a(1) - floor(t_buffer / dt);
                        a_id_t2 = a(end) + floor(t_buffer /dt);
                    else
                        a_id_t1 = NaN;
                        a_id_t2 = NaN;
                    end
                    
                    if a_id_t1 > 0
                        break
                    end
                end
                
                % For determining bias manually:
                % figure; t = this.t(1:end-1); plot(t, diff(g)./dt)
                % a_id_t1 = floor(10/dt);

                for n = 1:length(variable)
                    if a_id_t1 > 0
                        a_avg = mean(this.(variable{n})(1:a_id_t1));
                    elseif a_id_t2 < length(g)
                        a_avg = mean(this.(variable{n})(a_id_t2:end));
                    elseif isnan(a_id_t1) && isnan(a_id_t2)
                        a_avg = mean(this.(variable{n}));
                    else
                        warning('Could not find bias')
                        a_avg = 0;
                    end

                   this.bias.(variable{n}) = a_avg;
                end
            end
            
            % Apply bias
            variable = fieldnames(this.bias);
            for n = 1:length(variable)
                this.(variable{n}) = this.(variable{n}) - this.bias.(variable{n});
            end
        end
        function save_bias(this)
            
            file_path = fullfile(this.folder,'bias', strcat(this.name, '.txt'));
            
            fid = fopen(file_path, 'w+');
            
            variables = fieldnames(this.bias);
            
            for n = 1:length(variables)
                fprintf(fid, "%s: %.3f\n", variables{n}, this.bias.(variables{n}));
            end
            fclose(fid);
        end
        function this = getActuatorResponseData(this, t_settling)
            if nargin < 3
                t_settling = 1;
            end
        
            % Find blocks of constant pwm
        
            t_temp = this.t;
            d_pwm = gradient(this.pwm);
        
            m = 1;
            ts{1}.start = 0;
            ts{1}.start_id = 1;
            for nn = 1:length(d_pwm)
        
                if abs(d_pwm(nn)) > 5
                    if t_temp(nn) >= ts{m}.start + t_settling
                        m = m+1;
                        ts{m}.start = t_temp(nn);
                        ts{m}.start_id = nn;
        
                        ts{m}.end = t_temp(nn+1);
                        ts{m}.end_id = nn+1;
        
                    end
                elseif abs(d_pwm(nn)) < 5
                    ts{m}.end = t_temp(nn);
                    ts{m}.end_id = nn;
                end
            end
        
            mm = 1;
            for m = 2:length(ts)%-2
                if (ts{m}.end - ts{m}.start) > 1.5
                    ts2{mm} = ts{m};
                    mm = mm+1;
                end
            end
        
            this.actuator.ts = ts2;
        end
        function [] = pltActuatorResponse(this, Yname, plot_actuator_id, LineSpec, MKsize)
            if nargin < 5
                MKsize = 4;
            end
            if nargin < 4
                LineSpec = '.';
            end
            if nargin < 3 || isempty(plot_actuator_id)
                plot_actuator_id = 1:length(this.actuator.ts);
            end
            if nargin < 2
                Yname = 'RPMo';
            end


            for n = 1:length(plot_actuator_id)
                nn = plot_actuator_id(n);
                t_temp = this.actuator.ts{nn};
                dt = mean(diff(this.t));
                t_span = t_temp.start_id:t_temp.end_id;
                t_min = t_temp.start;

                t_2 = t_temp.start_id + floor(1/dt);
                t_3 = t_temp.start_id + floor(2/dt);

                if t_2 < t_temp.end_id

                    X = this.t(t_span) - t_min;

                    Y = this.(Yname)(t_span);
                    Y0 = this.(Yname)(t_span(1));
                    Y2 = mean(this.(Yname)(t_2:t_3));
                    Y = (Y - Y0) ./ abs(Y2 - Y0);

                    d_Y = gradient(Y);
                    id = find(abs(d_Y) > 0.01, 1);

                    X = X(id:end) - X(id);
                    Y = Y(id:end);

                    h = plot(X, Y, LineSpec, ...
                        ...'DisplayName', displayName, ...
                        'Markersize', MKsize);

                    try
                        h.Color = color;
                    catch
                        color = h.Color;
                    end
                end

            end
        end
    end
    methods(Static)
        function label = getLabel(Name)
            label = '';
            switch Name
                case 't'
                    label = 'Time (s)';
                case 'pwm'
                    label = 'ESC signal [µs]';
                case 'accX'
                    label = 'Acceleration in x direction [m/s2]';
                case 'accY'
                    label = 'Acceleration in y direction [m/s2]';
                case 'accZ'
                    label = 'Acceleration in z direction [m/s2]';
                case 'T'
                    label = 'Thrust (N)';
                case 'RPM'
                    label = 'RPM [rev/min]';
                case 'Qm'
                    label = 'Torque [Nm]';
                case 'I'
                    label = 'Current [A]';
                case 'U'
                    label = 'Voltage [V]';
                case 'Pelec'
                    label = 'Electric power [W]';
                case 'Pshaft'
                    label = 'Mechanical power [W]';
                case 'Eff_mot'
                    label = 'Motor efficiency [%]';
                case 'Eff_prop'
                    label = 'Propeller efficiency [N/W]';
                case 'Eff_tot'
                    label = 'Total efficiency [N/W]';
                case 'FOM'
                    label = 'Figure of merit [%]';
                case 'Vinf'
                    label = 'Airspeed [m/s]';
                case 'Temp'
                    label = 'Ambient temperature [deg C]';
            end
        end
    end
end
