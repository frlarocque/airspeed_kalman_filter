classdef RCTestbench
    
    properties
        name    % Name, showing motor, propeller and configuration
        perf    % Data for performance tests
        resp    % Data for step response tests
        prop    % Details of the propeller
        motor   % Details of the motor
    end
    methods
        function this = RCTestbench()
        end
        function perfplt(this, Xname, Yname, LineSpec, MKsize)
            if nargin < 5
                MKsize = 4;
            end
            if nargin < 4
                LineSpec = 'o-';
            end
            if nargin < 3
                Yname = 'T';
            end
            if nargin < 2
                Xname = 'RPM';
            end
            
            this.plt('perf',Xname, Yname, LineSpec, MKsize);
        end
        function plt(this, test, Xname, Yname, LineSpec, MKsize)
            if nargin < 6
                MKsize = 4;
            end
            if nargin < 5
                LineSpec = 'o-';
            end
            if nargin < 4
                Yname = 'T';
            end
            if nargin < 3
                Xname = 'RPM';
            end
            if nargin < 2
                test = 'perf';
            end
            
            tests = fields(this.(test));
            
            X = [];
            Y = [];
            for n = 1:length(tests)
                X = [X; this.(test).(tests{n}).(Xname)];
                Y = [Y; this.(test).(tests{n}).(Yname)];
            end
            
            displayname = strcat(this.name);%, ' -', tests{n});
            
            h = plot(X, Y, LineSpec, ...
                'DisplayName', displayname, ...
                'Markersize', MKsize);
            
            if strcmp(Yname, 'FOM')
                ylim([0 100])
            end

            legend('Location', 'SouthEast')
            xlabel(this.getLabel(Xname))
            ylabel(this.getLabel(Yname))
        end
        function this = getFOM(this)
            if ~isempty(this.perf)
                R = this.prop.D/2;
                rho = 1.225;
                A = pi()*R^2;
                
                tests = fieldnames(this.perf);
                for n = 1:length(tests)
                    T = this.perf.(tests{n}).T;
                    P = this.perf.(tests{n}).Pshaft;
                    P(P<=1) = NaN;
                    T(T<=0) = NaN;
                    this.perf.(tests{n}).FOM = (T.^(3/2) .* sqrt(0.5 ./ (rho .* A)) ./ P) .* 100;
                end
            end
        end
        function getVariables(this)
            tests = fieldnames(this.perf);
            fields = fieldnames(this.perf.(tests{1}));
            for n = 1:length(fields)
                if ~strcmp(fields{n}, 'name')
                    fprintf('%s\t\t %s \n',fields{n},this.getLabel(fields{n}))
                end
            end
        end
    end
    methods(Static)
        function this = getData(folder)
            this = RCTestbench();
            
            folder_parts = regexp(folder, filesep, 'split');
            this.name = folder_parts{end};
            
            files = dir(folder);
            
            n_perf = 0;
            n_resp = 0;
            for n = 1:length(files)
                file = files(n);
                if file.bytes > 0
                    file_path = fullfile(folder, file.name);
                    
                    if contains(file.name, 'test_setup')
                        S = readlines(file_path);
                        
                        for nn = 1:length(S)
                            line = S(nn);
                            
                            if contains(line, 'Motor')
                                this.motor.name = string(regexpi(line, '(?<=Motor: )\w+', 'match'));
                            elseif contains(line, 'Kv')
                                this.motor.kv = str2double(regexpi(line, '(?<=Kv: )\w+', 'match'));
                            elseif contains(line, 'Propeller')
                                this.prop.name = string(regexpi(line, '(?<=Propeller: )\w+', 'match'));
                            elseif contains(line, 'Diameter')
                                this.prop.D = this.inch2meter(...
                                    str2double(...
                                    regexpi(line, '(?<=Diameter: )[\d|.]+', 'match')));
                            elseif contains(line, 'Pitch')
                                this.prop.pitch = this.inch2meter(...
                                    str2double(...
                                    regexpi(line, '(?<=Pitch: )[\d|.]+', 'match')));
                            end
                        end
                        

                    else
                        fid = fopen(file_path);
                        
                        if contains(file.name, 'StepsTest')
                            n_perf = n_perf + 1;
                            test = 'perf';
                            nn = n_perf;

                            headers = textscan(fid, '%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s', 1, ...
                                'Delimiter', ',');
                            data = textscan(fid, '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%s%s%f', ...
                                'Delimiter', ',', 'Headerlines', 1);

                        elseif contains(file.name, '90PERCENT')
                            n_resp = n_resp + 1;
                            test = 'resp';
                            nn = n_resp;

                            headers = textscan(fid, '%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s', 1, ...
                                'Delimiter', ',');
                            data = textscan(fid, '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%s', ...
                                'Delimiter', ',', 'Headerlines', 1);

                        end

                        for l = 1:length(headers)
                            switch headers{l}{1}
                                case '﻿Time (s)'
                                    variable = 't';
                                case 'ESC signal (µs)'
                                    variable = 'pwm';
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
                                    variable = 'RPM';
                                case 'Electrical Power (W)'
                                    variable = 'Pelec';
                                case 'Mechanical Power (W)'
                                    variable = 'Pshaft';
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
                                otherwise
                                    variable = 'None';
                            end

                            testnr = ['test' + string(nn)];

                            g = 9.81;
                            if strcmp(variable, 'None')
                                
                            elseif strcmp(variable, 'T')
                                if mean(data{l}) <= 0
                                    this.(test).(testnr).(variable) = -data{l};
                                else
                                    this.(test).(testnr).(variable) = data{l};
                                end
                            elseif strcmp(variable, 'Tkgf')
                                this.(test).(testnr).T = data{l}*g;
                            elseif strcmp(variable, 'Eff_prop_kgf')
                                this.(test).(testnr).Eff_prop = data{l}*g;
                            elseif strcmp(variable, 'Eff_tot_kgf')
                                this.(test).(testnr).Eff_tot = data{l}*g;
                            else
                                this.(test).(testnr).(variable) = data{l};
                            end 
                        end
                        fclose(fid); 
                        this.(test).(testnr).name = file.name;
                    end
                end
            end
            this = this.getFOM;
        end
        function label = getLabel(Name)
            label = '';
            switch Name
                case 't'
                    label = 'Time (s)';
                case 'pwm'
                    label = 'ESC signal [µs]';
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
            end
        end
        function output = inch2meter(input)
            output = input * 0.0254;
        end
    end
end
