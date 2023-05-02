% Recursive function to search for .mat files
function fileList = findMATFiles(folder)
    fileList = {};  % Initialize an empty cell array to store file paths
    
    % Get the list of all files and folders in the current folder
    contents = dir(folder);
    
    for i = 1:numel(contents)
        % Skip current and parent directory entries
        if strcmp(contents(i).name, '.') || strcmp(contents(i).name, '..')
            continue;
        end
        
        % Construct the full path of the current entry
        fullPath = fullfile(folder, contents(i).name);
        
        if contents(i).isdir
            % Recursively search the subfolder and append its results
            subfolderList = findMATFiles(fullPath);
            fileList = [fileList; subfolderList];
        else
            % Check if the file has the .mat extension
            [~, ~, ext] = fileparts(contents(i).name);
            if strcmp(ext, '.mat')
                % Append the file path to the list
                fileList = [fileList; {fullPath}];
            end
        end
    end
end