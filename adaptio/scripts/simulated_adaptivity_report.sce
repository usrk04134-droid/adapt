version_numbers = strsplit(getversion(),["-","."])
supported_version = 2026;
if evstr(version_numbers(2)) < supported_version then
    disp("Scilab version must be >= 2026.*.*, aborting.")
end

function [test_cases] = FindTestCasesInFile(file_path)
    test_cases = [];
    os_name = getos();
    command_text = "";
    
    if os_name == "Linux"
        command_text = msprintf("grep -n ""Starting test:"" %s", file_path);
    elseif os_name == "Windows"
        command_text = msprintf("powershell -Command ""Select-String -Pattern ''Starting test:'' -Path ''%s'' | ForEach-Object {\\""$($_.LineNumber): $($_.Line )\\""}""", file_path);
    end

    [status, stdout, stderr] = host(command_text);

    for i = 1:size(stdout,1)
        string_parts = strsplit(stdout(i),":");
        test_cases(i) = struct("start_row", evstr(string_parts(1)),...
                               "end_row", %inf,...
                               "test_name", string_parts(5));
        if i > 1
            test_cases(i-1).end_row = evstr(string_parts(1));
        end
    end

endfunction

function [slices, nbr_samples_per_rev] = ReadAdaptivtyLog(file_path, test_case, waitbar_handle)
    
    fd = mopen(file_path, "rt");
    try
        mgetl(fd, test_case.start_row);
        nbr_rows_in_test_case = -1;
        if ~isinf(test_case.end_row) 
            nbr_rows_in_test_case = test_case.end_row - test_case.start_row;
        end

        file_content = mgetl(fd, nbr_rows_in_test_case);
    catch
        mclose(fd);
        error("Failed to read log file.");
    end

    mclose(fd);
    // Grep the relevant rows in the file Slice sample index
    [slice_rows, w] = grep(file_content,"first_slice_behind_torch");
    
    // Get the JSON objects
    slices = [];
    nbr_samples_per_rev = 0;
    nbr_slices = length(slice_rows);
    for i = 1:nbr_slices
        waitbar(i/nbr_slices, waitbar_handle);
        json_string = "{";
        row_idx = slice_rows(i) + 1;
        unclosed_brace_cnt = 1;
        while unclosed_brace_cnt <> 0
            row = file_content(row_idx);
            json_string = [json_string; row];
            row_idx = row_idx + 1;
            if grep(row, "{")
                unclosed_brace_cnt = unclosed_brace_cnt + 1;
            end
            if grep(row, "}")
                unclosed_brace_cnt = unclosed_brace_cnt - 1;
            end
        end
        
        //Deserialize
        obj = fromJSON(json_string);
        //slices = [slices,obj];
        slices(i) = obj;
        nbr_samples_per_rev = max([nbr_samples_per_rev, obj.slice_sample_index + 1]);
        
    end
endfunction

function [fig] = InitPlotWindow(plot_settings)
    // Init plot window
    handles = [];

    fig = scf();
    fig.figure_size = [626,1000];
    fig.figure_name = msprintf("Adaptivity visualizer (%i)", fig.figure_id);
    clf(fig);
    fig.immediate_drawing = "off";
    fig.color_map = jet(plot_settings.n_colors);
    m = uimenu(fig,'label', 'Adaptivity');
    m2 = uimenu(m,'label', 'Show controls', 'callback', msprintf("CreateTestLogGui(%i)", fig.figure_id));
    delete(fig.children(2));

    deposition_min = plot_settings.current_min / plot_settings.speed_max;
    deposition_max = plot_settings.current_max / plot_settings.speed_min;
    unit_text = "";
    
    for j = 1:3
        subplot(3,1,j)
        ax = gca()
        ax.isoview = "on";
        ax.box = "on";
        ax.axes_reverse(1) = "on";
        ax.font_size = 3;
        ax.title.font_size = 3;
        ax.x_label.text = "ROCS x (m)";
        ax.x_label.font_size = 3;
        ax.y_label.text = "ROCS r (m)";
        ax.y_label.font_size = 3;
        ax.zoom_box = plot_settings.zoom_box;
        
        select j
        case 1
            handles.dep_axes_handle = ax;
            ax.title.text = msprintf("Adaptivity at %.1f°", 0);;
            min_value = deposition_min / 1000;
            max_value = deposition_max / 1000;
            unit_text = "As/mm";
        case 2
            handles.current_axes_handle = ax;
            ax.title.text = msprintf("Current at %.1f°", 0);
            min_value = plot_settings.current_min;
            max_value = plot_settings.current_max;
            unit_text = "A";
        case 3
            handles.speed_axes_handle = ax;
            ax.title.text = msprintf("Speed at %.1f°", 0);
            min_value = plot_settings.speed_min * 6000;
            max_value = plot_settings.speed_max * 6000;
            unit_text = "cm/min";
        end
        
        // Add the color bar
        colorbar_handle = colorbar(min_value, max_value, [1,64]);
        colorbar_handle.title.text = unit_text;
        e = gce();
        e.font_size = 3;
    end

    fig.user_data.plot_settings = plot_settings;
    fig.user_data.handles = handles;
    fig.user_data.slices = [];
    fig.user_data.control_win_tag = "";

endfunction

function [] = PlotBeadsAtPosition(position_index, fig)
    
    k = find(list2vec(fig.user_data.slices.slice_sample_index) == position_index);
    slices = fig.user_data.slices(k);

    scf(fig);
    fig.immediate_drawing = "off";
    plot_settings = fig.user_data.plot_settings;
    handles = fig.user_data.handles;
    
    deposition_min = plot_settings.current_min / plot_settings.speed_max;
    deposition_max = plot_settings.current_max / plot_settings.speed_min;
    ClearPlots(fig);
    //Plot
    for i = [length(slices) : -1 : 1]
        obj = slices(i);
    
        slice_x = list2vec(obj.slice_points_rocs.x);
        slice_r = sqrt(list2vec(obj.slice_points_rocs.y).^2 + list2vec(obj.slice_points_rocs.z).^2);

        // ================================================
        // Current
        sca(handles.current_axes_handle)
        xfpoly([slice_x; slice_x(1)], [slice_r; slice_r(1)]);
        fill_handle = gce();
        fill_handle.clip_state = "on";
        
        if obj.current == [] // This is supposed to be the initial empty joint
            fill_handle.background = color("light gray"); 
        else
            plot_value = obj.current;
            min_value = plot_settings.current_min;
            max_value = plot_settings.current_max;
            fill_handle.background = ceil(1 + (plot_settings.n_colors - 1) * (plot_value - min_value) ./ (max_value - min_value));
        end
        
        if obj.current <> []
            torch_x = obj.torch_pos_rocs.x;
            torch_r = sqrt(obj.torch_pos_rocs.y^2 + obj.torch_pos_rocs.z^2) 
            plot2d(torch_x, torch_r - plot_settings.stickout, -9);
            torch_handle = gce().children(1);
            torch_handle.mark_size_unit = "point";
            torch_handle.mark_size = 4;
            glue([fill_handle, torch_handle.parent]);
        end
        
        // ================================================
        // Speed
        sca(handles.speed_axes_handle);
        xfpoly([slice_x; slice_x(1)], [slice_r; slice_r(1)]);
        fill_handle = gce();
        fill_handle.clip_state = "on";
        
        if obj.current == [] // This is supposed to be the initial empty joint
            fill_handle.background = color("light gray"); 
        else
            plot_value = obj.weld_speed;
            min_value = plot_settings.speed_min;
            max_value = plot_settings.speed_max;
            fill_handle.background = ceil(1 + (plot_settings.n_colors - 1) * (plot_value - min_value) ./ (max_value - min_value));
        end
        
        if obj.current <> []
            torch_x = obj.torch_pos_rocs.x;
            torch_r = sqrt(obj.torch_pos_rocs.y^2 + obj.torch_pos_rocs.z^2) 
            plot2d(torch_x, torch_r - plot_settings.stickout, -9)
            torch_handle = gce().children(1);
            torch_handle.mark_size_unit = "point";
            torch_handle.mark_size = 4;
            glue([fill_handle, torch_handle.parent]);
        end
            
        // ================================================
        // Deposition
        sca(handles.dep_axes_handle);
        xfpoly([slice_x; slice_x(1)], [slice_r; slice_r(1)]);
        fill_handle = gce();
        fill_handle.clip_state = "on";
        
        if obj.current == [] // This is supposed to be the initial empty joint
            fill_handle.background = color("light gray"); 
        else
            plot_value = obj.current / obj.weld_speed;
            min_value = deposition_min;
            max_value = deposition_max;
            fill_handle.background = ceil(1 + (plot_settings.n_colors - 1) * (plot_value - min_value) ./ (max_value - min_value));
        end
    
        if obj.current <> []
            torch_x = obj.torch_pos_rocs.x;
            torch_r = sqrt(obj.torch_pos_rocs.y^2 + obj.torch_pos_rocs.z^2);
            plot2d(torch_x, torch_r - plot_settings.stickout, -9);
            torch_handle = gce().children(1);
            torch_handle.mark_size_unit = "point";
            torch_handle.mark_size = 4;
            glue([fill_handle, torch_handle.parent]);
        end
        
    end
    
    
    SetBeadVisibility(fig, plot_settings.nbr_beads_to_show);
    // Turn on immediate drawing again to show what has been plotted.
    fig.immediate_drawing = "on";

endfunction

function [] = CreateTestLogGui(fig_no)
    
    plot_fig = scf(fig_no);

    control_fig = [];
    // Try get control window if it has been tagged before
    if plot_fig.user_data.control_win_tag <> ""
      control_fig = findobj("tag", plot_fig.user_data.control_win_tag);
    end

    if control_fig == []
        // Existing window not found, create new
        control_fig = createWindow();
        control_fig.figure_name = "Plot control for window " + string(fig_no);
        control_fig.background = color(230,230,230);
        control_fig.figure_size = [600,450];
        control_fig.closerequestfcn = "gcbo().visible = ""off"";";
        //fig.resize = "off"; //Does not seem to work on linux and fucks up figure_size

        os_name = getos();
        if os_name == "Linux"
            [status, uuid, stderr]=host("uuidgen"); //Only works on windows if Win SDK is on path.
        elseif os_name == "Windows"
            [status, uuid, stderr]=host("powershell -Command ""[guid]::NewGuid().ToString()"""); //Using powershell
        end

        if status <> 0
            uuid = string(grand(1,1,'uin',1,1e6)); //Fallback if real UUID could not be generated
        end
        control_fig.tag = uuid;
        plot_fig.user_data.control_win_tag = uuid;
    else
        // Existing window found, raise it.
        control_fig.visible = "on";
        //TODO: This does apparently not work on linux or WSL
        show_window(control_fig);
        return;
    end

    plot_fig.closerequestfcn = msprintf("close(%i);close(%i)", control_fig.figure_id, fig_no);
    nbr_beads = 1;
    plot_settings = plot_fig.user_data.plot_settings;

    enable_controls = "off";
    if plot_fig.user_data.slices <> []
        enable_controls = "on";
    end

    y = 10;
    h = 25;
    w = 300;
    m = 5;
    uicontrol(control_fig,..
              "style", "checkbox",..
              "position", [10,y,200,h],..
              "string", "Show speed",..
              "value", 1,...
              "callback", msprintf("OnShowSpeedChanged(%i)", fig_no),...
              "enable", enable_controls,..
              "BackgroundColor", [1,1,1]*0.9);       
    y = y + h + m;          
    uicontrol(control_fig,..
              "style", "checkbox",..
              "position", [10,y,200,h],..
              "string", "Show current",..
              "value", 1, ...
              "callback", msprintf("OnShowCurrentChanged(%i)", fig_no),...
              "enable", enable_controls,...
              "BackgroundColor", [1,1,1]*0.9);       
    y = y + h + m;  
    uicontrol(control_fig,..
              "style", "checkbox",..
              "position", [10,y,200,h],..
              "string", "Show adaptivity",..
              "value", 1,...
              "callback", msprintf("OnShowAdaptivityChanged(%i)",fig_no),...
              "enable", enable_controls,...
              "BackgroundColor", [1,1,1]*0.9);       
    y = y + h + m;                          
    uicontrol(control_fig,..
              "style", "slider",..
              "value", nbr_beads,..
              "Min", 0,...
              "Max",nbr_beads,...
              "position", [10,y,w,h],...
              "SliderStep", [1,1],..
              "SnapToTicks", "on",...
              "callback", msprintf("OnBeadSliderChanged(%i,%i)", fig_no, control_fig.figure_id),...
              "enable", enable_controls,...
              "tag","bead_slider",..
              "BackgroundColor", [1,1,1]*0.9);
              
    uicontrol(control_fig,..
              "style", "text",..
              "position", [w + 10,y,40,h],..
              "string", string(nbr_beads),...
              "tag","bead_label",..
              "BackgroundColor", [1,1,1]*0.9);

    y = y + h;
    uicontrol(control_fig,..
              "style", "text",..
              "position", [10,y,w,h],..
              "string", "Bead number",..
              "BackgroundColor", [1,1,1]*0.9);
    
    y = y + h + m;          
    uicontrol(control_fig,..
              "style", "slider",..
              "value", 0,..
              "Min", 0,...
              "Max", plot_settings.nbr_samples_per_rev - 1,...
              "position", [10,y,w,h],...
              "SliderStep", [1,1],..
              "SnapToTicks", "on",...
              "callback", msprintf("OnPositionSliderChanged(%i,%i,%i)", fig_no, control_fig.figure_id, 0),...
              "tag","position_slider",..
              "enable", enable_controls,...
              "BackgroundColor", [1,1,1]*0.9);
              
    uicontrol(control_fig,..
              "style", "text",..
              "position", [w + 10,y,40,h],..
              "string", "0°",...
              "tag","position_label",..
              "BackgroundColor", [1,1,1]*0.9);

    y = y + h;
    uicontrol(control_fig,..
              "style", "text",..
              "position", [10,y,w,h],..
              "string", "Weld position",..
              "BackgroundColor", [1,1,1]*0.9);              

    y = y + h + m;
    uicontrol(control_fig,..
              "style", "popupmenu",..
              "position", [10,y,w,h],..
              "string", "hej|san",..
              "BackgroundColor", [1,1,1],...
              "tag", "test_case_combobox");     

    uicontrol(control_fig,..
              "style", "pushbutton",..
              "position", [20 + w,y,70,h],..
              "string", "Load",...
              "enable", enable_controls,...
              "tag", "load_button",...
              "callback", msprintf("OnLoadLogFilePressed(%i, %i)", fig_no, control_fig.figure_id));

    y = y + h;
    uicontrol(control_fig,..
              "style", "text",..
              "position", [10,y,w,h],..
              "string", "Test case",..
              "BackgroundColor", [1,1,1]*0.9);   

    y = y + h + m;
    uicontrol(control_fig,..
              "style", "edit",..
              "position", [10,y,w,h],..
              "string", plot_settings.log_file,..
              "BackgroundColor", [1,1,1],...
              "tag", "file_path_edit");

    uicontrol(control_fig,..
              "style", "pushbutton",..
              "position", [20 + w,y,70,h],..
              "string", "Select",...
              "callback", msprintf("OnSelectFilePressed(%i, %i)", fig_no, control_fig.figure_id));

    y = y + h;
    uicontrol(control_fig,..
              "style", "text",..
              "position", [10,y,w,h],..
              "string", "Log file",..
              "BackgroundColor", [1,1,1]*0.9);         

endfunction

function [] = OnShowCurrentChanged(fig_no)
    checkbox_handle = gcbo();
    fig = scf(fig_no);
    if checkbox_handle.value == 0
        fig.user_data.handles.current_axes_handle.visible = "off";
    else
        fig.user_data.handles.current_axes_handle.visible = "on";
    end
endfunction

function [] = OnShowAdaptivityChanged(fig_no)
    checkbox_handle = gcbo();
    fig = scf(fig_no);
    if checkbox_handle.value == 0
        fig.user_data.handles.dep_axes_handle.visible = "off";
    else
        fig.user_data.handles.dep_axes_handle.visible = "on";
    end
endfunction

function [] = OnShowSpeedChanged(fig_no)
    checkbox_handle = gcbo();
    fig = scf(fig_no);
    if checkbox_handle.value == 0
        fig.user_data.handles.speed_axes_handle.visible = "off";
    else
        fig.user_data.handles.speed_axes_handle.visible = "on";
    end
endfunction

function [] = OnBeadSliderChanged(fig_no, control_win_id)
    h_slider = gcbo(); // Get callback object
    h_slider.callback_type = -1; // Stop listening to slider changes
    control_win = scf(control_win_id);
    fig = scf(fig_no);
    h_label = findobj(control_win, "tag","bead_label");
    //slider_changing = %f;
    //while slider_changing
      slider_val = h_slider.Value;
      h_label.string = string(slider_val);
      fig.user_data.plot_settings.nbr_beads_to_show = slider_val;
      
      SetBeadVisibility(fig, slider_val);
      //slider_changing = h_slider.value <> slider_val;
    //end
    //Check if slider value has changed during the execution of this callback
    // if h_slider.value <> slider_val
    //     OnBeadSliderChanged(fig_no, control_win_id);
    // end

    h_slider.callback_type = 2; // Start listening to changes again
endfunction

function [] = OnPositionSliderChanged(fig_no, control_win_id, iter)

    h_slider = gcbo();
    // Stop listening to slider changes to avoid flooding
    // of events from UI. 
    // Reason: Scilab cannot execute this particular callback
    // fast enough --> build up of waiting callbacks --> lag and crash risk.
    h_slider.callback_type = -1;
    //iter = iter + 1;
    control_win = scf(control_win_id);
    fig_handle = scf(fig_no);

    h_bead_slider = findobj(control_win, "tag","bead_slider");
    h_bead_label = findobj(control_win, "tag","bead_label");
    h_label = findobj(control_win, "tag","position_label");

    slider_changing = %t;
    while slider_changing
        position_index = h_slider.value;
        position_degrees = position_index * 360 / fig_handle.user_data.plot_settings.nbr_samples_per_rev;
        h_label.string = msprintf("%.1f°", position_degrees);
        nbr_beads_at_position = length(find(list2vec(fig_handle.user_data.slices.slice_sample_index) == position_index)) - 1;

        fig_handle.user_data.handles.dep_axes_handle.title.text = msprintf("Adaptivity at %.1f°", position_degrees);
        fig_handle.user_data.handles.current_axes_handle.title.text = msprintf("Current at %.1f°", position_degrees);
        fig_handle.user_data.handles.speed_axes_handle.title.text = msprintf("Speed at %.1f°", position_degrees);

        if nbr_beads_at_position < h_bead_slider.value
            h_bead_slider.value = nbr_beads_at_position;
            h_bead_label.string = string(nbr_beads_at_position);
            fig_handle.user_data.plot_settings.nbr_beads_to_show = nbr_beads_at_position;
        end

        h_bead_slider.max = nbr_beads_at_position

        PlotBeadsAtPosition(position_index, fig_handle);
        slider_changing = h_slider.value <> position_index;
    end
    // Check if pos slider has changed during the callback execution
    // This is done to handle all the missed slider changes during
    // callback execution when listenting is shut off.
    // if position_index <> h_slider.value & iter < 15;
    //     OnPositionSliderChanged(fig_no, control_win_id, iter);
    // end

    h_slider.callback_type = 2; // Start listening to changes again

endfunction


function [] = OnSelectFilePressed(fig_no, control_win_id)
    control_win = scf(control_win_id);
    fig_handle = scf(fig_no);  
    file_path = uigetfile(["*.txt"]);

    if file_path <> []
        h_label = findobj(control_win, "tag", "file_path_edit");
        h_label.string = file_path;
        h_button = findobj(control_win, "tag", "load_button");
        h_button.enable = "on";
        h_test_combo = findobj(control_win, "tag", "test_case_combobox");

        test_cases = FindTestCasesInFile(file_path);
        items = "";
        for i = 1:length(test_cases)
            items = items + "|" + test_cases(i).test_name;
        end

        h_test_combo.string = items;
        h_test_combo.value = 1;
        fig_handle.user_data.plot_settings.test_cases = test_cases;
    end

endfunction

function [] = OnLoadLogFilePressed(fig_no, control_win_id)

    control_win = scf(control_win_id);
    fig_handle = scf(fig_no);
    h_label = findobj(control_win, "tag", "file_path_edit");
    h_combobox = findobj(control_win, "tag", "test_case_combobox");
    file_path = h_label.string;
    test_case_index = h_combobox.value;
    test_case = fig_handle.user_data.plot_settings.test_cases(test_case_index);
    fig_handle.user_data.plot_settings.log_file = file_path;

    ClearPlots(fig_handle);
    DisableControls(control_win);

    // Read new data
    p_handle = waitbar("Parsing log file..");
    try
        [slices, nbr_samples_per_rev] = ReadAdaptivtyLog(file_path, test_case, p_handle);
    catch
        messagebox("Could not read log file.", "Error", "error", "OK", "modal");
        close(p_handle);
        EnableControls(control_win);
        return;
    end
    close(p_handle);

    nbr_beads_at_position = length(find(list2vec(slices.slice_sample_index) == 0)) - 1;
    fig_handle.user_data.plot_settings.nbr_samples_per_rev = nbr_samples_per_rev;
    fig_handle.user_data.plot_settings.nbr_beads_to_show = nbr_beads_at_position;
    fig_handle.user_data.slices = slices;

    // Reset position and bead sliders
    h_slider = findobj(control_win, "tag","bead_slider");
    h_label = findobj(control_win, "tag","bead_label");
    h_slider.Value = 0;
    h_slider.max = nbr_beads_at_position
    h_slider.Value = nbr_beads_at_position;
    h_label.string = string(nbr_beads_at_position);

    h_slider = findobj(control_win, "tag","position_slider");
    h_label = findobj(control_win, "tag","position_label");
    h_slider.Value = 0;
    h_slider.max = nbr_samples_per_rev - 1;
    h_label.string = "0°";

    // Replot
    PlotBeadsAtPosition(0, fig_handle);
    EnableControls(control_win);
    
    fig_handle.user_data.handles.dep_axes_handle.title.text = msprintf("Adaptivity at %.1f°", 0);
    fig_handle.user_data.handles.current_axes_handle.title.text = msprintf("Current at %.1f°", 0);
    fig_handle.user_data.handles.speed_axes_handle.title.text = msprintf("Speed at %.1f°", 0);

endfunction

function [] = EnableControls(control_win_handle)
    
    for i = 1:length(control_win_handle.children)
        if control_win_handle.children(i).type == "uicontrol"
            control_win_handle.children(i).enable = "on";
        end
    end

endfunction

function [] = DisableControls(control_win_handle)
    
    for i = 1:length(control_win_handle.children)
        if control_win_handle.children(i).type == "uicontrol"
            control_win_handle.children(i).enable = "off";
        end
    end

endfunction

function [] = ClearPlots(fig)
    //Erase previous plots
    handles = fig.user_data.handles;
    delete(handles.current_axes_handle.children);
    delete(handles.speed_axes_handle.children);
    delete(handles.dep_axes_handle.children);

endfunction

function [] = SetBeadVisibility(fig, nbr_beads_to_show)

    // Weld speed axes
    fig.user_data.handles.speed_axes_handle.children(1:nbr_beads_to_show + 1).visible = "on";
    fig.user_data.handles.speed_axes_handle.children(nbr_beads_to_show + 2 : $).visible = "off";
    // Weld current axes
    fig.user_data.handles.current_axes_handle.children(1:nbr_beads_to_show + 1).visible = "on";
    fig.user_data.handles.current_axes_handle.children(nbr_beads_to_show + 2 : $).visible = "off";
    // Adaptivity axes
    fig.user_data.handles.dep_axes_handle.children(1:nbr_beads_to_show + 1).visible = "on";
    fig.user_data.handles.dep_axes_handle.children(nbr_beads_to_show + 2 : $).visible = "off";
endfunction


// Init the plot window

file_path = "";
plot_settings = struct("n_colors", 64,..
    "stickout", 25e-3,..
    "current_max", 850,..
    "current_min", 650,..
    "speed_max", 95 /(100*60),..
    "speed_min", 80 /(100*60),..
    "zoom_box", [-0.05,0.9627521,0.05,1.01,-1,1],..
    "log_file", file_path,...
    "nbr_samples_per_rev", 0,...
    "nbr_beads_to_show", 0);


//[slices, nbr_samples_per_rev] = ReadAdaptivtyLog(file_path);
//plot_settings.nbr_samples_per_rev = nbr_samples_per_rev;
//plot_settings.nbr_beads_to_show = length(find(list2vec(slices.slice_sample_index) == 0)) - 1;
fig  = InitPlotWindow(plot_settings);
//fig.user_data.slices = slices;
fig.user_data.plot_settings = plot_settings;
//PlotBeadsAtPosition(0, fig);


