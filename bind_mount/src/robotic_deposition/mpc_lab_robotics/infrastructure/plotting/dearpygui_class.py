
"""
Class for creating a GUI to visualize data
Author: Tony Zheng
"""
import dearpygui.dearpygui as dpg
from math import sin, cos
import time
import signal

class DearPyGuiCreator():
    def __init__(self):
        dpg.create_context() 
        signal.signal(signal.SIGINT, self.stop_dpg)  

        main_window = dpg.add_window(label="Plots", tag="main_window", width=1200, height=1200,)


        with dpg.theme(tag="plot_theme") as theme:
            with dpg.theme_component(dpg.mvLineSeries):
                # dpg.add_theme_color(dpg.mvPlotCol_MarkerFill, [255, 0, 255, 255], category=dpg.mvThemeCat_Plots)
                # dpg.add_theme_color(dpg.mvPlotCol_MarkerOutline, [255, 0, 255, 255], category=dpg.mvThemeCat_Plots)
                # dpg.add_theme_color(dpg.mvPlotCol_Line, (60, 150, 200), category=dpg.mvThemeCat_Plots)
                # dpg.add_theme_style(dpg.mvPlotStyleVar_Marker, dpg.mvPlotMarker_Square, category=dpg.mvThemeCat_Plots)
                # dpg.add_theme_style(dpg.mvPlotStyleVar_MarkerSize, 4, category=dpg.mvThemeCat_Plots)
                dpg.add_theme_style(dpg.mvPlotStyleVar_LineWeight, 4, category=dpg.mvThemeCat_Plots)

        dpg.add_tab_bar(tag="Tab1", reorderable=True, parent="main_window")
        self.plot_tags = []

        dpg.create_viewport(title='UR5e Real-time Data', width=1200, height=1200)
        dpg.set_primary_window(main_window, True)
        dpg.setup_dearpygui()
        dpg.show_viewport()
        # dpg.start_dearpygui()
        # dpg.destroy_context()

        self.tcp_label_nums = {
            0   : 'x',
            1   : 'y',
            2   : 'z',
            'P' : 'Theta',
            'V' : 'Omega',
            'F' : 'T'}

    def generate_subplot_in_tab(self, tag_name, tab_label, tab_parent, plot_label):
        subplot_tag = tag_name + '/subplot'
        with dpg.tab(tag = tag_name, label=tab_label, parent=tab_parent):
            
            with dpg.subplots(2, 1, tag = subplot_tag, parent = tag_name, label=plot_label, width=-1, height=-1, row_ratios=[1.0, 1.0]) as subplot_id:
                    for i in range(2):
                        if i == 0:
                            axis_label = "/linear"
                        else:
                            axis_label = "/angular"
                        subplot_sub_tag = subplot_tag+axis_label
                        with dpg.plot(no_title=True, parent = subplot_tag, tag = subplot_sub_tag):
                            dpg.add_plot_legend()
                                
                            dpg.add_plot_axis(dpg.mvXAxis, label=plot_label+axis_label+"/x_axis", tag=subplot_sub_tag+"/x_axis",parent =subplot_sub_tag)
                            dpg.add_plot_axis(dpg.mvYAxis, label=plot_label+axis_label+"/y_axis", tag=subplot_sub_tag+"/y_axis",parent =subplot_sub_tag)
                            self.plot_tags.append(subplot_sub_tag)

    def generate_plot_in_tab(self, tag_name, tab_label, tab_parent, plot_label):
        plot_tag = tag_name + '/plot'
        with dpg.tab(tag = tag_name, label=tab_label, parent=tab_parent): 
            with dpg.plot(no_title=True, parent = tag_name, tag = plot_tag, width=-1, height=-1):
                dpg.add_plot_legend()
                dpg.add_plot_axis(dpg.mvXAxis, label=plot_label+"/x_axis", tag=plot_tag+"/x_axis",parent =plot_tag)
                dpg.add_plot_axis(dpg.mvYAxis, label=plot_label+"/y_axis", tag=plot_tag+"/y_axis",parent =plot_tag)
                self.plot_tags.append(plot_tag)

    def add_line(self, data_x = [], data_y = [], tag_name = '', label = '', parent_name = ''):
        # self.tag_list.append(tag_name)
        if "joint" in parent_name:
            parent_tag = parent_name +'/plot'
            for i in range(6):
                if data_y == []:
                    dpg.add_line_series(data_x, data_y, tag=tag_name+'/'+label+str(i), label= label+str(i), parent= parent_tag + "/y_axis")
                else:
                    dpg.add_line_series(data_x, data_y[i,:], tag=tag_name+'/'+label+str(i), label= label+str(i), parent= parent_tag + "/y_axis")
        elif "tcp" in parent_name:
            parent_tag = parent_name + '/subplot' 
            for i in range(3):
                if data_y == []:
                    dpg.add_line_series(data_x, data_y, tag=tag_name+'/'+label+self.tcp_label_nums.get(i), label= label+self.tcp_label_nums.get(i), parent= parent_tag+ '/linear' + "/y_axis")
                    dpg.add_line_series(data_x, data_y, tag=tag_name+'/'+self.tcp_label_nums.get(label)+self.tcp_label_nums.get(i), label= self.tcp_label_nums.get(label)+self.tcp_label_nums.get(i), parent= parent_tag+ '/angular' + "/y_axis")
                else:
                    dpg.add_line_series(data_x, data_y[i,:], tag=tag_name+'/'+label+self.tcp_label_nums.get(i), label= label+self.tcp_label_nums.get(i), parent= parent_tag+ '/linear' + "/y_axis")
                    dpg.add_line_series(data_x, data_y[i+3,:], tag=tag_name+'/'+self.tcp_label_nums.get(label)+self.tcp_label_nums.get(i), label= self.tcp_label_nums.get(label)+self.tcp_label_nums.get(i), parent= parent_tag+ '/angular' + "/y_axis")
        
        
    def update_data(self, tag_name, label, data_x, data_y):
        tag_name = tag_name + '_lines'
        if "joint" in tag_name:
            for i in range(6):
                tag = tag_name+'/'+label+str(i)
                dpg.set_value(tag, [data_x, data_y[i,:]])
        elif "tcp" in tag_name: 
            for i in range(3):
                tag = tag_name+'/'+label+self.tcp_label_nums.get(i)
                dpg.set_value(tag, [data_x, data_y[i,:]])
                tag = tag_name+'/'+self.tcp_label_nums.get(label)+self.tcp_label_nums.get(i)
                dpg.set_value(tag, [data_x, data_y[i+3,:]])

    def update_item_label(self, tag_name, label):
        dpg.set_item_label(tag_name, label)

    def set_theme_for_lines(self,tag_name):
        matching = [s for s in dpg.get_aliases() if tag_name in s]
        # print(matching)
        for tag in matching:
            dpg.bind_item_theme(tag, "plot_theme")


    def stop_dpg(self, *args, **kwargs):
        dpg.destroy_context()

    def render_frame(self):
        dpg.render_dearpygui_frame()

    def set_x_axis(self,tag_name, xmin, xmax):
        dpg.set_axis_limits(tag_name, xmin, xmax)

    def set_y_axis(self,tag_name, ymin, ymax):
        dpg.set_axis_limits(tag_name, ymin, ymax)
    
    def set_all_x_axis(self, xmin, xmax):
        matching = [s for s in dpg.get_aliases() if "x_axis" in s]
        for tag in matching:
            self.set_x_axis(tag,xmin,xmax)

    def get_tag_names(self):
        return sorted(dpg.get_aliases())

    def print_tag_names(self):
        for names in sorted(dpg.get_aliases()):
            print(names)

if __name__ == "__main__":
    pygui = DearPyGuiCreator()
    tstart = time.time()
    tag_name = 'Positions'
    # pygui.add_line(tag_name=tag_name)
    pygui.set_x_axis(0,1)
    pygui.set_y_axis(0,1)
    while dpg.is_dearpygui_running(): 
        a = time.time()
        t = time.time()-tstart
        cosdatax = []
        cosdatay = []
        for i in range(0, 500):
            cosdatax.append(0.5 + 0.5 * sin(50 * i / 1000)*(sin(t)+1)/2)
            cosdatay.append(0.5 + 0.5 * cos(50 * i / 1000)*(sin(t)+1)/2)
        # pygui.update_data(cosdatax, cosdatay, tag_name )
        pygui.render_frame()
        print(1/(time.time()-a))

    
