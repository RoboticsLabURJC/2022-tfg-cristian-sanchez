#! /usr/bin/env python
import friss as fr
import numpy as np
import matplotlib.pylab as plt
from matplotlib.widgets import Slider, CheckButtons, Button

def update(val):
    global im_default, im_fixed, cbar_def, fixed_selected, res, world_size
    
    # Data extraction from sliders
    Pt = power_t.val
    Gt = gain_t.val
    Gr = gain_r.val
    Fq = freq.val * (10**9)
    l = losses_factor.val
    n = loss_exp.val

    res = res_sl.val
    world_size = (world_sz_sl.val, world_sz_sl.val)

    # Recalculate data with the new values
    my_model.set_values(Pt, Gt, Gr, Fq, l, n)
    data = my_model.model_power_signal(origin)

    # Only update the one that is selected
    if fixed_selected:
        im_fixed.set_data(data)
    else:
        im_default.set_data(data)
        cbar_def.mappable.autoscale()

    # Terminal debugging
    my_model.test(2, 10)

def checkbox(label): 
    global fixed_selected
    fixed_selected = not fixed_selected

def button_function(val):
    global res, world_size, my_model, cbar_def
    print("RESOLUTION:", np.round(res, 1))
    print("SIZE:", int(world_size[0]))

    my_model.reset_world(np.round(res, 1),  (int(world_size[0]), int(world_size[0])))
    data = my_model.model_power_signal(origin)
    
    im_fixed.set_data(data)
    im_default.set_data(data)
    cbar_def.mappable.autoscale()

if __name__ ==  "__main__":
    # -- INIT DATA -- #
    world_size = (100, 100)
    res = 0.5
    origin = (20/res, 20/res)

    my_model = fr.Friss(world_sz=world_size, resolution=res)
    data = my_model.model_power_signal(origin)
    mesh_min, mesh_max = my_model.CUSTOM_RANGE
    x_ticks, y_ticks = world_size

    # -- INIT FOR DISPLAY -- #
    fig = plt.figure("Heatmap")
    ax_def = fig.add_subplot(221)
    ax_fix = fig.add_subplot(222)
    ax_def.set_title("Default")
    title = "Fixed limits (" + str(mesh_min) + ", " + str(mesh_max) +")"
    ax_fix.set_title(title)

    # -- SHOW DATA & COLORBARS -- #
    im_default = ax_def.imshow(data, cmap = 'afmhot', aspect='equal', origin='lower')    
    ax_def.set_xticks(np.arange(start=0, stop=(x_ticks - 1)/res, step=(x_ticks/res)/5))
    ax_def.set_xticklabels(np.arange(start=0, stop=(x_ticks - 1), step=x_ticks/5))
    ax_def.set_yticks(np.arange(start=0, stop=(x_ticks - 1)/res, step=(x_ticks/res)/5))
    ax_def.set_yticklabels(np.arange(start=0, stop=(x_ticks - 1), step=x_ticks/5))
    cbar_def = plt.colorbar(im_default, ax=ax_def)


    im_fixed = ax_fix.imshow(data, cmap = 'afmhot', aspect='equal', origin='lower', 
                             vmin=mesh_min, vmax=mesh_max)
    ax_fix.set_xticks(np.arange(start=0, stop=(x_ticks - 1)/res, step=(x_ticks/res)/5))
    ax_fix.set_xticklabels(np.arange(start=0, stop=(x_ticks - 1), step=x_ticks/5))
    ax_fix.set_yticks(np.arange(start=0, stop=(x_ticks - 1)/res, step=(x_ticks/res)/5))
    ax_fix.set_yticklabels(np.arange(start=0, stop=(x_ticks - 1), step=x_ticks/5))
    cbar_fix = plt.colorbar(im_fixed, ax=ax_fix)

    # -- SLIDERS -- #
    ax_Pt = fig.add_axes([0.15, 0.35, 0.7, 0.03])
    ax_Gt = fig.add_axes([0.15, 0.3, 0.7, 0.03])
    ax_Gr = fig.add_axes([0.15, 0.25, 0.7, 0.03])
    ax_Fq = fig.add_axes([0.15, 0.2, 0.7, 0.03])
    ax_l = fig.add_axes([0.15, 0.15, 0.7, 0.03])
    ax_n = fig.add_axes([0.15, 0.1, 0.7, 0.03]) 

    ax_resolution = fig.add_axes([0.15, 0.05, 0.7, 0.01])
    ax_world_sz = fig.add_axes([0.15, 0.04, 0.7, 0.01])
    

    power_t = Slider(ax_Pt, 'Pt (W)', 0.00001, 100.0, 1.0, valstep=1)
    gain_t = Slider(ax_Gt, 'Gt (W)', 0.00001, 100.0, 1.0, valstep=1.0)
    gain_r = Slider(ax_Gr, 'Gr (W)', 0.00001, 100.0, 1.0, valstep=1.0)
    freq = Slider(ax_Fq, 'Fq (GHz)', 0.00001, 10.0, fr.FREQ_WIFI/(10**9), valstep=0.1)
    losses_factor = Slider(ax_l, 'L', 1.0, 10.0, 1.0, valstep=0.1)
    loss_exp = Slider(ax_n, 'n', 1.6, 6.0, 2.0, valstep=0.1)

    res_sl = Slider(ax_resolution, 'resolution', 0.5, world_size[0]/2, 1.0, valstep=0.5)
    world_sz_sl = Slider(ax_world_sz, 'size (val x val)', 10.0, 100.0, world_size[0], valstep=1.0)

    power_t.on_changed(update)
    gain_t.on_changed(update)
    gain_r.on_changed(update)
    freq.on_changed(update)
    losses_factor.on_changed(update)
    loss_exp.on_changed(update)

    res_sl.on_changed(update)
    world_sz_sl.on_changed(update)

    # -- CHECKBUTTON -- # 
    fixed_selected = False   
    button_ax = fig.add_axes([0.15, 0.4, 0.05, 0.05])
    check = CheckButtons(button_ax, ["Fixed limits"], [fixed_selected])
    check.on_clicked(checkbox)

    # -- SIMPLE BUTTON -- #
    # defining button and add its functionality
    button_ax = fig.add_axes([0.9, 0.04, 0.025, 0.025])
    b_set_res_and_sz = Button(button_ax, 'Set', color="orange")
    b_set_res_and_sz.on_clicked(button_function)

    # Terminal debugging
    my_model.test(2, 10)
    
    # Display app (press q to quit)
    mng = plt.get_current_fig_manager()
    mng.full_screen_toggle()
    plt.show()
