#! /usr/bin/env python
'''
FRISS APP

Here is where the GUI of the friss model is done.

There are defined some elements that allows the user
to modify dinamically the signal behavior.

Also, it provides a graphical representation, in order
to see the results on an easier way.
'''
import numpy as np
import matplotlib.pylab as plt
from matplotlib.widgets import Slider, CheckButtons, Button
import friss as fr

# -- CONSTANTS -- #
DEFAULT_WORLD_SIZE = (10, 10)   # size range --> 10 <= (a x a) <= 100
DEFAULT_RESOLUTION = 1.0        # 1 default
DEFAULT_ORIGIN = (3, 5)       # Origin of the signal
CUSTOM_RANGE = (-40, 40)        # Limits for fixed representation


def update(val):
    '''
    Sliders update method. Also updates figures.
    '''
    # Data extraction from sliders
    Pt = power_t.val
    Gt = gain_t.val
    Gr = gain_r.val
    Fq = freq.val * (10**9)
    l = losses_factor.val
    n = loss_exp.val

    # Recalculate data with the new values
    my_model.set_values(Pt, Gt, Gr, Fq, l, n)
    current_data = my_model.model_power_signal(origin)

    # Only update the one that is selected
    if fixed_selected:
        im_fixed.set_data(current_data)
    else:
        im_default.set_data(current_data)
        cbar_def.mappable.autoscale()


def update_res_sz(val):
    '''
    Set sliders update method, it must do nothing, because it waits set button.
    '''
    pass


def checkbox(label):
    '''
    Fixed limit button update method.
    '''
    global fixed_selected
    fixed_selected = not fixed_selected


def button_function(val):
    '''
    Set button update method. Also updates figures.
    '''
    global world_size, res, origin

    world_size = (int(world_sz_sl.val), int(world_sz_sl.val))
    res = res_sl.val
    origin = (int(np.round(DEFAULT_ORIGIN[0]/res)), int(np.round(DEFAULT_ORIGIN[1]/res)))
    check_origin_range()

    my_model.reset_world(res, (world_size[0], world_size[1]))
    current_data = my_model.model_power_signal(origin)

    x_ticks_loc, y_ticks_loc = world_size
    ticks_x_ar = np.around(np.arange(start=0, stop=(x_ticks_loc-1), step=x_ticks_loc/5), decimals=1)
    ticks_y_ar = np.around(np.arange(start=0, stop=(y_ticks_loc-1), step=y_ticks_loc/5), decimals=1)

    ax_def.set_xticklabels(ticks_x_ar)
    ax_def.set_yticklabels(ticks_y_ar)
    ax_fix.set_xticklabels(ticks_x_ar)
    ax_fix.set_yticklabels(ticks_y_ar)

    im_fixed.set_data(current_data)
    im_default.set_data(current_data)
    cbar_def.mappable.autoscale()


def check_origin_range():
    '''
    Checks if the origin is inside the size. If not, puts origin in the middle.
    '''
    global origin

    rows, cols = world_size
    o_row, o_col = origin
    new_o_row, new_o_col = origin

    if rows < o_row or o_row < 0:
        print("Warning, x origin out of range --> new x is now total_x/2")
        new_o_row = int(np.round(rows/2))

    if cols < o_col or o_col < 0:
        print("Warning, y origin out of range --> new y is now total_y/2")
        new_o_col = int(np.round(cols/2))

    origin = (new_o_row, new_o_col)


if __name__ ==  "__main__":
    # -- INIT DATA -- #
    world_size = DEFAULT_WORLD_SIZE
    res = DEFAULT_RESOLUTION
    origin = (int(np.round(DEFAULT_ORIGIN[0]/res)), int(np.round(DEFAULT_ORIGIN[1]/res)))
    check_origin_range()

    my_model = fr.Friss(world_sz=world_size, resolution=res)
    data = my_model.model_power_signal(origin)
    mesh_min, mesh_max = CUSTOM_RANGE
    x_ticks, y_ticks = world_size

    # -- INIT FIGURES -- #
    fig = plt.figure("Heatmap")
    ax_def = fig.add_subplot(221)
    ax_fix = fig.add_subplot(222)
    ax_def.set_title("Default")
    title = "Fixed limits (" + str(mesh_min) + ", " + str(mesh_max) +")"
    ax_fix.set_title(title)

    # -- SHOW DATA & COLORBARS -- #
    # To avoid infinite decimals
    ticks_x = np.around(np.arange(start=0, stop=(x_ticks-1)/res, step=(x_ticks/res)/5), decimals=1)
    ticks_y = np.around(np.arange(start=0, stop=(y_ticks-1)/res, step=(y_ticks/res)/5), decimals=1)

    im_default = ax_def.imshow(data, cmap = 'afmhot', aspect='equal', origin='lower')
    ax_def.set_xticks(ticks_x)
    ax_def.set_xticklabels(np.arange(start=0, stop=(x_ticks - 1), step=x_ticks/5))
    ax_def.set_yticks(ticks_y)
    ax_def.set_yticklabels(np.arange(start=0, stop=(y_ticks - 1), step=y_ticks/5))
    cbar_def = plt.colorbar(im_default, ax=ax_def)

    im_fixed = ax_fix.imshow(data, cmap = 'afmhot', aspect='equal', origin='lower',
                             vmin=mesh_min, vmax=mesh_max)
    ax_fix.set_xticks(ticks_x)
    ax_fix.set_xticklabels(np.arange(start=0, stop=(x_ticks - 1), step=x_ticks/5))
    ax_fix.set_yticks(ticks_y)
    ax_fix.set_yticklabels(np.arange(start=0, stop=(y_ticks - 1), step=y_ticks/5))
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

    power_t = Slider(ax_Pt, 'PowerTransmitter (W)', 0.00001, 100.0, 1.0, valstep=1.0)
    gain_t = Slider(ax_Gt, 'GainTransmitter (W)', 0.00001, 100.0, 1.0, valstep=0.5)
    gain_r = Slider(ax_Gr, 'GainReceiver (W)', 0.00001, 100.0, 1.0, valstep=0.5)
    freq = Slider(ax_Fq, 'Frequency (GHz)', 0.00001, 10.0, fr.FREQ_WIFI/(10**9), valstep=0.1)
    losses_factor = Slider(ax_l, 'OtherLosses [L]', 1.0, 10.0, 1.0, valstep=0.1)
    loss_exp = Slider(ax_n, 'PathLossExponent [n]', 1.6, 6.0, 2.0, valstep=0.1)
    res_sl = Slider(ax_resolution, 'MapResolution', 0.5, world_size[0]/2, res, valstep=0.5)
    world_sz_sl = Slider(ax_world_sz, 'MapSize (AxA)', 10.0, 100.0, world_size[0], valstep=1.0)

    power_t.on_changed(update)
    gain_t.on_changed(update)
    gain_r.on_changed(update)
    freq.on_changed(update)
    losses_factor.on_changed(update)
    loss_exp.on_changed(update)
    res_sl.on_changed(update_res_sz)
    world_sz_sl.on_changed(update_res_sz)

    # -- CHECKBUTTON -- #
    fixed_selected = False
    button_ax = fig.add_axes([0.15, 0.4, 0.05, 0.05])
    check = CheckButtons(button_ax, ["Fixed Lim"], [fixed_selected])
    check.on_clicked(checkbox)

    # -- SET BUTTON -- #
    button_ax = fig.add_axes([0.9, 0.04, 0.025, 0.025])
    b_set_res_and_sz = Button(button_ax, 'Set', color="orange")
    b_set_res_and_sz.on_clicked(button_function)

    # -- FULLSCREEN (press q to quit) -- #
    mng = plt.get_current_fig_manager()
    mng.full_screen_toggle()
    plt.show()
